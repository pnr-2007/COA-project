#pragma once

// PipelineCore: everything the 4-stage and 6-stage pipelines have in common.
//   - Config / Stats / public API
//   - register files, memory system, microcode decode
//   - fetch + branch prediction (IF), commit (last stage), trap entry
//   - the ALU / FPU / branch / system execute core
// The depth-specific parts live in:
//   pipeline_four_stage.cpp   IF | DE+EX | MA | RW
//   pipeline_six_stage.cpp    IF | ID | EX | MEM1 | MEM2 | WB

#include "exec.cpp"
#include "branch_predictor.cpp"
#include "interrupt_handler.cpp"
#include "memory_unit.cpp"
#include "horizontal_microcode6.cpp"
#include <vector>

enum class PipelineDepth : uint8_t { Four = 4, Six = 6 };

class PipelineCore {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    struct Config {
        size_t   imemBytes    = 1 << 18;
        size_t   dmemBytes    = 1 << 19;
        uint32_t longOpCycles = 64;
        RLBranchPredictor::Config predictor;
        HierarchicalMemorySystem::Config memory;
        FPU::Latency fp;
    };

    struct Stats {
        uint64_t cycles            = 0;
        uint64_t retired           = 0;
        uint64_t takenBranches     = 0;
        uint64_t execBusyCycles    = 0;
        uint64_t rawHazards        = 0;   // 4-stage only (always 0 in 6-stage)
        uint64_t fpOps             = 0;
        uint64_t ifStallCycles     = 0;
        uint64_t memStallCycles    = 0;
        uint64_t branches          = 0;
        uint64_t mispredicts       = 0;
        uint64_t exceptionsTaken   = 0;
        uint64_t interruptsTaken   = 0;
        uint64_t lastInterruptRetired = 0;
    };

    // The 4-stage fills fromExMa/fromMaRw; the 6-stage fills the rest.
    struct ForwardStats {
        uint64_t fromExMa = 0, fromMaRw = 0;
        uint64_t fromMem1 = 0, fromMem2 = 0, fromWb = 0, lateRegFile = 0, blocked = 0;
    };

    // loadUseByBubbles is 6-stage only.
    struct InterlockStats {
        uint64_t loadUse = 0, serialize = 0;
        uint64_t loadUseByBubbles[3] = {0, 0, 0};
    };

    explicit PipelineCore(const Config& cfg)
        : cfg_(cfg), mem_(cfg.dmemBytes, cfg.imemBytes, cfg.memory), mau_(mem_, cfg.dmemBytes),
          hz_(cfg.predictor) {}
    virtual ~PipelineCore() = default;
    PipelineCore(const PipelineCore&) = delete;
    PipelineCore& operator=(const PipelineCore&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("Pipeline::loadProgram: program does not fit");
        mem_.loadInstructions(baseBytes, words);
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;
        advance();
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    // ---- depth-specific (implemented by the two parts) ----
    virtual PipelineDepth  depth() const = 0;
    virtual unsigned       mispredictPenalty() const = 0;
    virtual ForwardStats   forwardStats() const = 0;
    virtual InterlockStats interlockStats() const = 0;

    unsigned stages() const { return depth() == PipelineDepth::Four ? 4u : 6u; }
    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    const RLBranchPredictor::Stats& predictorStats() const { return hz_.predictor.stats(); }
    const InterruptHandler& specialRegs() const { return hz_.interrupts; }
    const HorizontalMicrocodeUnit6& microcode() const { return ucode_; }
    void raiseInterrupt(unsigned line) { hz_.interrupts.raise(line); }
    uint64_t pc()  const { return pc_; }
    Word reg(unsigned i)  const { return regs_[i]; }
    Word freg(unsigned i) const { return fregs_[i]; }
    std::vector<Word> dmem() const { return mem_.dataSnapshot(); }
    void pokeDmem(size_t wordIndex, Word value) {
        if (wordIndex >= cfg_.dmemBytes / 8) throw std::out_of_range("Pipeline::pokeDmem");
        mem_.pokeData(wordIndex * 8, value);
    }
    const HierarchicalMemorySystem& memory() const { return mem_; }

protected:
    // ---- pipeline registers (one layout for every stage of either depth) ----
    struct IfId {
        bool valid = false, fetchFault = false, hasPred = false;
        uint64_t pc = 0, predNext = 0;
        RLBranchPredictor::Prediction pred;
        ISA::InstrWord word = 0;
    };
    struct Slot {
        bool valid = false, regWrite = false, destFp = false, memRead = false, memWrite = false, halt = false;
        ControlWord w;
        uint8_t fpFlags = 0, aluFlags = 0;
        uint8_t rd = 0;
        uint64_t pc = 0, nextPc = 0, predNext = 0, faultValue = 0;
        bool hasPred = false;
        RLBranchPredictor::Prediction pred;
        Word value = 0;
        Word storeData = 0;   // 4-stage: store data captured in DE+EX
        ISA::InstrWord word = 0;
        Fault fault = Fault::None;
    };
    struct Busy {             // multi-cycle execute
        bool active = false;
        uint32_t remaining = 0;
        Slot result;
    };

    // One clock cycle of the back end + fetch (commit() is called first by the part).
    virtual void advance() = 0;
    // Drop everything in flight behind commit (called by takeTrap).
    virtual void clearBackEnd() = 0;

    // Shared hazard state: predictor, return stack, special registers.
    struct SharedHazard {
        explicit SharedHazard(const RLBranchPredictor::Config& c) : predictor(c) {}
        RLBranchPredictor predictor;
        ReturnStack       returnStack;
        InterruptHandler  interrupts;
    };

    // Flush everything and jump to a trap handler.
    void takeTrap(uint64_t handler) {
        ifWait_ = 0;
        pendingIf_ = IfId();
        ifId_ = IfId();
        clearBackEnd();
        pc_ = handler;
        fetchEnabled_ = true;
    }

    // Commit the oldest instruction. Returns true when the cycle is over (trap taken,
    // machine halted, or fault halt) and the rest of the cycle must be skipped.
    bool commit(const Slot& s) {
        if (!s.valid) return false;
        if (s.fault != Fault::None) {
            if (hz_.interrupts.canTakeException()) {
                ++stats_.exceptionsTaken;
                takeTrap(hz_.interrupts.enterException(static_cast<unsigned>(s.fault), s.pc, s.faultValue));
                return true;
            }
            fault_ = s.fault;
            halted_ = true;
            return true;
        }
        if (s.regWrite) {
            if (s.destFp)                 fregs_[s.rd] = s.value;
            else if (s.rd != ISA::ZERO)   regs_[s.rd] = s.value;
        }
        if (s.fpFlags)  hz_.interrupts.orFpFlags(s.fpFlags);
        if (s.aluFlags) hz_.interrupts.setStatusBits(s.aluFlags);
        ++stats_.retired;
        if (s.halt) { halted_ = true; return true; }
        const int line = hz_.interrupts.pendingInterrupt();
        if (line >= 0) {
            ++stats_.interruptsTaken;
            stats_.lastInterruptRetired = stats_.retired;
            takeTrap(hz_.interrupts.enterInterrupt(static_cast<unsigned>(line), s.nextPc));
            return true;
        }
        return false;
    }

    // IF stage: fetch, pre-decode and predict.
    IfId fetch(uint32_t& latency) {
        IfId r;
        r.valid = true;
        r.pc    = pc_;
        if ((pc_ & 3) != 0 || pc_ >= cfg_.imemBytes) {
            r.fetchFault  = true;
            fetchEnabled_ = false;
            return r;
        }
        const MemoryAccessUnit::Fetch fe = mau_.fetch(pc_);
        r.word = fe.word;
        latency = fe.cycles;
        uint64_t next = pc_ + 4;
        try {
            const ISA::InstructionFields f = ISA::decode(r.word);
            const uint64_t rel = pc_ + static_cast<Word>(static_cast<int64_t>(f.immediate) * 4);
            switch (f.opcode) {
                case ISA::Opcode::JEQ:
                case ISA::Opcode::JGT:
                case ISA::Opcode::JZ:
                case ISA::Opcode::JNZ:
                    r.pred = hz_.predictor.predict(pc_, f.immediate < 0);
                    r.hasPred = true;
                    if (r.pred.taken) next = rel;
                    break;
                case ISA::Opcode::JMP:  next = rel; break;
                case ISA::Opcode::CALL: hz_.returnStack.push(pc_ + 4); next = rel; break;
                case ISA::Opcode::RET: {
                    uint64_t t;
                    if (hz_.returnStack.top(t)) { next = t; hz_.returnStack.pop(); }
                    break;
                }
                default: break;
            }
        } catch (const std::invalid_argument&) {}
        r.predNext = next;
        pc_ = next;
        return r;
    }

    // First half of decode: microcode translation, then fault/HALT become terminal slots.
    // Returns true when `out` is final (stopFetch set).
    bool decodeFront(Slot& out, const HorizontalMicrocodeUnit6::Result& mc, bool& stopFetch) {
        const ControlWord& w = mc.word;
        out.valid    = true;
        out.w        = w;
        out.pc       = ifId_.pc;
        out.word     = ifId_.word;
        out.nextPc   = ifId_.pc + 4;
        out.predNext = ifId_.predNext;
        out.hasPred  = ifId_.hasPred;
        out.pred     = ifId_.pred;

        if (w.fault != Fault::None) {
            out.fault = w.fault;
            out.faultValue = w.faultValue;
            stopFetch = true;
            return true;
        }
        if (w.halt) { out.halt = true; stopFetch = true; return true; }
        return false;
    }

    // RETI outside a trap handler is illegal. The 4-stage checks it before the interlock,
    // the 6-stage after it (a serializing RETI may stall first); each part keeps its order.
    bool checkStrayReti(Slot& out, bool& stopFetch) {
        if (out.w.branch == BranchKind::RETI && !hz_.interrupts.inTrap()) {
            out.fault = Fault::IllegalInstruction;
            out.faultValue = ifId_.word;
            stopFetch = true;
            return true;
        }
        return false;
    }

    static Exec::RegUse regUse(const ControlWord& w) {
        Exec::RegUse u;
        u.readsA = w.readsA; u.a = w.srcA; u.aFp = w.srcAFp;
        u.readsB = w.readsB; u.b = w.srcB; u.bFp = w.srcBFp;
        u.writes = w.regWrite; u.dest = w.dest; u.destFp = w.destFp;
        return u;
    }

    enum class Core { Done, IllegalRounding };

    // ALU / FPU / branch / system semantics. `computeMemAddr` is true for the 4-stage DE+EX
    // (address and store data are produced there); the 6-stage does that in MEM1/MEM2.
    Core executeCore(Slot& out, Word a, Word b, uint32_t& latency, bool computeMemAddr) {
        const ControlWord& w = out.w;
        const Word imm = static_cast<Word>(static_cast<int64_t>(w.immediate));
        const bool memOp = w.memRead || w.memWrite;

        if (w.fpOp) {
            const unsigned rm = hz_.interrupts.roundingMode();
            if (w.fpRounds && !FPU::validRounding(rm)) return Core::IllegalRounding;
            uint8_t fl = 0;
            out.value = FPU::execute(w.fpKind, a, b, rm, fl);
            out.fpFlags = fl;
            latency = FPU::latency(w.fpKind, cfg_.fp);
            ++stats_.fpOps;
        } else if (w.alu != AluOp::NONE) {
            Exec::alu(w.opcode, a, w.immB ? imm : b, out.value, out.aluFlags);
            if (w.longOp) latency = cfg_.longOpCycles;
        } else if (memOp) {
            if (computeMemAddr) {
                out.value = a + imm;
                if (w.memRead) out.memRead = true;
                else { out.memWrite = true; out.storeData = b; }
            }
        } else if (w.branch != BranchKind::NONE && w.branch != BranchKind::RETI) {
            const bool taken = Exec::branchTaken(w.opcode, a, b);
            if (taken) {
                ++stats_.takenBranches;
                out.nextPc = (w.branch == BranchKind::RET) ? a : out.pc + imm * 4;
            }
            if (out.hasPred) { ++stats_.branches; hz_.predictor.update(out.pred, taken); }
            if (w.branch == BranchKind::CALL) out.value = out.pc + 4;
        } else if (w.readsSR) {
            out.value = hz_.interrupts.read(static_cast<unsigned>(w.immediate));
        } else if (w.writesSR) {
            hz_.interrupts.write(static_cast<unsigned>(w.immediate), a);
        } else if (w.branch == BranchKind::RETI) {
            out.nextPc = hz_.interrupts.returnFromTrap();
        }
        return Core::Done;
    }

    // IF-side update once the back end has decided squash/stop/redirect/hold.
    // `flushClearsPending`: the 6-stage also drops a fetch in progress and the IF/ID register.
    void advanceFetch(IfId& nextId, bool killIf, bool stopFetch, bool redirect, uint64_t target, bool hold,
                      bool flushClearsPending) {
        if (killIf || stopFetch) {
            fetchEnabled_ = false;
            ifWait_ = 0;
            if (flushClearsPending) {
                pendingIf_ = IfId();
                if (killIf) nextId = IfId();
            }
        } else if (redirect) {
            pc_ = target;
            fetchEnabled_ = true;
            ifWait_ = 0;
            if (flushClearsPending) { pendingIf_ = IfId(); nextId = IfId(); }
        } else if (hold) {
            // front end held
        } else if (ifWait_ > 0) {
            ++stats_.ifStallCycles;
            if (--ifWait_ == 0) nextId = pendingIf_;
        } else if (fetchEnabled_) {
            uint32_t latency = 1;
            IfId fetched = fetch(latency);
            if (latency > 1) { pendingIf_ = fetched; ifWait_ = latency - 1; }
            else             { nextId = fetched; }
        }
    }

    Config cfg_;
    mutable HierarchicalMemorySystem mem_;
    MemoryAccessUnit            mau_;
    SharedHazard                hz_;
    HorizontalMicrocodeUnit6    ucode_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;

    IfId     ifId_;
    IfId     pendingIf_;
    uint32_t ifWait_ = 0;
    Stats    stats_;
};
