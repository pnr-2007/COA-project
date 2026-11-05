#pragma once

// UnifiedPipeline: one pipeline model with a selectable depth.
//
//   PipelineDepth::Four  IF | DE+EX | MA | RW
//   PipelineDepth::Six   IF | ID | EX | MEM1 | MEM2 | WB
//
// Everything that does not depend on the depth is written once and shared:
// configuration, statistics, register files, memory system, microcode decode,
// fetch + branch prediction, writeback/commit, trap entry, the FP/ALU/branch/
// system execution core and the public API.  Only the stage-to-stage movement
// (hazard checks, forwarding, memory stages, stall/flush rules) is per depth.
//
// The depth-specific behaviour is cycle-exact with the stand-alone Pipeline4 and
// Pipeline6 classes (see unified_equiv_test.cpp, which compares them cycle by cycle).

#include "exec.cpp"
#include "forwarding_unit.cpp"
#include "interlock_unit.cpp"
#include "forwarding_unit6.cpp"
#include "interlock_unit6.cpp"
#include "control_unit6.cpp"
#include "branch_predictor.cpp"
#include "interrupt_handler.cpp"
#include "memory_unit.cpp"
#include "memory_access6.cpp"
#include "horizontal_microcode6.cpp"
#include <vector>

enum class PipelineDepth : uint8_t { Four = 4, Six = 6 };

// Hazard hardware for both depths. The predictor, return stack and special
// registers are shared; forwarding/interlock exist in a 4-stage and 6-stage form.
class UnifiedHazardUnit {
public:
    struct Config {
        PipelineDepth depth = PipelineDepth::Four;
        Stage6 branchResolveStage = Stage6::EX;   // 6-stage only
        RLBranchPredictor::Config predictor;
    };

    UnifiedHazardUnit() : UnifiedHazardUnit(Config()) {}
    explicit UnifiedHazardUnit(const Config& c) : predictor(c.predictor), cfg_(c) {}

    // Bubbles after a mispredicted branch: 4-stage resolves in DE+EX (1), 6-stage in EX (2).
    unsigned mispredictPenalty() const {
        if (cfg_.depth == PipelineDepth::Four) return 1;
        return ControlUnit6::redirectPenalty(cfg_.branchResolveStage);
    }
    const Config& config() const { return cfg_; }

    ForwardingUnit    forwarding4;
    InterlockUnit     interlock4;
    ForwardingUnit6   forwarding6;
    InterlockUnit6    interlock6;
    RLBranchPredictor predictor;
    ReturnStack       returnStack;
    InterruptHandler  interrupts;

private:
    Config cfg_;
};

class UnifiedPipeline {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    struct Config {
        PipelineDepth depth   = PipelineDepth::Four;
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

    // Forwarding counters for either depth. The 4-stage fills fromExMa/fromMaRw,
    // the 6-stage fills fromMem1/fromMem2/fromWb/lateRegFile/blocked; the rest stay 0.
    struct ForwardStats {
        uint64_t fromExMa = 0, fromMaRw = 0;
        uint64_t fromMem1 = 0, fromMem2 = 0, fromWb = 0, lateRegFile = 0, blocked = 0;
    };

    // Interlock counters for either depth (loadUseByBubbles is 6-stage only).
    struct InterlockStats {
        uint64_t loadUse = 0, serialize = 0;
        uint64_t loadUseByBubbles[3] = {0, 0, 0};
    };

    UnifiedPipeline() : UnifiedPipeline(Config()) {}
    explicit UnifiedPipeline(const Config& cfg)
        : cfg_(cfg), mem_(cfg.dmemBytes, cfg.imemBytes, cfg.memory), mau_(mem_, cfg.dmemBytes),
          ms_(mau_, cfg.dmemBytes), hz_(makeHazardConfig(cfg)) {}
    UnifiedPipeline(const UnifiedPipeline&) = delete;
    UnifiedPipeline& operator=(const UnifiedPipeline&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("UnifiedPipeline::loadProgram: program does not fit");
        mem_.loadInstructions(baseBytes, words);
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;
        if (four()) step4(); else step6();
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    PipelineDepth depth()  const { return cfg_.depth; }
    unsigned      stages() const { return four() ? 4u : 6u; }
    bool          halted() const { return halted_; }
    Fault         fault()  const { return fault_; }
    const Stats&  stats()  const { return stats_; }

    ForwardStats forwardStats() const {
        ForwardStats r;
        if (four()) {
            r.fromExMa = hz_.forwarding4.stats().fromExMa;
            r.fromMaRw = hz_.forwarding4.stats().fromMaRw;
        } else {
            const ForwardingUnit6::Stats& s = hz_.forwarding6.stats();
            r.fromMem1 = s.fromMem1; r.fromMem2 = s.fromMem2; r.fromWb = s.fromWb;
            r.lateRegFile = s.lateRegFile; r.blocked = s.blocked;
        }
        return r;
    }
    InterlockStats interlockStats() const {
        InterlockStats r;
        if (four()) {
            r.loadUse = hz_.interlock4.stats().loadUse;
            r.serialize = hz_.interlock4.stats().serialize;
        } else {
            const InterlockUnit6::Stats& s = hz_.interlock6.stats();
            r.loadUse = s.loadUse; r.serialize = s.serialize;
            for (int i = 0; i < 3; ++i) r.loadUseByBubbles[i] = s.loadUseByBubbles[i];
        }
        return r;
    }
    const RLBranchPredictor::Stats& predictorStats() const { return hz_.predictor.stats(); }
    const InterruptHandler& specialRegs() const { return hz_.interrupts; }
    const HorizontalMicrocodeUnit6& microcode() const { return ucode_; }
    unsigned mispredictPenalty() const { return hz_.mispredictPenalty(); }
    void raiseInterrupt(unsigned line) { hz_.interrupts.raise(line); }
    uint64_t pc()  const { return pc_; }
    Word reg(unsigned i)  const { return regs_[i]; }
    Word freg(unsigned i) const { return fregs_[i]; }
    std::vector<Word> dmem() const { return mem_.dataSnapshot(); }
    void pokeDmem(size_t wordIndex, Word value) {
        if (wordIndex >= cfg_.dmemBytes / 8) throw std::out_of_range("UnifiedPipeline::pokeDmem");
        mem_.pokeData(wordIndex * 8, value);
    }
    const HierarchicalMemorySystem& memory() const { return mem_; }

private:
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

    bool four() const { return cfg_.depth == PipelineDepth::Four; }

    static UnifiedHazardUnit::Config makeHazardConfig(const Config& c) {
        UnifiedHazardUnit::Config h;
        h.depth = c.depth;
        h.predictor = c.predictor;
        return h;
    }

    // =====================================================================
    // Shared by both depths
    // =====================================================================

    // Flush everything and jump to a trap handler.
    void takeTrap(uint64_t handler) {
        ifWait_ = 0;
        pendingIf_ = IfId();
        maHoldValid_ = false;
        maWait_ = 0;
        m2Started_ = false;
        m2Wait_ = 0;
        busy_ = Busy();
        exMa_ = maRw_ = Slot();
        idEx_ = exM1_ = m1M2_ = m2Wb_ = Slot();
        ifId_ = IfId();
        pc_ = handler;
        fetchEnabled_ = true;
    }

    // Writeback/commit of the oldest instruction. Returns true when the cycle is over
    // (trap taken, machine halted, or fault halt) and the rest of step() must be skipped.
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

    // Common first half of decode: translate through the microcode, turn faults/HALT/
    // into a terminal slot. Returns true when `out` is final (stopFetch set).
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
    // the 6-stage after it (a serializing RETI may stall first); the order is kept per depth.
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

    // The execute core: ALU / FPU / branch / system semantics. `computeMemAddr` is true for
    // the 4-stage DE+EX (address and store data are produced there); the 6-stage does that
    // in MEM1/MEM2 instead.
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

    // Fetch/IF-side update shared by both depths once the back end has decided
    // squash/stop/redirect/hold. `hold` keeps the current IF/ID contents (fetch frozen).
    void advanceFetch(IfId& nextId, bool killIf, bool stopFetch, bool redirect, uint64_t target, bool hold) {
        if (killIf || stopFetch) {
            fetchEnabled_ = false;
            ifWait_ = 0;
            if (!four()) {
                pendingIf_ = IfId();
                if (killIf) nextId = IfId();
            }
        } else if (redirect) {
            pc_ = target;
            fetchEnabled_ = true;
            ifWait_ = 0;
            if (!four()) { pendingIf_ = IfId(); nextId = IfId(); }
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

    // =====================================================================
    // 4-stage:  IF | DE+EX | MA | RW
    // =====================================================================

    static ForwardingUnit::Producer producer4(const Slot& e, bool inRw) {
        return {e.valid, e.regWrite, inRw ? false : e.memRead, e.rd, e.value, e.destFp};
    }

    void step4() {
        if (commit(maRw_)) return;

        bool squash = false, frozen = false, released = false;
        uint32_t maLatency = 1;
        Slot nextMaRw;
        if (maHoldValid_) {
            if (--maWait_ > 0) { frozen = true; ++stats_.memStallCycles; }
            else               { nextMaRw = maHold_; maHoldValid_ = false; released = true; }
        }
        if (!frozen && !released && exMa_.valid) {
            nextMaRw = exMa_;
            nextMaRw.memRead = nextMaRw.memWrite = false;   // not carried into RW
            if (exMa_.fault == Fault::None) {
                if (exMa_.memRead || exMa_.memWrite) {
                    const MemoryAccessUnit::Result m = mau_.access(exMa_.memWrite, exMa_.value, exMa_.storeData);
                    if (m.fault) {
                        nextMaRw.fault      = Fault::MemoryFault;
                        nextMaRw.faultValue = exMa_.value;
                        nextMaRw.regWrite   = false;
                        squash = true;
                    } else {
                        if (exMa_.memRead) nextMaRw.value = m.data;
                        maLatency = m.cycles;
                    }
                }
            }
        }

        if (maLatency > 1 && !squash) {
            maHold_ = nextMaRw;
            maHoldValid_ = true;
            maWait_ = maLatency - 1;
            nextMaRw = Slot();
            frozen = true;
            ++stats_.memStallCycles;
        }
        if (frozen) { maRw_ = nextMaRw; return; }

        Slot nextExMa;
        bool stallFetch = false, stopFetch = false, redirect = false;
        uint64_t target = 0;

        if (busy_.active) {
            if (--busy_.remaining == 0) { nextExMa = busy_.result; busy_.active = false; }
            else                        { stallFetch = true; ++stats_.execBusyCycles; }
        } else if (ifId_.valid) {
            decodeExecute4(nextExMa, stopFetch, redirect, target, stallFetch);
        }

        IfId nextIf;
        if (squash) { nextExMa = Slot(); busy_.active = false; }
        if (!(squash || stopFetch) && !redirect && stallFetch) nextIf = ifId_;
        advanceFetch(nextIf, false, squash || stopFetch, redirect, target, stallFetch);

        maRw_ = nextMaRw;
        exMa_ = nextExMa;
        ifId_ = nextIf;
    }

    void decodeExecute4(Slot& out, bool& stopFetch, bool& redirect, uint64_t& target, bool& stallFetch) {
        const HorizontalMicrocodeUnit6::Result mc = ucode_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
        if (decodeFront(out, mc, stopFetch)) return;
        if (checkStrayReti(out, stopFetch)) return;
        const ControlWord& w = mc.word;

        const Exec::RegUse u = regUse(w);

        InterlockUnit::Query q;
        q.use = &u;
        q.isSerializing = w.serializing;
        q.exMa = producer4(exMa_, false);
        if (hz_.interlock4.check(q) != InterlockUnit::Stall::None) {
            out = Slot();
            stallFetch = true;
            return;
        }

        if (ForwardingUnit::matches(producer4(exMa_, false), exMa_.rd, exMa_.destFp) &&
            ((u.readsA && u.a == exMa_.rd && u.aFp == exMa_.destFp) ||
             (u.readsB && u.b == exMa_.rd && u.bFp == exMa_.destFp)))
            ++stats_.rawHazards;

        auto readReg = [&](uint8_t r, bool fp) -> Word {
            if (!fp && r == ISA::ZERO) return 0;
            if (!fp && r == ISA::PC)   return ifId_.pc;
            switch (hz_.forwarding4.select(r, fp, producer4(exMa_, false), producer4(maRw_, true))) {
                case ForwardingUnit::Source::ExMa: return exMa_.value;
                case ForwardingUnit::Source::MaRw: return maRw_.value;
                default:                           return fp ? fregs_[r] : regs_[r];
            }
        };
        const Word a = w.readsA ? readReg(w.srcA, w.srcAFp) : 0;
        const Word b = w.readsB ? readReg(w.srcB, w.srcBFp) : 0;

        out.regWrite = w.regWrite;
        out.rd       = w.dest;
        out.destFp   = w.destFp;
        uint32_t latency = 1;

        if (executeCore(out, a, b, latency, true) == Core::IllegalRounding) {
            out.regWrite = false;
            out.fault = Fault::IllegalInstruction;
            out.faultValue = ifId_.word;
            stopFetch = true;
            return;
        }
        if (latency > 1) {
            busy_.active    = true;
            busy_.remaining = latency - 1;
            busy_.result    = out;
            out             = Slot();
            stallFetch      = true;
            ++stats_.execBusyCycles;
        }
        if (out.valid && out.nextPc != ifId_.predNext) {
            redirect = true;
            target = out.nextPc;
            ++stats_.mispredicts;
        }
    }

    // =====================================================================
    // 6-stage:  IF | ID | EX | MEM1 | MEM2 | WB
    // =====================================================================

    static ForwardingUnit6::Producer prod6(const Slot& s, bool inWb) {
        ForwardingUnit6::Producer p;
        p.valid = s.valid;
        p.regWrite = s.regWrite;
        p.fp = s.destFp;
        p.rd = s.rd;
        p.value = s.value;
        p.ready = inWb || !s.memRead;
        p.readyStage = s.memRead ? Stage6::MEM2 : Stage6::EX;
        return p;
    }

    Word resolve6(uint8_t r, bool fp, Stage6 need, uint64_t pc, const Slot& m1, const Slot& m2, const Slot& wb) {
        if (!fp && r == ISA::ZERO) return 0;
        if (!fp && r == ISA::PC)   return pc;
        ForwardingUnit6::Ahead ahead{prod6(m1, false), prod6(m2, false), prod6(wb, true)};
        const ForwardingUnit6::Selection sel = hz_.forwarding6.select(r, fp, need, ahead);
        switch (sel.source) {
            case ForwardingUnit6::Source::FromMem1: return m1.value;
            case ForwardingUnit6::Source::FromMem2: return m2.value;
            case ForwardingUnit6::Source::FromWb:   return wb.value;
            default:                                return fp ? fregs_[r] : regs_[r];
        }
    }

    void step6() {
        const Slot oldM1 = exM1_, oldM2 = m1M2_, oldWb = m2Wb_;

        if (commit(m2Wb_)) return;

        bool killM1 = false, killEx = false, killId = false, killIf = false;

        Slot nextWb, nextM2, nextM1, nextEx;
        IfId nextId;
        bool frozen = false;

        if (m1M2_.valid) {
            if (m2Started_) {
                if (--m2Wait_ > 0) { frozen = true; ++stats_.memStallCycles; }
                else               { m2Started_ = false; nextWb = m1M2_; }
            } else {
                Slot s = m1M2_;
                bool moveOn = true;
                if (s.fault == Fault::None && (s.memRead || s.memWrite)) {
                    Word sd = 0;
                    if (s.memWrite) sd = resolve6(s.w.srcB, s.w.srcBFp, Stage6::MEM2, s.pc, oldM1, oldM2, oldWb);
                    const MemoryStages6::Mem2Result m = ms_.access(s.memWrite, s.value, sd);
                    if (m.fault) {
                        s.fault = Fault::MemoryFault;
                        s.faultValue = s.value;
                        s.regWrite = false;
                        killM1 = killEx = killId = killIf = true;
                    } else {
                        if (s.memRead) s.value = m.data;
                        if (m.cycles > 1) {
                            m1M2_ = s;
                            m2Started_ = true;
                            m2Wait_ = m.cycles - 1;
                            frozen = true;
                            ++stats_.memStallCycles;
                            moveOn = false;
                        }
                    }
                }
                if (moveOn) nextWb = s;
            }
        }

        if (frozen) { m2Wb_ = Slot(); return; }

        bool exHold = false;

        if (exM1_.valid && !killM1) {
            Slot s = exM1_;
            if (s.fault == Fault::None && (s.memRead || s.memWrite)) {
                const Word base = resolve6(s.w.srcA, s.w.srcAFp, Stage6::MEM1, s.pc, oldM1, oldM2, oldWb);
                const MemoryStages6::Mem1Result r = ms_.address(base, s.w.immediate);
                s.value = r.addr;
                if (r.fault) {
                    s.fault = Fault::MemoryFault;
                    s.faultValue = r.addr;
                    s.regWrite = false;
                    killEx = killId = killIf = true;
                }
            }
            nextM2 = s;
        }

        bool redirect = false, stopFetch = false;
        uint64_t target = 0;

        if (busy_.active && killEx) busy_.active = false;
        if (busy_.active) {
            if (--busy_.remaining == 0) { nextM1 = busy_.result; busy_.active = false; }
            else                        { exHold = true; ++stats_.execBusyCycles; }
        } else if (idEx_.valid && !killEx) {
            execute6(nextM1, exHold, redirect, target, killId, killIf, oldM1, oldM2, oldWb);
        }
        if (killEx) nextM1 = Slot();

        bool idStall = false;
        if (exHold) {
            nextId = ifId_;
            nextEx = idEx_;
        } else if (killId || redirect) {
        } else if (ifId_.valid) {
            decode6(nextEx, stopFetch, idStall);
        }
        if (idStall) nextId = ifId_;

        if (killId) nextEx = Slot();

        advanceFetch(nextId, killIf, stopFetch, redirect, target, exHold || idStall);

        m2Wb_ = nextWb;
        m1M2_ = nextM2;
        exM1_ = nextM1;
        idEx_ = nextEx;
        ifId_ = nextId;
    }

    void decode6(Slot& out, bool& stopFetch, bool& stall) {
        const HorizontalMicrocodeUnit6::Result mc = ucode_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
        if (decodeFront(out, mc, stopFetch)) return;
        const ControlWord& w = mc.word;

        const Exec::RegUse u = regUse(w);

        InterlockUnit6::Query q;
        q.use = &u;
        q.needs = mc.needs;
        q.isSerializing = w.serializing;
        q.ex   = prod6(idEx_, false);
        q.mem1 = prod6(exM1_, false);
        q.mem2 = prod6(m1M2_, false);
        if (hz_.interlock6.check(q) != InterlockUnit6::Stall::None) {
            out = Slot();
            stall = true;
            return;
        }
        if (checkStrayReti(out, stopFetch)) return;

        out.regWrite = w.regWrite;
        out.rd       = w.dest;
        out.destFp   = w.destFp;
        out.memRead  = w.memRead;
        out.memWrite = w.memWrite;
    }

    void execute6(Slot& out, bool& hold, bool& redirect, uint64_t& target, bool& killId, bool& killIf,
                  const Slot& m1, const Slot& m2, const Slot& wb) {
        out = idEx_;
        const ControlWord& w = out.w;
        if (out.fault != Fault::None || out.halt) return;

        const bool memOp = w.memRead || w.memWrite;
        const Word a = (w.readsA && !memOp) ? resolve6(w.srcA, w.srcAFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
        const Word b = (w.readsB && !memOp) ? resolve6(w.srcB, w.srcBFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
        uint32_t latency = 1;

        if (executeCore(out, a, b, latency, false) == Core::IllegalRounding) {
            out.regWrite = false;
            out.fault = Fault::IllegalInstruction;
            out.faultValue = out.word;
            killId = killIf = true;
            return;
        }
        if (latency > 1) {
            busy_.active = true;
            busy_.remaining = latency - 1;
            busy_.result = out;
            out = Slot();
            hold = true;
            ++stats_.execBusyCycles;
            return;
        }
        if (out.nextPc != out.predNext) {
            redirect = true;
            target = out.nextPc;
            ++stats_.mispredicts;
        }
    }

    // =====================================================================
    // State
    // =====================================================================

    struct Busy {
        bool active = false;
        uint32_t remaining = 0;
        Slot result;
    };

    Config cfg_;
    mutable HierarchicalMemorySystem mem_;
    MemoryAccessUnit            mau_;
    MemoryStages6               ms_;
    UnifiedHazardUnit           hz_;
    HorizontalMicrocodeUnit6    ucode_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;

    // shared front end
    IfId     ifId_;
    IfId     pendingIf_;
    uint32_t ifWait_ = 0;
    Busy     busy_;                       // multi-cycle EX (DE+EX in the 4-stage)

    // 4-stage back end
    Slot     exMa_, maRw_;
    bool     maHoldValid_ = false;
    uint32_t maWait_ = 0;
    Slot     maHold_;

    // 6-stage back end
    Slot     idEx_, exM1_, m1M2_, m2Wb_;
    bool     m2Started_ = false;
    uint32_t m2Wait_ = 0;

    Stats stats_;
};

// Fixed-depth views, so code and tests written for one depth can use the unified model
// unchanged (default-constructed Config already selects the depth).
template <PipelineDepth D>
class FixedDepthPipeline : public UnifiedPipeline {
public:
    struct Config : UnifiedPipeline::Config {
        Config() { depth = D; }
    };
    FixedDepthPipeline() : UnifiedPipeline(Config()) {}
    explicit FixedDepthPipeline(const Config& c) : UnifiedPipeline(c) {}
};

using UnifiedPipeline4 = FixedDepthPipeline<PipelineDepth::Four>;
using UnifiedPipeline6 = FixedDepthPipeline<PipelineDepth::Six>;
