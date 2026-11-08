#pragma once

#include "exec.cpp"
#include "hazard_unit.cpp"
#include "memory_unit.cpp"
#include <vector>

class Pipeline {
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
        uint64_t rawHazards        = 0;
        uint64_t fpOps             = 0;
        uint64_t ifStallCycles     = 0;
        uint64_t memStallCycles    = 0;
        uint64_t branches          = 0;
        uint64_t mispredicts       = 0;
        uint64_t exceptionsTaken   = 0;
        uint64_t interruptsTaken   = 0;
        uint64_t lastInterruptRetired = 0;
    };

    Pipeline() : Pipeline(Config()) {}
    explicit Pipeline(const Config& cfg)
        : cfg_(cfg), mem_(cfg.dmemBytes, cfg.imemBytes, cfg.memory), mau_(mem_, cfg.dmemBytes),
          hz_(cfg.predictor) {}
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("Pipeline::loadProgram: program does not fit");
        mem_.loadInstructions(baseBytes, words);
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;

        if (maRw_.valid) {
            if (maRw_.fault != Fault::None) {
                if (hz_.interrupts.canTakeException()) {
                    ++stats_.exceptionsTaken;
                    takeTrap(hz_.interrupts.enterException(static_cast<unsigned>(maRw_.fault), maRw_.pc, maRw_.faultValue));
                    return;
                }
                fault_ = maRw_.fault;
                beginDrain();
                return;
            }
            if (maRw_.regWrite) {
                if (maRw_.destFp)                 fregs_[maRw_.rd] = maRw_.value;
                else if (maRw_.rd != ISA::ZERO)   regs_[maRw_.rd] = maRw_.value;
            }
            if (maRw_.fpFlags) hz_.interrupts.orFpFlags(maRw_.fpFlags);
            if (maRw_.aluFlags) hz_.interrupts.setStatusBits(maRw_.aluFlags);
            ++stats_.retired;
            if (maRw_.halt) { beginDrain(); return; }
            const int line = hz_.interrupts.pendingInterrupt();
            if (line >= 0) {
                ++stats_.interruptsTaken;
                stats_.lastInterruptRetired = stats_.retired;
                takeTrap(hz_.interrupts.enterInterrupt(static_cast<unsigned>(line), maRw_.nextPc));
                return;
            }
        }

        bool squash = false, frozen = false, released = false;
        uint32_t maLatency = 1;
        MaRw nextMaRw;
        if (maHoldValid_) {
            if (--maWait_ > 0) { frozen = true; ++stats_.memStallCycles; }
            else               { nextMaRw = maHold_; maHoldValid_ = false; released = true; }
        }
        if (!frozen && !released && exMa_.valid) {
            nextMaRw.valid    = true;
            nextMaRw.pc       = exMa_.pc;
            nextMaRw.nextPc   = exMa_.nextPc;
            nextMaRw.rd       = exMa_.rd;
            nextMaRw.regWrite = exMa_.regWrite;
            nextMaRw.halt     = exMa_.halt;
            nextMaRw.fault    = exMa_.fault;
            nextMaRw.faultValue = exMa_.faultValue;
            nextMaRw.value    = exMa_.value;
            nextMaRw.destFp   = exMa_.destFp;
            nextMaRw.fpFlags  = exMa_.fpFlags;
            nextMaRw.aluFlags = exMa_.aluFlags;
            if (exMa_.fault == Fault::None) {
                if (exMa_.memRead || exMa_.memWrite) {
                    const MemoryAccessUnit::Result m = mau_.access(exMa_.memWrite, exMa_.value, exMa_.storeData);
                    if (m.fault) {
                        nextMaRw.fault    = Fault::MemoryFault;
                        nextMaRw.faultValue = exMa_.value;
                        nextMaRw.regWrite = false;
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
            nextMaRw = MaRw();
            frozen = true;
            ++stats_.memStallCycles;
        }
        if (frozen) { maRw_ = nextMaRw; return; }

        ExMa nextExMa;
        bool stallFetch = false, stopFetch = false, redirect = false;
        uint64_t target = 0;

        if (busy_.active) {
            if (--busy_.remaining == 0) { nextExMa = busy_.result; busy_.active = false; }
            else                        { stallFetch = true; ++stats_.execBusyCycles; }
        } else if (ifDe_.valid) {
            decodeExecute(nextExMa, stopFetch, redirect, target, stallFetch);
        }

        IfDe nextIfDe;
        if (squash || stopFetch) {
            fetchEnabled_ = false;
            ifWait_ = 0;
            if (squash) { nextExMa = ExMa(); busy_.active = false; }
        } else if (redirect) {
            pc_ = target;
            fetchEnabled_ = true;
            ifWait_ = 0;
        } else if (stallFetch) {
            nextIfDe = ifDe_;
        } else if (ifWait_ > 0) {
            ++stats_.ifStallCycles;
            if (--ifWait_ == 0) nextIfDe = pendingIf_;
        } else if (fetchEnabled_) {
            uint32_t latency = 1;
            IfDe fetched = fetch(latency);
            if (latency > 1) { pendingIf_ = fetched; ifWait_ = latency - 1; }
            else             { nextIfDe = fetched; }
        }

        maRw_ = nextMaRw;
        exMa_ = nextExMa;
        ifDe_ = nextIfDe;
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    const ForwardingUnit::Stats&   forwardStats()   const { return hz_.forwarding.stats(); }
    const InterlockUnit::Stats&    interlockStats() const { return hz_.interlock.stats(); }
    const RLBranchPredictor::Stats& predictorStats() const { return hz_.predictor.stats(); }
    const InterruptHandler& specialRegs() const { return hz_.interrupts; }
    void raiseInterrupt(unsigned line) { hz_.interrupts.raise(line); }
    uint64_t     pc()     const { return pc_; }
    Word         reg(unsigned i) const { return regs_[i]; }
    Word         freg(unsigned i) const { return fregs_[i]; }
    std::vector<Word> dmem() const { return mem_.dataSnapshot(); }
    void pokeDmem(size_t wordIndex, Word value) {
        if (wordIndex >= cfg_.dmemBytes / 8) throw std::out_of_range("Pipeline::pokeDmem");
        mem_.pokeData(wordIndex * 8, value);
    }
    const HierarchicalMemorySystem& memory() const { return mem_; }

private:
    struct IfDe {
        bool valid = false, fetchFault = false, hasPred = false;
        uint64_t pc = 0, predNext = 0;
        RLBranchPredictor::Prediction pred;
        ISA::InstrWord word = 0;
    };
    struct ExMa {
        bool valid = false, regWrite = false, memRead = false, memWrite = false, halt = false;
        bool destFp = false;
        uint8_t fpFlags = 0, aluFlags = 0;
        uint64_t pc = 0, nextPc = 0, faultValue = 0;
        uint8_t  rd = 0;
        Word     value = 0;
        Word     storeData = 0;
        Fault    fault = Fault::None;
    };
    struct MaRw {
        bool valid = false, regWrite = false, halt = false, destFp = false;
        uint8_t fpFlags = 0, aluFlags = 0;
        uint64_t pc = 0, nextPc = 0, faultValue = 0;
        uint8_t  rd = 0;
        Word     value = 0;
        Fault    fault = Fault::None;
    };
    struct Busy {
        bool active = false;
        uint32_t remaining = 0;
        ExMa result;
    };

    void beginDrain() { halted_ = true; }

    void takeTrap(uint64_t handler) {
        ifWait_ = 0;
        pendingIf_ = IfDe();
        maHoldValid_ = false;
        maWait_ = 0;
        exMa_ = ExMa();
        ifDe_ = IfDe();
        maRw_ = MaRw();
        busy_.active = false;
        pc_ = handler;
        fetchEnabled_ = true;
    }

    static ForwardingUnit::Producer producer(const ExMa& e) {
        return {e.valid, e.regWrite, e.memRead, e.rd, e.value, e.destFp};
    }
    static ForwardingUnit::Producer producer(const MaRw& m) {
        return {m.valid, m.regWrite, false, m.rd, m.value, m.destFp};
    }

    IfDe fetch(uint32_t& latency) {
        IfDe r;
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

    void decodeExecute(ExMa& out, bool& stopFetch, bool& redirect, uint64_t& target, bool& stallFetch) {
        out.valid = true;
        out.pc    = ifDe_.pc;
        out.nextPc = ifDe_.pc + 4;

        ISA::InstructionFields f;
        Fault bad = Fault::None;
        if (ifDe_.fetchFault) {
            bad = Fault::FetchFault;
        } else {
            try { f = ISA::decode(ifDe_.word); bad = Exec::validate(f); }
            catch (const std::invalid_argument&) { bad = Fault::IllegalInstruction; }
        }
        if (bad != Fault::None) {
            out.fault = bad;
            out.faultValue = (bad == Fault::FetchFault) ? ifDe_.pc : ifDe_.word;
            stopFetch = true;
            return;
        }
        if (f.opcode == ISA::Opcode::HALT) { out.halt = true; stopFetch = true; return; }

        if (f.opcode == ISA::Opcode::RETI && !hz_.interrupts.inTrap()) { out.fault = Fault::IllegalInstruction; out.faultValue = ifDe_.word; stopFetch = true; return; }

        const Exec::RegUse u = Exec::regUse(f);

        InterlockUnit::Query q;
        q.use = &u;
        q.isSerializing = Exec::isSerializing(f.opcode);
        q.exMa = producer(exMa_);
        const InterlockUnit::Stall stall = hz_.interlock.check(q);
        if (stall != InterlockUnit::Stall::None) {
            out = ExMa();
            stallFetch = true;
            return;
        }

        if (ForwardingUnit::matches(producer(exMa_), exMa_.rd, exMa_.destFp) &&
            ((u.readsA && u.a == exMa_.rd && u.aFp == exMa_.destFp) ||
             (u.readsB && u.b == exMa_.rd && u.bFp == exMa_.destFp)))
            ++stats_.rawHazards;

        auto readReg = [&](uint8_t r, bool fp) -> Word {
            if (!fp && r == ISA::ZERO) return 0;
            if (!fp && r == ISA::PC)   return ifDe_.pc;
            switch (hz_.forwarding.select(r, fp, producer(exMa_), producer(maRw_))) {
                case ForwardingUnit::Source::ExMa: return exMa_.value;
                case ForwardingUnit::Source::MaRw: return maRw_.value;
                default:                           return fp ? fregs_[r] : regs_[r];
            }
        };
        const Word a = u.readsA ? readReg(u.a, u.aFp) : 0;
        const Word b = u.readsB ? readReg(u.b, u.bFp) : 0;
        const Word imm = static_cast<Word>(static_cast<int64_t>(f.immediate));

        out.regWrite = u.writes;
        out.rd       = u.dest;
        out.destFp   = u.destFp;
        uint32_t latency = 1;

        switch (f.type()) {
            case ISA::InstructionType::ALU: {
                Exec::alu(f.opcode, a, f.modifier ? imm : b, out.value, out.aluFlags);
                if (Exec::isLongOp(f.opcode)) latency = cfg_.longOpCycles;
                break;
            }
            case ISA::InstructionType::FP: {
                const unsigned rm = hz_.interrupts.roundingMode();
                if (Exec::fpRoundingIllegal(f.opcode, rm)) {
                    out.regWrite = false;
                    out.fault = Fault::IllegalInstruction;
                    out.faultValue = ifDe_.word;
                    stopFetch = true;
                    return;
                }
                const FPU::Op op = Exec::fpOpOf(f.opcode);
                uint8_t fl = 0;
                out.value = FPU::execute(op, a, b, rm, fl);
                out.fpFlags = fl;
                latency = FPU::latency(op, cfg_.fp);
                ++stats_.fpOps;
                break;
            }
            case ISA::InstructionType::MEMORY:
                out.value = a + imm;
                if (f.opcode == ISA::Opcode::LOAD || f.opcode == ISA::Opcode::FLOAD) out.memRead = true;
                else { out.memWrite = true; out.storeData = b; }
                break;
            case ISA::InstructionType::BRANCH:
            {
                const bool taken = Exec::branchTaken(f.opcode, a, b);
                if (taken) {
                    ++stats_.takenBranches;
                    out.nextPc = (f.opcode == ISA::Opcode::RET)
                               ? a
                               : ifDe_.pc + static_cast<Word>(static_cast<int64_t>(f.immediate) * 4);
                }
                if (ifDe_.hasPred) { ++stats_.branches; hz_.predictor.update(ifDe_.pred, taken); }
                if (f.opcode == ISA::Opcode::CALL) out.value = ifDe_.pc + 4;
                break;
            }
            case ISA::InstructionType::SYSTEM:
                if (f.opcode == ISA::Opcode::RDSR)      out.value = hz_.interrupts.read(static_cast<unsigned>(f.immediate));
                else if (f.opcode == ISA::Opcode::WRSR) hz_.interrupts.write(static_cast<unsigned>(f.immediate), a);
                else if (f.opcode == ISA::Opcode::RETI) out.nextPc = hz_.interrupts.returnFromTrap();
                break;
            default: break;
        }
        if (latency > 1) {
            busy_.active    = true;
            busy_.remaining = latency - 1;
            busy_.result    = out;
            out             = ExMa();
            stallFetch      = true;
            ++stats_.execBusyCycles;
        }
        if (out.valid && out.nextPc != ifDe_.predNext) {
            redirect = true;
            target = out.nextPc;
            ++stats_.mispredicts;
        }
    }

    Config cfg_;
    mutable HierarchicalMemorySystem mem_;
    MemoryAccessUnit            mau_;
    HazardUnit                  hz_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;

    IfDe ifDe_;
    ExMa exMa_;
    MaRw maRw_;
    Busy busy_;
    bool     maHoldValid_ = false;
    uint32_t maWait_ = 0, ifWait_ = 0;
    MaRw     maHold_;
    IfDe     pendingIf_;
    Stats stats_;
};
