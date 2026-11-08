#pragma once

#include "exec.cpp"
#include "hazard_unit6.cpp"
#include "memory_access6.cpp"
#include "horizontal_microcode6.cpp"
#include <vector>

class Pipeline6 {
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

    Pipeline6() : Pipeline6(Config()) {}
    explicit Pipeline6(const Config& cfg)
        : cfg_(cfg), mem_(cfg.dmemBytes, cfg.imemBytes, cfg.memory), mau_(mem_, cfg.dmemBytes),
          ms_(mau_, cfg.dmemBytes), hz_(makeHazardConfig(cfg)) {}
    Pipeline6(const Pipeline6&) = delete;
    Pipeline6& operator=(const Pipeline6&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("Pipeline6::loadProgram: program does not fit");
        mem_.loadInstructions(baseBytes, words);
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;

        const Slot oldM1 = exM1_, oldM2 = m1M2_, oldWb = m2Wb_;

        if (m2Wb_.valid) {
            if (m2Wb_.fault != Fault::None) {
                if (hz_.interrupts.canTakeException()) {
                    ++stats_.exceptionsTaken;
                    takeTrap(hz_.interrupts.enterException(static_cast<unsigned>(m2Wb_.fault), m2Wb_.pc, m2Wb_.faultValue));
                    return;
                }
                fault_ = m2Wb_.fault;
                halted_ = true;
                return;
            }
            if (m2Wb_.regWrite) {
                if (m2Wb_.destFp)                 fregs_[m2Wb_.rd] = m2Wb_.value;
                else if (m2Wb_.rd != ISA::ZERO)   regs_[m2Wb_.rd] = m2Wb_.value;
            }
            if (m2Wb_.fpFlags) hz_.interrupts.orFpFlags(m2Wb_.fpFlags);
            if (m2Wb_.aluFlags) hz_.interrupts.setStatusBits(m2Wb_.aluFlags);
            ++stats_.retired;
            if (m2Wb_.halt) { halted_ = true; return; }
            const int line = hz_.interrupts.pendingInterrupt();
            if (line >= 0) {
                ++stats_.interruptsTaken;
                stats_.lastInterruptRetired = stats_.retired;
                takeTrap(hz_.interrupts.enterInterrupt(static_cast<unsigned>(line), m2Wb_.nextPc));
                return;
            }
        }

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
                    if (s.memWrite) sd = resolve(s.w.srcB, s.w.srcBFp, Stage6::MEM2, s.pc, oldM1, oldM2, oldWb);
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
                const Word base = resolve(s.w.srcA, s.w.srcAFp, Stage6::MEM1, s.pc, oldM1, oldM2, oldWb);
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

        if (exBusy_ && killEx) exBusy_ = false;
        if (exBusy_) {
            if (--exRemaining_ == 0) { nextM1 = exResult_; exBusy_ = false; }
            else                     { exHold = true; ++stats_.execBusyCycles; }
        } else if (idEx_.valid && !killEx) {
            execute(nextM1, exHold, redirect, target, killId, killIf, oldM1, oldM2, oldWb);
        }
        if (killEx) nextM1 = Slot();

        bool idStall = false;
        if (exHold) {
            nextId = ifId_;
            nextEx = idEx_;
        } else if (killId || redirect) {
        } else if (ifId_.valid) {
            decode(nextEx, stopFetch, idStall);
        }
        if (idStall) nextId = ifId_;

        if (killId) nextEx = Slot();

        if (killIf || stopFetch) {
            fetchEnabled_ = false;
            ifWait_ = 0;
            pendingIf_ = IfId();
            if (killIf) nextId = IfId();
        } else if (redirect) {
            pc_ = target;
            fetchEnabled_ = true;
            ifWait_ = 0;
            pendingIf_ = IfId();
            nextId = IfId();
        } else if (exHold || idStall) {
        } else if (ifWait_ > 0) {
            ++stats_.ifStallCycles;
            if (--ifWait_ == 0) nextId = pendingIf_;
        } else if (fetchEnabled_) {
            uint32_t latency = 1;
            IfId fetched = fetch(latency);
            if (latency > 1) { pendingIf_ = fetched; ifWait_ = latency - 1; }
            else             { nextId = fetched; }
        }

        m2Wb_ = nextWb;
        m1M2_ = nextM2;
        exM1_ = nextM1;
        idEx_ = nextEx;
        ifId_ = nextId;
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    const ForwardingUnit6::Stats&  forwardStats()   const { return hz_.forwarding.stats(); }
    const InterlockUnit6::Stats&   interlockStats() const { return hz_.interlock.stats(); }
    const RLBranchPredictor::Stats& predictorStats() const { return hz_.predictor.stats(); }
    const InterruptHandler& specialRegs() const { return hz_.interrupts; }
    const HorizontalMicrocodeUnit6& microcode() const { return ucode_; }
    unsigned mispredictPenalty() const { return hz_.mispredictPenalty(); }
    void raiseInterrupt(unsigned line) { hz_.interrupts.raise(line); }
    uint64_t     pc()     const { return pc_; }
    Word         reg(unsigned i) const { return regs_[i]; }
    Word         freg(unsigned i) const { return fregs_[i]; }
    std::vector<Word> dmem() const { return mem_.dataSnapshot(); }
    void pokeDmem(size_t wordIndex, Word value) {
        if (wordIndex >= cfg_.dmemBytes / 8) throw std::out_of_range("Pipeline6::pokeDmem");
        mem_.pokeData(wordIndex * 8, value);
    }
    const HierarchicalMemorySystem& memory() const { return mem_; }

private:
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
        ISA::InstrWord word = 0;
        Fault fault = Fault::None;
    };

    static HazardUnit6::Config makeHazardConfig(const Config& c) {
        HazardUnit6::Config h;
        h.predictor = c.predictor;
        return h;
    }

    void takeTrap(uint64_t handler) {
        ifWait_ = 0;
        pendingIf_ = IfId();
        m2Started_ = false;
        m2Wait_ = 0;
        exBusy_ = false;
        m2Wb_ = Slot();
        m1M2_ = Slot();
        exM1_ = Slot();
        idEx_ = Slot();
        ifId_ = IfId();
        pc_ = handler;
        fetchEnabled_ = true;
    }

    static ForwardingUnit6::Producer prod(const Slot& s, bool inWb) {
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

    Word resolve(uint8_t r, bool fp, Stage6 need, uint64_t pc, const Slot& m1, const Slot& m2, const Slot& wb) {
        if (!fp && r == ISA::ZERO) return 0;
        if (!fp && r == ISA::PC)   return pc;
        ForwardingUnit6::Ahead ahead{prod(m1, false), prod(m2, false), prod(wb, true)};
        const ForwardingUnit6::Selection sel = hz_.forwarding.select(r, fp, need, ahead);
        switch (sel.source) {
            case ForwardingUnit6::Source::FromMem1: return m1.value;
            case ForwardingUnit6::Source::FromMem2: return m2.value;
            case ForwardingUnit6::Source::FromWb:   return wb.value;
            default:                                return fp ? fregs_[r] : regs_[r];
        }
    }

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

    void decode(Slot& out, bool& stopFetch, bool& stall) {
        const HorizontalMicrocodeUnit6::Result mc = ucode_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
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
            return;
        }
        if (w.halt) { out.halt = true; stopFetch = true; return; }

        Exec::RegUse u;
        u.readsA = w.readsA; u.a = w.srcA; u.aFp = w.srcAFp;
        u.readsB = w.readsB; u.b = w.srcB; u.bFp = w.srcBFp;

        InterlockUnit6::Query q;
        q.use = &u;
        q.needs = mc.needs;
        q.isSerializing = w.serializing;
        q.ex   = prod(idEx_, false);
        q.mem1 = prod(exM1_, false);
        q.mem2 = prod(m1M2_, false);
        const InterlockUnit6::Stall st = hz_.interlock.check(q);
        if (st != InterlockUnit6::Stall::None) {
            out = Slot();
            stall = true;
            return;
        }

        if (w.branch == BranchKind::RETI && !hz_.interrupts.inTrap()) {
            out.fault = Fault::IllegalInstruction;
            out.faultValue = ifId_.word;
            stopFetch = true;
            return;
        }

        out.regWrite = w.regWrite;
        out.rd       = w.dest;
        out.destFp   = w.destFp;
        out.memRead  = w.memRead;
        out.memWrite = w.memWrite;
    }

    void execute(Slot& out, bool& hold, bool& redirect, uint64_t& target, bool& killId, bool& killIf,
                 const Slot& m1, const Slot& m2, const Slot& wb) {
        out = idEx_;
        const ControlWord& w = out.w;
        if (out.fault != Fault::None || out.halt) return;

        const bool memOp = w.memRead || w.memWrite;
        const Word a = (w.readsA && !memOp) ? resolve(w.srcA, w.srcAFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
        const Word b = (w.readsB && !memOp) ? resolve(w.srcB, w.srcBFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
        const Word imm = static_cast<Word>(static_cast<int64_t>(w.immediate));
        uint32_t latency = 1;

        if (w.fpOp) {
            const unsigned rm = hz_.interrupts.roundingMode();
            if (w.fpRounds && !FPU::validRounding(rm)) {
                out.regWrite = false;
                out.fault = Fault::IllegalInstruction;
                out.faultValue = out.word;
                killId = killIf = true;
                return;
            }
            uint8_t fl = 0;
            out.value = FPU::execute(w.fpKind, a, b, rm, fl);
            out.fpFlags = fl;
            latency = FPU::latency(w.fpKind, cfg_.fp);
            ++stats_.fpOps;
        } else if (w.alu != AluOp::NONE) {
            Exec::alu(w.opcode, a, w.immB ? imm : b, out.value, out.aluFlags);
            if (w.longOp) latency = cfg_.longOpCycles;
        } else if (memOp) {
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
        if (latency > 1) {
            exBusy_ = true;
            exRemaining_ = latency - 1;
            exResult_ = out;
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

    Config cfg_;
    mutable HierarchicalMemorySystem mem_;
    MemoryAccessUnit            mau_;
    MemoryStages6               ms_;
    HazardUnit6                 hz_;
    HorizontalMicrocodeUnit6    ucode_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;

    IfId ifId_;
    Slot idEx_, exM1_, m1M2_, m2Wb_;
    bool     exBusy_ = false;
    uint32_t exRemaining_ = 0;
    Slot     exResult_;
    bool     m2Started_ = false;
    uint32_t m2Wait_ = 0, ifWait_ = 0;
    IfId     pendingIf_;
    Stats stats_;
};
