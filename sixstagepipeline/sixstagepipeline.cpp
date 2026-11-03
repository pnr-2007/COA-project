#pragma once

#include "exec.cpp"
#include "matrix_unit.cpp"
#include "exception_handler.cpp"
#include "floating_point_unit.cpp"
#include "forwarding_unit.cpp"
#include "interlock_unit.cpp"
#include "branch_predictor.cpp"
#include "microprogrammed_control_unit.cpp"
#include <stdexcept>
#include <vector>

class SixStagePipeline {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    struct Config {
        size_t   imemBytes    = 1 << 18;
        size_t   dmemBytes    = 1 << 19;
        uint32_t longOpCycles = 64;
        uint32_t memLatency   = 1;
        unsigned microWidth   = 1;
        FPU::Latency            fpLat;
        MatrixUnit::Config      matrix;
        RLBranchPredictor::Config predictor;
    };

    struct Stats {
        uint64_t cycles            = 0;
        uint64_t retired           = 0;
        uint64_t branches          = 0;
        uint64_t takenBranches     = 0;
        uint64_t mispredicts       = 0;
        uint64_t idRedirects       = 0;
        uint64_t execBusyCycles    = 0;
        uint64_t memBusyCycles     = 0;
        uint64_t microStallCycles  = 0;
        uint64_t loadUseStalls     = 0;
        uint64_t matrixStallCycles = 0;
        uint64_t serializeStalls   = 0;
        uint64_t fwdFromMem1       = 0;
        uint64_t fwdFromMem2       = 0;
        uint64_t fwdFromWb         = 0;
        uint64_t matrixOps         = 0;
        uint64_t fpOps             = 0;
        uint64_t exceptionsTaken   = 0;
        uint64_t interruptsTaken   = 0;
        uint64_t drainCycles       = 0;
        double ipc() const { return cycles ? double(retired) / double(cycles) : 0.0; }
    };

    SixStagePipeline() : SixStagePipeline(Config()) {}
    explicit SixStagePipeline(const Config& cfg)
        : cfg_(cfg), imem_(cfg.imemBytes / 4, 0xFFFFFFFFu), dmem_(cfg.dmemBytes / 8, 0),
          mu_(dmem_, cfg.matrix), predictor_(cfg.predictor) {}
    SixStagePipeline(const SixStagePipeline&) = delete;
    SixStagePipeline& operator=(const SixStagePipeline&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("SixStagePipeline::loadProgram: program does not fit");
        for (size_t i = 0; i < words.size(); ++i) imem_[(baseBytes >> 2) + i] = words[i];
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;

        if (draining_) {
            ++stats_.drainCycles;
            stepMatrix();
            if (!mu_.busy()) halted_ = true;
            return;
        }
        stepMatrix();

        int stall = -1, flush = -1;
        bool redirect = false;
        uint64_t redirectPc = 0;
        Slot  nMem2Wb, nMem1Mem2, nExMem1;
        IdEx  nIdEx;
        IfId  nIfId;

        if (mem2Wb_.valid) {
            const Slot& w = mem2Wb_;
            if (w.fault != Fault::None) {
                if (trap_.canTakeException()) { takeException(w); return; }
                fault_ = w.fault;
                beginDrain();
                return;
            }
            if (w.regWrite) {
                if (w.destFp)                 fregs_[w.rd] = w.value;
                else if (w.rd != ISA::ZERO)   regs_[w.rd]  = w.value;
            }
            trap_.orFpFlags(w.fpFlags);
            ++stats_.retired;
            if (w.halt) { beginDrain(); return; }
            for (const Scheduled& e : schedule_)
                if (e.atRetired == stats_.retired) trap_.raise(e.line);
            const int line = trap_.pending();
            if (line >= 0) { takeInterrupt(static_cast<unsigned>(line), w.nextPc); return; }
        }

        if (mem1Mem2_.valid) {
            Slot s = mem1Mem2_;
            bool busy = false;
            if (s.fault == Fault::None && (s.memRead || s.memWrite) && cfg_.memLatency > 1) {
                if (!s.started) { mem1Mem2_.started = true; m2Remaining_ = cfg_.memLatency - 1; }
                if (m2Remaining_ > 0) { --m2Remaining_; busy = true; }
            }
            if (busy) {
                stall = 4;
                nMem1Mem2 = mem1Mem2_;
                ++stats_.memBusyCycles;
            } else {
                if (s.fault == Fault::None) {
                    if (s.memRead) {
                        s.value = dmem_[s.value >> 3];
                    } else if (s.memWrite) {
                        dmem_[s.value >> 3] = s.storeData;
                    } else if (s.matrixOp) {
                        const MatrixUnit::StartResult r = mu_.start(s.matFields, s.matPtrD, s.matPtrA, s.matPtrB);
                        if (r == MatrixUnit::StartResult::Started) {
                            ++stats_.matrixOps;
                        } else if (r == MatrixUnit::StartResult::Busy) {
                            throw std::logic_error("SixStagePipeline: matrix unit started while busy");
                        } else {
                            s.fault = (r == MatrixUnit::StartResult::MemoryFault) ? Fault::MemoryFault
                                                                                   : Fault::IllegalInstruction;
                            s.badValue = s.matPtrD;
                            s.regWrite = false;
                            s.matrixOp = false;
                            flush = 3;
                            fetchEnabled_ = false;
                        }
                    }
                }
                nMem2Wb = s;
            }
        }

        if (flush >= 3) {
        } else if (stall >= 3) {
            if (exMem1_.valid) nExMem1 = exMem1_;
        } else if (exMem1_.valid) {
            Slot s = exMem1_;
            if (s.fault == Fault::None && (s.memRead || s.memWrite)) {
                const Word addr = s.value;
                if ((addr & 7) != 0 || addr >= cfg_.dmemBytes) {
                    s.fault = Fault::MemoryFault;
                    s.badValue = addr;
                    s.regWrite = false;
                    s.memRead = s.memWrite = false;
                    flush = 2;
                    fetchEnabled_ = false;
                }
            }
            nMem1Mem2 = s;
        }

        if (flush >= 2) {
            exRemaining_ = 0;
        } else if (stall >= 2) {
            if (idEx_.valid) nIdEx = idEx_;
        } else if (idEx_.valid) {
            IdEx e = idEx_;
            if (!e.started) {
                computeExecute(e, flush, redirect, redirectPc);
                e.started = true;
            }
            if (exRemaining_ > 0 && e.res.fault == Fault::None) {
                --exRemaining_;
                stall = 2;
                nIdEx = e;
                ++stats_.execBusyCycles;
            } else {
                nExMem1 = e.res;
            }
        }

        if (flush >= 1) {
            ucLoaded_ = false;
            ucRemaining_ = 0;
        } else if (stall >= 1) {
            if (ifId_.valid) nIfId = ifId_;
        } else if (ifId_.valid) {
            decodeStage(nIdEx, nIfId, stall, flush, redirect, redirectPc);
        }

        if (flush < 0 && stall < 0 && fetchEnabled_) nIfId = fetch();

        if (redirect) pc_ = redirectPc;
        mem2Wb_  = nMem2Wb;
        mem1Mem2_ = nMem1Mem2;
        exMem1_  = nExMem1;
        idEx_    = nIdEx;
        ifId_    = nIfId;
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    const MatrixUnit::Stats&                 matrixStats()    const { return mu_.stats(); }
    const MicroprogrammedControlUnit::Stats& microStats()     const { return mcu_.stats(); }
    const RLBranchPredictor::Stats&          predictorStats() const { return predictor_.stats(); }
    const ExceptionHandler&                  specialRegs()    const { return trap_; }
    uint64_t     pc()     const { return pc_; }
    Word         reg(unsigned i)  const { return regs_[i]; }
    Word         freg(unsigned i) const { return fregs_[i]; }
    const std::vector<Word>& dmem() const { return dmem_; }
    void pokeDmem(size_t wordIndex, Word value) { dmem_.at(wordIndex) = value; }
    void raiseInterrupt(unsigned line) { trap_.raise(line); }
    void scheduleInterrupt(uint64_t atRetired, unsigned line) { schedule_.push_back({atRetired, line}); }
    unsigned verifyMicrocode() { return mcu_.selfCheck(); }

private:
    class TrapUnit : public ExceptionHandler {
    public:
        static constexpr Word     CAUSE_BASE  = 16;
        static constexpr unsigned VECTOR_BASE = 4;

        void raise(unsigned line) {
            if (line < INTERRUPT_LINES) sr_[IPEND] |= Word(1) << line;
        }
        int pending() const {
            if (!(sr_[STATUS] & STATUS_IE) || inTrap() || !handlerInstalled()) return -1;
            const Word p = sr_[IPEND] & sr_[IMASK];
            for (unsigned i = 0; i < INTERRUPT_LINES; ++i)
                if (p & (Word(1) << i)) return static_cast<int>(i);
            return -1;
        }
        Word enterInterrupt(unsigned line, Word nextPc) {
            sr_[IPEND] &= ~(Word(1) << line);
            sr_[EPC]    = nextPc;
            sr_[CAUSE]  = CAUSE_BASE + line;
            beginTrap();
            return sr_[TVEC] + VECTOR_STRIDE * (VECTOR_BASE + line);
        }
    };

    struct IfId {
        bool valid = false, fetchFault = false;
        uint64_t pc = 0;
        ISA::InstrWord word = 0;
    };

    struct Slot {
        bool valid = false, regWrite = false, destFp = false;
        bool memRead = false, memWrite = false, halt = false, matrixOp = false, started = false;
        uint64_t pc = 0, nextPc = 0;
        uint8_t  rd = 0, fpFlags = 0;
        Word     value = 0, storeData = 0, badValue = 0;
        ISA::InstructionFields matFields;
        Word     matPtrD = 0, matPtrA = 0, matPtrB = 0;
        Fault    fault = Fault::None;
    };

    struct IdEx {
        bool valid = false, started = false;
        ControlWord cw;
        uint64_t pc = 0;
        ISA::InstrWord word = 0;
        Word a = 0, b = 0, c = 0;
        uint64_t predNext = 0;
        RLBranchPredictor::Prediction pred;
        Slot res;
    };

    struct Scheduled { uint64_t atRetired; unsigned line; };

    void stepMatrix() {
        if (mu_.step() && mu_.lastOverflow()) trap_.setStatusBits(ExceptionHandler::STATUS_OVERFLOW);
    }

    void beginDrain() {
        draining_ = true;
        if (!mu_.busy()) halted_ = true;
    }

    void flushAll() {
        mem2Wb_ = Slot(); mem1Mem2_ = Slot(); exMem1_ = Slot(); idEx_ = IdEx(); ifId_ = IfId();
        exRemaining_ = 0; m2Remaining_ = 0;
        ucLoaded_ = false; ucRemaining_ = 0;
        fetchEnabled_ = true;
    }

    void takeException(const Slot& s) {
        const unsigned code = static_cast<unsigned>(s.fault);
        const Word pcOfFault = s.pc, bad = s.badValue;
        flushAll();
        pc_ = trap_.enterException(code, pcOfFault, bad);
        ++stats_.exceptionsTaken;
    }

    void takeInterrupt(unsigned line, Word nextPc) {
        flushAll();
        pc_ = trap_.enterInterrupt(line, nextPc);
        ++stats_.interruptsTaken;
    }

    IfId fetch() {
        IfId r;
        r.valid = true;
        r.pc    = pc_;
        if ((pc_ & 3) != 0 || pc_ >= cfg_.imemBytes) {
            r.fetchFault  = true;
            fetchEnabled_ = false;
        } else {
            r.word = imem_[pc_ >> 2];
            pc_ += 4;
        }
        return r;
    }

    static ForwardingUnit::Producer producerOf(const Slot& s) {
        ForwardingUnit::Producer p;
        p.valid = s.valid;
        p.regWrite = s.regWrite && s.fault == Fault::None;
        p.memRead = s.memRead;
        p.rd = s.rd;
        p.value = s.value;
        p.fp = s.destFp;
        return p;
    }

    static ForwardingUnit::Producer producerOf(const IdEx& e) {
        ForwardingUnit::Producer p;
        p.valid = e.valid;
        p.regWrite = e.cw.regWrite && e.cw.fault == Fault::None;
        p.memRead = e.cw.memRead;
        p.rd = e.cw.dest;
        p.fp = e.cw.destFp;
        return p;
    }

    static Exec::RegUse useOf(const ControlWord& cw) {
        Exec::RegUse u;
        u.readsA = cw.readsA; u.a = cw.srcA; u.aFp = cw.srcAFp;
        u.readsB = cw.readsB; u.b = cw.srcB; u.bFp = cw.srcBFp;
        u.readsC = cw.readsC; u.c = cw.srcC;
        u.writes = cw.regWrite; u.dest = cw.dest; u.destFp = cw.destFp;
        return u;
    }

    bool matrixPending() const {
        auto pend = [](const Slot& s) { return s.valid && s.matrixOp && s.fault == Fault::None; };
        return mu_.busy() || (idEx_.valid && idEx_.cw.matrixOp && idEx_.cw.fault == Fault::None) ||
               pend(exMem1_) || pend(mem1Mem2_);
    }

    Word bypass(uint8_t r, bool fp, Word fromRegFile) {
        if (!fp && (r == ISA::ZERO || r == ISA::PC)) return fromRegFile;
        const ForwardingUnit::Producer p3 = producerOf(exMem1_);
        if (ForwardingUnit::matches(p3, r, fp)) { ++stats_.fwdFromMem1; return p3.value; }
        const ForwardingUnit::Producer p4 = producerOf(mem1Mem2_);
        if (ForwardingUnit::matches(p4, r, fp)) { ++stats_.fwdFromMem2; return p4.value; }
        const ForwardingUnit::Producer p5 = producerOf(mem2Wb_);
        if (ForwardingUnit::matches(p5, r, fp)) { ++stats_.fwdFromWb; return p5.value; }
        return fromRegFile;
    }

    Word readReg(uint8_t r, bool fp, uint64_t pc) const {
        if (fp)             return fregs_[r];
        if (r == ISA::ZERO) return 0;
        if (r == ISA::PC)   return pc;
        return regs_[r];
    }

    void decodeStage(IdEx& nIdEx, IfId& nIfId, int& stall, int& flush, bool& redirect, uint64_t& redirectPc) {
        if (!ucLoaded_) {
            const MicroprogrammedControlUnit::Result r = mcu_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
            const unsigned w = cfg_.microWidth ? cfg_.microWidth : 1;
            ucWord_      = r.word;
            ucRemaining_ = (r.microCycles + w - 1) / w;
            ucLoaded_    = true;
        }
        if (ucRemaining_ > 1) {
            --ucRemaining_;
            stall = 1;
            nIfId = ifId_;
            ++stats_.microStallCycles;
            return;
        }

        const ControlWord& cw = ucWord_;
        if (cw.fault != Fault::None || cw.halt) {
            nIdEx.valid = true;
            nIdEx.cw    = cw;
            nIdEx.pc    = ifId_.pc;
            nIdEx.word  = ifId_.word;
            flush = 0;
            fetchEnabled_ = false;
            ucLoaded_ = false;
            return;
        }

        const Exec::RegUse use = useOf(cw);
        InterlockUnit::Query q;
        q.use            = &use;
        q.isMatrixOrWait = cw.matrixOp || cw.matWait;
        q.matrixBusy     = matrixPending();
        q.exMa           = producerOf(idEx_);
        InterlockUnit::Stall st = interlock_.check(q);
        if (st == InterlockUnit::Stall::None) {
            if (cw.serializing && (idEx_.valid || exMem1_.valid || mem1Mem2_.valid)) {
                st = InterlockUnit::Stall::Serialize;
            } else {
                InterlockUnit::Query q2;
                q2.use   = &use;
                q2.exMa  = producerOf(exMem1_);
                st = interlock_.check(q2);
            }
        }
        if (st != InterlockUnit::Stall::None) {
            if (st == InterlockUnit::Stall::MatrixBusy)     ++stats_.matrixStallCycles;
            else if (st == InterlockUnit::Stall::Serialize) ++stats_.serializeStalls;
            else                                            ++stats_.loadUseStalls;
            stall = 1;
            nIfId = ifId_;
            return;
        }

        nIdEx.valid = true;
        nIdEx.cw    = cw;
        nIdEx.pc    = ifId_.pc;
        nIdEx.word  = ifId_.word;
        nIdEx.a = cw.readsA ? readReg(cw.srcA, cw.srcAFp, ifId_.pc) : 0;
        nIdEx.b = cw.readsB ? readReg(cw.srcB, cw.srcBFp, ifId_.pc) : 0;
        nIdEx.c = cw.readsC ? readReg(cw.srcC, false, ifId_.pc) : 0;

        const uint64_t pc = ifId_.pc;
        const uint64_t target = pc + static_cast<Word>(static_cast<int64_t>(cw.immediate) * 4);
        nIdEx.predNext = pc + 4;
        switch (cw.branch) {
            case BranchKind::JEQ: case BranchKind::JGT: case BranchKind::JZ: case BranchKind::JNZ:
                nIdEx.pred = predictor_.predict(pc, cw.immediate < 0);
                if (nIdEx.pred.taken) nIdEx.predNext = target;
                break;
            case BranchKind::JMP:
                nIdEx.predNext = target;
                break;
            case BranchKind::CALL:
                nIdEx.predNext = target;
                rstack_.push(pc + 4);
                break;
            case BranchKind::RET: {
                uint64_t t;
                if (rstack_.top(t)) { nIdEx.predNext = t; rstack_.pop(); }
                break;
            }
            default: break;
        }
        ucLoaded_ = false;
        if (nIdEx.predNext != pc + 4) {
            redirect = true;
            redirectPc = nIdEx.predNext;
            flush = 0;
            ++stats_.idRedirects;
        }
    }

    void computeExecute(IdEx& e, int& flush, bool& redirect, uint64_t& redirectPc) {
        const ControlWord& cw = e.cw;
        Slot r;
        r.valid  = true;
        r.pc     = e.pc;
        r.nextPc = e.pc + 4;
        exRemaining_ = 0;

        if (cw.fault != Fault::None) { r.fault = cw.fault; r.badValue = cw.faultValue; e.res = r; return; }
        if (cw.halt)                 { r.halt = true; e.res = r; return; }

        auto failWith = [&](Fault f, Word bad) {
            r.fault = f; r.badValue = bad;
            flush = 1;
            fetchEnabled_ = false;
            e.res = r;
        };

        if (cw.branch == BranchKind::RETI && !trap_.inTrap()) { failWith(Fault::IllegalInstruction, e.word); return; }
        const unsigned rm = trap_.roundingMode();
        if (cw.fpOp && Exec::fpRoundingIllegal(cw.opcode, rm)) { failWith(Fault::IllegalInstruction, e.word); return; }

        const Word a = cw.readsA ? bypass(cw.srcA, cw.srcAFp, e.a) : 0;
        const Word b = cw.readsB ? bypass(cw.srcB, cw.srcBFp, e.b) : 0;
        const Word c = cw.readsC ? bypass(cw.srcC, false, e.c) : 0;
        const Word imm = static_cast<Word>(static_cast<int64_t>(cw.immediate));

        r.regWrite = cw.regWrite;
        r.destFp   = cw.destFp;
        r.rd       = cw.dest;

        if (cw.alu != AluOp::NONE) {
            Exec::alu(cw.opcode, a, cw.immB ? imm : b, r.value);
            if (cw.longOp && cfg_.longOpCycles > 1) exRemaining_ = cfg_.longOpCycles - 1;
        } else if (cw.fpOp) {
            uint8_t fl = 0;
            r.value   = FPU::execute(cw.fpKind, a, b, rm, fl);
            r.fpFlags = fl;
            ++stats_.fpOps;
            const unsigned lat = FPU::latency(cw.fpKind, cfg_.fpLat);
            if (lat > 1) exRemaining_ = lat - 1;
        } else if (cw.memRead || cw.memWrite) {
            r.value    = a + imm;
            r.memRead  = cw.memRead;
            r.memWrite = cw.memWrite;
            r.storeData = b;
        } else if (cw.branch != BranchKind::NONE) {
            bool taken;
            uint64_t actual;
            if (cw.branch == BranchKind::RETI) {
                taken  = true;
                actual = trap_.returnFromTrap();
            } else {
                taken  = Exec::branchTaken(cw.opcode, a, b);
                actual = taken ? ((cw.branch == BranchKind::RET) ? a : e.pc + imm * 4) : e.pc + 4;
            }
            if (cw.branch == BranchKind::CALL) r.value = e.pc + 4;
            r.nextPc = actual;
            ++stats_.branches;
            if (taken) ++stats_.takenBranches;
            if (cw.conditionalBranch) predictor_.update(e.pred, taken);
            if (actual != e.predNext) {
                ++stats_.mispredicts;
                flush = 1;
                redirect = true;
                redirectPc = actual;
                fetchEnabled_ = true;
            }
        } else if (cw.matrixOp) {
            r.matrixOp = true;
            r.matFields.opcode      = cw.opcode;
            r.matFields.matrixRows  = cw.matRows;
            r.matFields.matrixInner = cw.matInner;
            r.matFields.matrixCols  = cw.matCols;
            r.matPtrD = c;
            r.matPtrA = a;
            r.matPtrB = b;
        } else if (cw.readsSR) {
            r.value = trap_.read(static_cast<unsigned>(cw.immediate));
        } else if (cw.writesSR) {
            trap_.write(static_cast<unsigned>(cw.immediate), a);
        }
        e.res = r;
    }

    Config cfg_;
    std::vector<ISA::InstrWord> imem_;
    std::vector<Word>           dmem_;
    MatrixUnit                  mu_;
    TrapUnit                    trap_;
    MicroprogrammedControlUnit  mcu_;
    InterlockUnit               interlock_;
    RLBranchPredictor           predictor_;
    ReturnStack                 rstack_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    std::vector<Scheduled>      schedule_;

    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    bool     draining_ = false;
    Fault    fault_ = Fault::None;

    ControlWord ucWord_;
    unsigned    ucRemaining_ = 0;
    bool        ucLoaded_ = false;
    unsigned    exRemaining_ = 0;
    uint32_t    m2Remaining_ = 0;

    IfId ifId_;
    IdEx idEx_;
    Slot exMem1_, mem1Mem2_, mem2Wb_;
    Stats stats_;
};
