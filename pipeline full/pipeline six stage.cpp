#pragma once

// Six-stage pipeline:  IF | ID | EX | MEM1 | MEM2 | WB
//   ID    decode + hazard check (load-use / serialize interlock)
//   EX    operand forwarding, ALU/FPU/branch execute
//   MEM1  address generation + alignment/range check
//   MEM2  data memory access (a cache miss freezes the pipeline)
//   WB    register write / commit
// Forwarding comes from MEM1, MEM2 and WB; a LOAD result is ready at the end of MEM2, so a
// consumer waits up to 2 bubbles. A mispredicted branch is resolved in EX (2 bubbles).

#include "pipeline_core.cpp"
#include "forwarding_unit6.cpp"
#include "interlock_unit6.cpp"
#include "control_unit6.cpp"
#include "memory_access6.cpp"

class SixStagePipeline : public PipelineCore {
public:
    SixStagePipeline() : SixStagePipeline(Config()) {}
    explicit SixStagePipeline(const Config& cfg) : PipelineCore(cfg), ms_(mau_, cfg.dmemBytes) {}

    PipelineDepth depth() const override { return PipelineDepth::Six; }
    unsigned mispredictPenalty() const override { return ControlUnit6::redirectPenalty(Stage6::EX); }

    ForwardStats forwardStats() const override {
        ForwardStats r;
        const ForwardingUnit6::Stats& s = fwd_.stats();
        r.fromMem1 = s.fromMem1; r.fromMem2 = s.fromMem2; r.fromWb = s.fromWb;
        r.lateRegFile = s.lateRegFile; r.blocked = s.blocked;
        return r;
    }
    InterlockStats interlockStats() const override {
        InterlockStats r;
        const InterlockUnit6::Stats& s = ilk_.stats();
        r.loadUse = s.loadUse; r.serialize = s.serialize;
        for (int i = 0; i < 3; ++i) r.loadUseByBubbles[i] = s.loadUseByBubbles[i];
        return r;
    }

private:
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
        const ForwardingUnit6::Selection sel = fwd_.select(r, fp, need, ahead);
        switch (sel.source) {
            case ForwardingUnit6::Source::FromMem1: return m1.value;
            case ForwardingUnit6::Source::FromMem2: return m2.value;
            case ForwardingUnit6::Source::FromWb:   return wb.value;
            default:                                return fp ? fregs_[r] : regs_[r];
        }
    }

    void clearBackEnd() override {
        m2Started_ = false;
        m2Wait_ = 0;
        busy_ = Busy();
        m2Wb_ = m1M2_ = exM1_ = idEx_ = Slot();
    }

    void advance() override {
        const Slot oldM1 = exM1_, oldM2 = m1M2_, oldWb = m2Wb_;

        if (commit(m2Wb_)) return;

        bool killM1 = false, killEx = false, killId = false, killIf = false;

        Slot nextWb, nextM2, nextM1, nextEx;
        IfId nextId;
        bool frozen = false;

        // ---- MEM2 ----
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

        // ---- MEM1 ----
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

        // ---- EX ----
        bool redirect = false, stopFetch = false;
        uint64_t target = 0;

        if (busy_.active && killEx) busy_.active = false;
        if (busy_.active) {
            if (--busy_.remaining == 0) { nextM1 = busy_.result; busy_.active = false; }
            else                        { exHold = true; ++stats_.execBusyCycles; }
        } else if (idEx_.valid && !killEx) {
            execute(nextM1, exHold, redirect, target, killId, killIf, oldM1, oldM2, oldWb);
        }
        if (killEx) nextM1 = Slot();

        // ---- ID ----
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

        // ---- IF ----
        advanceFetch(nextId, killIf, stopFetch, redirect, target, exHold || idStall, true);

        m2Wb_ = nextWb;
        m1M2_ = nextM2;
        exM1_ = nextM1;
        idEx_ = nextEx;
        ifId_ = nextId;
    }

    void decode(Slot& out, bool& stopFetch, bool& stall) {
        const HorizontalMicrocodeUnit6::Result mc = ucode_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
        if (decodeFront(out, mc, stopFetch)) return;
        const ControlWord& w = mc.word;

        const Exec::RegUse u = regUse(w);

        InterlockUnit6::Query q;
        q.use = &u;
        q.needs = mc.needs;
        q.isSerializing = w.serializing;
        q.ex   = prod(idEx_, false);
        q.mem1 = prod(exM1_, false);
        q.mem2 = prod(m1M2_, false);
        if (ilk_.check(q) != InterlockUnit6::Stall::None) {
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

    void execute(Slot& out, bool& hold, bool& redirect, uint64_t& target, bool& killId, bool& killIf,
                 const Slot& m1, const Slot& m2, const Slot& wb) {
        out = idEx_;
        const ControlWord& w = out.w;
        if (out.fault != Fault::None || out.halt) return;

        const bool memOp = w.memRead || w.memWrite;
        const Word a = (w.readsA && !memOp) ? resolve(w.srcA, w.srcAFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
        const Word b = (w.readsB && !memOp) ? resolve(w.srcB, w.srcBFp, Stage6::EX, out.pc, m1, m2, wb) : 0;
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

    MemoryStages6   ms_;
    ForwardingUnit6 fwd_;
    InterlockUnit6  ilk_;
    Busy     busy_;
    Slot     idEx_, exM1_, m1M2_, m2Wb_;
    bool     m2Started_ = false;
    uint32_t m2Wait_ = 0;
};
