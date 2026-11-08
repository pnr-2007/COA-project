#pragma once

// Four-stage pipeline:  IF | DE+EX | MA | RW
//   DE+EX  decode, hazard check, operand forwarding and execute in one stage
//   MA     data memory access (a cache miss freezes the front end)
//   RW     register write / commit
// Forwarding comes from EX/MA (distance 1) and MA/RW (distance 2); a LOAD followed by a
// consumer costs one bubble. A mispredicted branch is resolved in DE+EX (1 bubble).

#include "pipeline_core.cpp"
#include "forwarding_unit.cpp"
#include "interlock_unit.cpp"

class FourStagePipeline : public PipelineCore {
public:
    FourStagePipeline() : PipelineCore(Config()) {}
    explicit FourStagePipeline(const Config& cfg) : PipelineCore(cfg) {}

    PipelineDepth depth() const override { return PipelineDepth::Four; }
    unsigned mispredictPenalty() const override { return 1; }

    ForwardStats forwardStats() const override {
        ForwardStats r;
        r.fromExMa = fwd_.stats().fromExMa;
        r.fromMaRw = fwd_.stats().fromMaRw;
        return r;
    }
    InterlockStats interlockStats() const override {
        InterlockStats r;
        r.loadUse   = ilk_.stats().loadUse;
        r.serialize = ilk_.stats().serialize;
        return r;
    }

private:
    static ForwardingUnit::Producer producer(const Slot& e, bool inRw) {
        return {e.valid, e.regWrite, inRw ? false : e.memRead, e.rd, e.value, e.destFp};
    }

    std::vector<StageView> stageView() const override {
        auto mk = [](const char* n, const Slot& s, std::string note = "") {
            StageView v; v.name = n; v.valid = s.valid; v.pc = s.pc; v.word = s.word;
            v.fault = s.fault; v.note = std::move(note); return v;
        };
        std::vector<StageView> r;
        StageView f; f.name = "IF"; f.hasWord = false; f.pc = pc_;
        f.valid = fetchEnabled_ && !halted_;
        f.note = !fetchEnabled_ ? "stopped" : (ifWait_ ? "fetch wait " + std::to_string(ifWait_) : "");
        r.push_back(f);
        StageView d;
        if (busy_.active) d = mk("DE+EX", busy_.result, "multi-cycle, " + std::to_string(busy_.remaining) + " left");
        else {
            d.name = "DE+EX"; d.valid = ifId_.valid; d.pc = ifId_.pc; d.word = ifId_.word;
            d.fault = ifId_.fetchFault ? Fault::FetchFault : Fault::None;
        }
        r.push_back(d);
        r.push_back(maHoldValid_ ? mk("MA", maHold_, "mem wait " + std::to_string(maWait_)) : mk("MA", exMa_));
        r.push_back(mk("RW", maRw_));
        return r;
    }

    void clearBackEnd() override {
        maHoldValid_ = false;
        maWait_ = 0;
        busy_ = Busy();
        exMa_ = maRw_ = Slot();
    }

    void advance() override {
        if (commit(maRw_)) return;

        // ---- MA ----
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

        // ---- DE+EX ----
        Slot nextExMa;
        bool stallFetch = false, stopFetch = false, redirect = false;
        uint64_t target = 0;

        if (busy_.active) {
            if (--busy_.remaining == 0) { nextExMa = busy_.result; busy_.active = false; }
            else                        { stallFetch = true; ++stats_.execBusyCycles; }
        } else if (ifId_.valid) {
            decodeExecute(nextExMa, stopFetch, redirect, target, stallFetch);
        }

        // ---- IF ----
        IfId nextIf;
        if (squash) { nextExMa = Slot(); busy_.active = false; }
        if (!(squash || stopFetch) && !redirect && stallFetch) nextIf = ifId_;
        advanceFetch(nextIf, false, squash || stopFetch, redirect, target, stallFetch, false);

        maRw_ = nextMaRw;
        exMa_ = nextExMa;
        ifId_ = nextIf;
    }

    void decodeExecute(Slot& out, bool& stopFetch, bool& redirect, uint64_t& target, bool& stallFetch) {
        const HorizontalMicrocodeUnit6::Result mc = ucode_.translateWord(ifId_.word, ifId_.pc, ifId_.fetchFault);
        if (decodeFront(out, mc, stopFetch)) return;
        if (checkStrayReti(out, stopFetch)) return;
        const ControlWord& w = mc.word;

        const Exec::RegUse u = regUse(w);

        InterlockUnit::Query q;
        q.use = &u;
        q.isSerializing = w.serializing;
        q.exMa = producer(exMa_, false);
        if (ilk_.check(q) != InterlockUnit::Stall::None) {
            out = Slot();
            stallFetch = true;
            return;
        }

        if (ForwardingUnit::matches(producer(exMa_, false), exMa_.rd, exMa_.destFp) &&
            ((u.readsA && u.a == exMa_.rd && u.aFp == exMa_.destFp) ||
             (u.readsB && u.b == exMa_.rd && u.bFp == exMa_.destFp)))
            ++stats_.rawHazards;

        auto readReg = [&](uint8_t r, bool fp) -> Word {
            if (!fp && r == ISA::ZERO) return 0;
            if (!fp && r == ISA::PC)   return ifId_.pc;
            switch (fwd_.select(r, fp, producer(exMa_, false), producer(maRw_, true))) {
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

        const Core core = executeCore(out, a, b, latency, true);
        if (core == Core::StackViolation) {      // precise MEMORY_FAULT, reported at RW
            stopFetch = true;
            return;
        }
        if (core == Core::IllegalRounding) {
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

    ForwardingUnit fwd_;
    InterlockUnit  ilk_;
    Busy     busy_;
    Slot     exMa_, maRw_;
    bool     maHoldValid_ = false;
    uint32_t maWait_ = 0;
    Slot     maHold_;
};
