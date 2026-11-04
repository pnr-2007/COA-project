#pragma once

#include "control_unit.cpp"
#include "forwarding_unit6.cpp"
#include <array>

enum class MemAction6 : uint8_t { None, Load, Store };

struct StageControl6 {
    bool valid = false, regWrite = false, destFp = false, memRead = false, memWrite = false;
    bool halt = false;
    uint8_t rd = 0;
    Exec::Fault fault = Exec::Fault::None;
    Stage6 resultReady = Stage6::EX;
    MemAction6 mem = MemAction6::None;
};

struct HazardInputs6 {
    bool exception = false, interrupt = false, memFreeze = false, redirect = false;
    bool loadUse = false, serialize = false, longOpBusy = false;
    Stage6 squashAt = Stage6::None;
};

struct PipelineControl6 {
    enum class Reason : uint8_t { None, Exception, Interrupt, MemFreeze, Squash, Redirect, LoadUse, Serialize, LongOpBusy };

    Reason reason = Reason::None;
    bool takeTrap = false;
    bool redirect = false;
    bool stopFetch = false;
    std::array<bool, 7> hold{};
    std::array<bool, 7> flush{};
    std::array<bool, 7> bubble{};
};

class ControlUnit6 {
public:
    static ControlWord decode(const ISA::InstructionFields& f) { return ControlUnit::decode(f); }

    static OperandNeeds6 needs(const ControlWord& w) {
        OperandNeeds6 n;
        if (w.memRead || w.memWrite) {
            n.a = Stage6::MEM1;
            n.b = Stage6::MEM2;
        }
        return n;
    }

    static Stage6 resultReady(const ControlWord& w) { return w.memRead ? Stage6::MEM2 : Stage6::EX; }

    static MemAction6 memAction(const ControlWord& w) {
        if (w.memRead)  return MemAction6::Load;
        if (w.memWrite) return MemAction6::Store;
        return MemAction6::None;
    }

    static StageControl6 toStage(const ControlWord& w) {
        StageControl6 s;
        s.valid = w.valid;
        s.regWrite = w.regWrite;
        s.destFp = w.destFp;
        s.memRead = w.memRead;
        s.memWrite = w.memWrite;
        s.halt = w.halt;
        s.rd = w.dest;
        s.fault = w.fault;
        s.resultReady = resultReady(w);
        s.mem = memAction(w);
        return s;
    }

    static unsigned redirectPenalty(Stage6 resolvedAt) { return unsigned(resolvedAt) - unsigned(Stage6::IF); }

    static PipelineControl6 resolve(const HazardInputs6& in) {
        PipelineControl6 c;
        using R = PipelineControl6::Reason;
        auto idx = [](Stage6 s) { return size_t(s); };
        auto holdUpTo = [&](Stage6 last) { for (size_t s = idx(Stage6::IF); s <= idx(last); ++s) c.hold[s] = true; };
        auto flushUpTo = [&](Stage6 last) { for (size_t s = idx(Stage6::IF); s <= idx(last); ++s) c.flush[s] = true; };

        if (in.exception) {
            c.reason = R::Exception; c.takeTrap = true;
            flushUpTo(Stage6::WB);
        } else if (in.interrupt) {
            c.reason = R::Interrupt; c.takeTrap = true;
            flushUpTo(Stage6::MEM2);
        } else if (in.memFreeze) {
            c.reason = R::MemFreeze;
            holdUpTo(Stage6::MEM2);
            c.bubble[idx(Stage6::WB)] = true;
        } else if (in.squashAt != Stage6::None) {
            c.reason = R::Squash; c.stopFetch = true;
            for (size_t s = idx(Stage6::IF); s < idx(in.squashAt); ++s) c.flush[s] = true;
        } else if (in.redirect) {
            c.reason = R::Redirect; c.redirect = true;
            c.flush[idx(Stage6::IF)] = true;
            c.flush[idx(Stage6::ID)] = true;
        } else if (in.serialize || in.loadUse) {
            c.reason = in.serialize ? R::Serialize : R::LoadUse;
            holdUpTo(Stage6::ID);
            c.bubble[idx(Stage6::EX)] = true;
        } else if (in.longOpBusy) {
            c.reason = R::LongOpBusy;
            holdUpTo(Stage6::EX);
            c.bubble[idx(Stage6::MEM1)] = true;
        }
        return c;
    }
};
