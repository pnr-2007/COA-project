#pragma once

#include "exec.cpp"
#include "forwarding_unit.cpp"

class InterlockUnit {
public:
    enum class Stall : uint8_t { None, LoadUse, MatrixBusy, Serialize };

    struct Stats {
        uint64_t loadUse = 0;
        uint64_t matrixBusy = 0;
        uint64_t serialize = 0;
    };

    struct Query {
        const Exec::RegUse* use = nullptr;
        bool isMatrixOrWait = false;
        bool isSerializing = false;
        bool matrixBusy = false;
        ForwardingUnit::Producer exMa;
    };

    Stall check(const Query& q) {
        if (q.isMatrixOrWait && q.matrixBusy) { ++stats_.matrixBusy; return Stall::MatrixBusy; }
        if (q.isSerializing && q.exMa.valid) { ++stats_.serialize; return Stall::Serialize; }
        const Exec::RegUse& u = *q.use;
        if (q.exMa.memRead && ForwardingUnit::matches(q.exMa, q.exMa.rd) &&
            ((u.readsA && u.a == q.exMa.rd) || (u.readsB && u.b == q.exMa.rd) || (u.readsC && u.c == q.exMa.rd))) {
            ++stats_.loadUse;
            return Stall::LoadUse;
        }
        return Stall::None;
    }

    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
};
