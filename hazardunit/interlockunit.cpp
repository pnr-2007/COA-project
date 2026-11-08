#pragma once

#include "exec.cpp"
#include "forwarding_unit.cpp"

class InterlockUnit {
public:
    enum class Stall : uint8_t { None, LoadUse, Serialize };

    struct Stats {
        uint64_t loadUse = 0;
        uint64_t serialize = 0;
    };

    struct Query {
        const Exec::RegUse* use = nullptr;
        bool isSerializing = false;
        ForwardingUnit::Producer exMa;
    };

    Stall check(const Query& q) {
        if (q.isSerializing && q.exMa.valid) { ++stats_.serialize; return Stall::Serialize; }
        const Exec::RegUse& u = *q.use;
        const ForwardingUnit::Producer& p = q.exMa;
        if (p.memRead && ForwardingUnit::matches(p, p.rd, p.fp) &&
            ((u.readsA && u.a == p.rd && u.aFp == p.fp) || (u.readsB && u.b == p.rd && u.bFp == p.fp))) {
            ++stats_.loadUse;
            return Stall::LoadUse;
        }
        return Stall::None;
    }

    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
};
