#pragma once

#include "forwarding_unit6.cpp"

class InterlockUnit6 {
public:
    enum class Stall : uint8_t { None, LoadUse, Serialize };

    struct Stats {
        uint64_t loadUse = 0;
        uint64_t serialize = 0;
        uint64_t loadUseByBubbles[3] = {0, 0, 0};
    };

    struct Query {
        const Exec::RegUse* use = nullptr;
        OperandNeeds6 needs;
        bool isSerializing = false;
        ForwardingUnit6::Producer ex, mem1, mem2;
    };

    static unsigned bubblesFor(const Query& q, uint8_t src, bool fp, Stage6 need) {
        struct Ahead { const ForwardingUnit6::Producer* p; int stage; };
        const Ahead order[] = {
            {&q.ex,   int(Stage6::EX)},
            {&q.mem1, int(Stage6::MEM1)},
            {&q.mem2, int(Stage6::MEM2)},
        };
        for (const Ahead& a : order) {
            if (!ForwardingUnit6::matches(*a.p, src, fp)) continue;
            const int need2 = int(need) - int(Stage6::ID);
            const int b = (int(a.p->readyStage) + 1) - (a.stage + need2);
            return b > 0 ? unsigned(b) : 0;
        }
        return 0;
    }

    static unsigned bubbles(const Query& q) {
        const Exec::RegUse& u = *q.use;
        unsigned worst = 0;
        if (u.readsA) worst = std::max(worst, bubblesFor(q, u.a, u.aFp, q.needs.a));
        if (u.readsB) worst = std::max(worst, bubblesFor(q, u.b, u.bFp, q.needs.b));
        return worst;
    }

    Stall check(const Query& q) {
        if (q.isSerializing && (q.ex.valid || q.mem1.valid || q.mem2.valid)) { ++stats_.serialize; return Stall::Serialize; }
        const unsigned b = bubbles(q);
        if (b > 0) {
            ++stats_.loadUse;
            ++stats_.loadUseByBubbles[std::min(b, 3u) - 1];
            return Stall::LoadUse;
        }
        return Stall::None;
    }

    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
};
