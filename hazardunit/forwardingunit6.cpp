
#pragma once

#include "exec.cpp"

enum class Stage6 : uint8_t { None = 0, IF = 1, ID = 2, EX = 3, MEM1 = 4, MEM2 = 5, WB = 6 };

struct OperandNeeds6 {
    Stage6 a = Stage6::EX;
    Stage6 b = Stage6::EX;
};

class ForwardingUnit6 {
public:
    using Word = ISA::Word;

    enum class Source : uint8_t { RegFile, FromMem1, FromMem2, FromWb };

    struct Producer {
        bool valid = false, regWrite = false, fp = false, ready = false;
        uint8_t rd = 0;
        Word value = 0;
        Stage6 readyStage = Stage6::EX;
    };

    struct Ahead {
        Producer mem1, mem2, wb;
    };

    struct Selection {
        Source source = Source::RegFile;
        bool blocked = false;
    };

    struct Stats {
        uint64_t fromMem1 = 0;
        uint64_t fromMem2 = 0;
        uint64_t fromWb = 0;
        uint64_t lateRegFile = 0;
        uint64_t blocked = 0;
    };

    static bool matches(const Producer& p, uint8_t src, bool fp) {
        return p.valid && p.regWrite && p.fp == fp && p.rd == src && (fp || src != ISA::ZERO);
    }

    Selection select(uint8_t src, bool fp, Stage6 need, const Ahead& a) {
        Selection s;
        struct Candidate { const Producer* p; Stage6 stage; Source source; uint64_t* counter; };
        const Candidate order[] = {
            {&a.mem1, Stage6::MEM1, Source::FromMem1, &stats_.fromMem1},
            {&a.mem2, Stage6::MEM2, Source::FromMem2, &stats_.fromMem2},
            {&a.wb,   Stage6::WB,   Source::FromWb,   &stats_.fromWb},
        };
        for (const Candidate& c : order) {
            if (c.stage <= need) continue;
            if (!matches(*c.p, src, fp)) continue;
            if (!c.p->ready) { s.blocked = true; ++stats_.blocked; return s; }
            s.source = c.source;
            ++*c.counter;
            return s;
        }
        if (need > Stage6::EX) ++stats_.lateRegFile;
        return s;
    }

    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
};
