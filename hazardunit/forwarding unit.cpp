#pragma once

#include "exec.cpp"

class ForwardingUnit {
public:
    using Word = ISA::Word;

    enum class Source : uint8_t { RegFile, ExMa, MaRw };

    struct Stats {
        uint64_t fromExMa = 0;
        uint64_t fromMaRw = 0;
    };

    struct Producer {
        bool valid = false, regWrite = false, memRead = false;
        uint8_t rd = 0;
        Word value = 0;
    };

    static bool matches(const Producer& p, uint8_t src) {
        return p.valid && p.regWrite && p.rd != ISA::ZERO && p.rd == src;
    }

    Source select(uint8_t src, const Producer& exMa, const Producer& maRw) {
        if (matches(exMa, src)) { ++stats_.fromExMa; return Source::ExMa; }
        if (matches(maRw, src)) { ++stats_.fromMaRw; return Source::MaRw; }
        return Source::RegFile;
    }

    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
};
