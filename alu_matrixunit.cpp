#pragma once

#include "alu.cpp"
#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

class MatrixUnit {
public:
    using Word = ISA::Word;

    static constexpr unsigned MAX_LANES = 32;
    static constexpr unsigned MAX_DIM   = ISA::MAX_MATRIX_ROWS;

    struct Config {
        unsigned lanes            = 4;
        unsigned memWordsPerCycle = 1;
        unsigned mulLatency       = 3;
        unsigned divLatency       = 16;
        unsigned dividers         = 1;
    };

    enum class StartResult : uint8_t { Started, Busy, InvalidDimensions, MemoryFault };

    struct Stats {
        uint64_t operations      = 0;
        uint64_t busyCycles      = 0;
        uint64_t loadCycles      = 0;
        uint64_t computeCycles   = 0;
        uint64_t storeCycles     = 0;
        uint64_t vectorSteps     = 0;
        uint64_t laneSlots       = 0;
        uint64_t activeLaneSlots = 0;

        double utilization() const {
            return laneSlots ? static_cast<double>(activeLaneSlots) / static_cast<double>(laneSlots) : 0.0;
        }
    };

    explicit MatrixUnit(std::vector<Word>& dmem) : MatrixUnit(dmem, Config()) {}
    MatrixUnit(std::vector<Word>& dmem, const Config& cfg) : dmem_(dmem), cfg_(cfg) {
        if (cfg.lanes < 1 || cfg.lanes > MAX_LANES) throw std::invalid_argument("MatrixUnit: lanes out of range");
        if (cfg.memWordsPerCycle < 1 || cfg.mulLatency < 1 || cfg.divLatency < 1 || cfg.dividers < 1)
            throw std::invalid_argument("MatrixUnit: configuration values must be at least 1");
    }

    StartResult start(const ISA::InstructionFields& f, Word rdPtr, Word rs1Ptr, Word rs2Ptr) {
        if (busy_) return StartResult::Busy;

        auto dimOk = [](unsigned d) { return d >= 1 && d <= MAX_DIM; };
        unsigned rows = 0, inner = 0, cols = 0, aWords = 0, bWords = 0, outWords = 0;

        switch (f.opcode) {
            case ISA::Opcode::MATADD:
                rows = f.matrixRows; cols = f.matrixCols;
                if (!dimOk(rows) || !dimOk(cols)) return StartResult::InvalidDimensions;
                aWords = bWords = outWords = rows * cols;
                break;
            case ISA::Opcode::MATMUL:
                rows = f.matrixRows; inner = f.matrixInner; cols = f.matrixCols;
                if (!dimOk(rows) || !dimOk(inner) || !dimOk(cols)) return StartResult::InvalidDimensions;
                aWords = rows * inner; bWords = inner * cols; outWords = rows * cols;
                break;
            case ISA::Opcode::MATDETERMINANT:
                rows = f.matrixRows; cols = f.matrixCols;
                if (!dimOk(rows) || rows != cols) return StartResult::InvalidDimensions;
                aWords = rows * rows; outWords = 1;
                break;
            default:
                return StartResult::InvalidDimensions;
        }

        if (!inRange(rs1Ptr, aWords) || (bWords && !inRange(rs2Ptr, bWords)) || !inRange(rdPtr, outWords))
            return StartResult::MemoryFault;

        for (unsigned e = 0; e < aWords; ++e) a_[e] = dmem_[(rs1Ptr >> 3) + e];
        for (unsigned e = 0; e < bWords; ++e) b_[e] = dmem_[(rs2Ptr >> 3) + e];

        ovf_ = false;
        const uint64_t load = ceilDiv(aWords, cfg_.memWordsPerCycle) +
                              (bWords ? ceilDiv(bWords, cfg_.memWordsPerCycle) : 0);
        uint64_t compute = 0;
        switch (f.opcode) {
            case ISA::Opcode::MATADD:         compute = computeAdd(aWords); break;
            case ISA::Opcode::MATMUL:         compute = computeMul(rows, inner, cols); break;
            default:                          compute = computeDet(rows, c_[0]); break;
        }
        const uint64_t store = ceilDiv(outWords, cfg_.memWordsPerCycle);

        outPtr_    = rdPtr;
        outWords_  = outWords;
        remaining_ = load + compute + store;
        busy_      = true;

        ++stats_.operations;
        stats_.loadCycles    += load;
        stats_.computeCycles += compute;
        stats_.storeCycles   += store;
        return StartResult::Started;
    }

    bool step() {
        if (!busy_) return false;
        ++stats_.busyCycles;
        if (--remaining_ != 0) return false;
        for (unsigned e = 0; e < outWords_; ++e) dmem_[(outPtr_ >> 3) + e] = c_[e];
        lastOverflow_ = ovf_;
        busy_ = false;
        return true;
    }

    bool busy() const { return busy_; }
    uint64_t remainingCycles() const { return busy_ ? remaining_ : 0; }
    bool lastOverflow() const { return lastOverflow_; }
    const Stats& stats() const { return stats_; }
    const Config& config() const { return cfg_; }

private:
    static uint64_t ceilDiv(uint64_t a, uint64_t b) { return (a + b - 1) / b; }

    static Word absWord(Word a) { return static_cast<int64_t>(a) < 0 ? Word(0) - a : a; }

    bool inRange(Word ptr, uint64_t words) const {
        const uint64_t bytes = static_cast<uint64_t>(dmem_.size()) * 8;
        return (ptr & 7) == 0 && ptr <= bytes && words * 8 <= bytes - ptr;
    }

    unsigned issue(unsigned active) {
        ++stats_.vectorSteps;
        stats_.laneSlots += cfg_.lanes;
        stats_.activeLaneSlots += active;
        return 1;
    }

    Word addW(Word a, Word b) { const ALU::Result r = ALU::add(a, b); ovf_ |= r.overflow; return r.value; }
    Word subW(Word a, Word b) { const ALU::Result r = ALU::sub(a, b); ovf_ |= r.overflow; return r.value; }
    Word mulW(Word a, Word b) { return ALU::fixedMul(a, b, ovf_); }
    Word divW(Word a, Word b) { return ALU::fixedDiv(a, b, ovf_); }

    uint64_t computeAdd(unsigned n) {
        uint64_t cycles = 0;
        for (unsigned e = 0; e < n; e += cfg_.lanes) {
            const unsigned active = std::min(cfg_.lanes, n - e);
            for (unsigned l = 0; l < active; ++l) c_[e + l] = addW(a_[e + l], b_[e + l]);
            cycles += issue(active);
        }
        return cycles;
    }

    uint64_t computeMul(unsigned rows, unsigned inner, unsigned cols) {
        uint64_t cycles = 0;
        std::array<Word, MAX_LANES> acc{};
        for (unsigned i = 0; i < rows; ++i) {
            for (unsigned g = 0; g < cols; g += cfg_.lanes) {
                const unsigned active = std::min(cfg_.lanes, cols - g);
                acc.fill(0);
                for (unsigned k = 0; k < inner; ++k) {
                    const Word a = a_[i * inner + k];
                    for (unsigned l = 0; l < active; ++l)
                        acc[l] = addW(acc[l], mulW(a, b_[k * cols + g + l]));
                    cycles += issue(active);
                }
                for (unsigned l = 0; l < active; ++l) c_[i * cols + g + l] = acc[l];
            }
        }
        return cycles + cfg_.mulLatency;
    }

    uint64_t computeDet(unsigned n, Word& det) {
        uint64_t cycles = 0;
        det = ALU::FX_SCALE;
        std::array<Word, MAX_DIM> factor{};

        for (unsigned k = 0; k < n; ++k) {
            unsigned piv = k;
            Word best = absWord(a_[k * n + k]);
            for (unsigned i = k + 1; i < n; ++i) {
                const Word v = absWord(a_[i * n + k]);
                if (v > best) { best = v; piv = i; }
            }
            for (unsigned r = k; r < n; r += cfg_.lanes) cycles += issue(std::min(cfg_.lanes, n - r));
            cycles += 1;
            if (!best) { det = 0; return cycles; }

            if (piv != k) {
                for (unsigned j = 0; j < n; ++j) std::swap(a_[k * n + j], a_[piv * n + j]);
                det = Word(0) - det;
                for (unsigned j = 0; j < n; j += cfg_.lanes) cycles += issue(std::min(cfg_.lanes, n - j));
            }

            const Word pivot = a_[k * n + k];
            det = mulW(det, pivot);
            cycles += issue(1);

            const unsigned below = n - k - 1;
            for (unsigned i = k + 1; i < n; ++i) factor[i] = divW(a_[i * n + k], pivot);
            if (below) cycles += ceilDiv(below, cfg_.dividers) * cfg_.divLatency;

            bool updated = false;
            for (unsigned i = k + 1; i < n; ++i) {
                if (!factor[i]) continue;
                for (unsigned j = k; j < n; j += cfg_.lanes) {
                    const unsigned active = std::min(cfg_.lanes, n - j);
                    for (unsigned l = 0; l < active; ++l) {
                        const Word t = mulW(factor[i], a_[k * n + j + l]);
                        a_[i * n + j + l] = subW(a_[i * n + j + l], t);
                    }
                    cycles += issue(active);
                }
                updated = true;
            }
            if (updated) cycles += cfg_.mulLatency;
        }
        return cycles + cfg_.mulLatency;
    }

    std::vector<Word>& dmem_;
    Config cfg_;
    std::array<Word, ALU::MAT_MAX_ELEMS> a_{}, b_{}, c_{};
    bool     ovf_ = false;
    bool     lastOverflow_ = false;
    bool     busy_ = false;
    uint64_t remaining_ = 0;
    Word     outPtr_ = 0;
    unsigned outWords_ = 0;
    Stats    stats_;
};
