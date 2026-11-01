#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

class RLBranchPredictor {
public:
    struct Config {
        bool     enabled      = true;
        unsigned tableBits    = 10;
        unsigned shortHistory = 4;
        unsigned longHistory  = 12;
        unsigned alphaShift   = 2;
        unsigned gammaShift   = 2;
        unsigned epsilonShift = 0;
        int32_t  qLimit       = 2048;
    };

    struct Prediction {
        bool     taken = false;
        bool     isBranch = false;
        std::array<uint32_t, 3> idx{};
        uint64_t history = 0;
        uint64_t pc = 0;
    };

    struct Stats {
        uint64_t predictions = 0;
        uint64_t correct = 0;
        uint64_t explored = 0;
        double accuracy() const { return predictions ? double(correct) / double(predictions) : 0.0; }
    };

    RLBranchPredictor() : RLBranchPredictor(Config()) {}
    explicit RLBranchPredictor(const Config& c) : cfg_(c) {
        for (auto& t : q_) t.assign(size_t(1) << cfg_.tableBits, std::array<int32_t, 2>{0, 0});
    }

    Prediction predict(uint64_t pc, bool backward) {
        Prediction p;
        p.isBranch = true;
        p.pc = pc;
        p.history = history_;
        indices(pc, history_, p.idx);
        if (!cfg_.enabled) return p;
        const int32_t qn = qSum(p.idx, 0), qt = qSum(p.idx, 1);
        p.taken = qt > qn ? true : (qt < qn ? false : backward);
        if (cfg_.epsilonShift && (nextRandom() & ((1u << cfg_.epsilonShift) - 1)) == 0) {
            p.taken = !p.taken;
            ++stats_.explored;
        }
        return p;
    }

    void update(const Prediction& p, bool taken) {
        ++stats_.predictions;
        if (p.taken == taken) ++stats_.correct;
        if (!cfg_.enabled) return;

        const uint64_t nextHistory = ((history_ << 1) | (taken ? 1u : 0u));
        std::array<uint32_t, 3> nextIdx{};
        indices(p.pc, nextHistory, nextIdx);
        const int32_t best = std::max(qSum(nextIdx, 0), qSum(nextIdx, 1));
        const int32_t reward = 256;

        for (unsigned a = 0; a < 2; ++a) {
            const int32_t r = ((a == 1) == taken) ? reward : -reward;
            const int32_t target = r + (best >> cfg_.gammaShift);
            const int32_t delta = target - qSum(p.idx, a);
            const int32_t step = delta / int32_t(3u << cfg_.alphaShift);
            for (unsigned t = 0; t < 3; ++t) {
                int32_t& q = q_[t][p.idx[t]][a];
                q += step;
                if (q >  cfg_.qLimit) q =  cfg_.qLimit;
                if (q < -cfg_.qLimit) q = -cfg_.qLimit;
            }
        }
        history_ = nextHistory;
    }

    const Stats&  stats() const { return stats_; }
    const Config& config() const { return cfg_; }

private:
    static uint64_t mask(unsigned bits) { return bits >= 64 ? ~uint64_t(0) : (uint64_t(1) << bits) - 1; }

    void indices(uint64_t pc, uint64_t hist, std::array<uint32_t, 3>& out) const {
        const uint64_t tm = mask(cfg_.tableBits);
        const uint64_t w = pc >> 2;
        const uint64_t hs = hist & mask(cfg_.shortHistory);
        const uint64_t hl = hist & mask(cfg_.longHistory);
        out[0] = uint32_t(w & tm);
        out[1] = uint32_t((w ^ (hs << 3) ^ (hs >> 1)) & tm);
        out[2] = uint32_t((w ^ (hl * 0x9E3779B1ull >> 7) ^ (hl << 1)) & tm);
    }

    int32_t qSum(const std::array<uint32_t, 3>& idx, unsigned a) const {
        return q_[0][idx[0]][a] + q_[1][idx[1]][a] + q_[2][idx[2]][a];
    }

    uint32_t nextRandom() {
        const uint32_t bit = ((lfsr_ >> 0) ^ (lfsr_ >> 2) ^ (lfsr_ >> 3) ^ (lfsr_ >> 5)) & 1u;
        lfsr_ = (lfsr_ >> 1) | (bit << 15);
        return lfsr_;
    }

    Config cfg_;
    std::array<std::vector<std::array<int32_t, 2>>, 3> q_;
    uint64_t history_ = 0;
    uint32_t lfsr_ = 0xACE1u;
    Stats stats_;
};

class ReturnStack {
public:
    explicit ReturnStack(size_t depth = 16) : depth_(depth) {}
    void push(uint64_t a) {
        if (s_.size() == depth_) s_.erase(s_.begin());
        s_.push_back(a);
    }
    bool top(uint64_t& a) const {
        if (s_.empty()) return false;
        a = s_.back();
        return true;
    }
    void pop() { if (!s_.empty()) s_.pop_back(); }

private:
    size_t depth_;
    std::vector<uint64_t> s_;
};
