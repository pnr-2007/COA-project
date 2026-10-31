#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

class RLBranchPredictor {
public:
    struct QState {
        double qNotTaken = 0.0;
        double qTaken    = 0.0;
    };

    struct PredictionResult {
        bool predictedTaken = false;
        uint64_t targetPC = 0;
        uint32_t stateHash = 0;
    };

    struct RLStats {
        uint64_t totalBranches  = 0;
        uint64_t correctPreads  = 0;
        uint64_t mispredictions = 0;

        double accuracy() const {
            return totalBranches ? (static_cast<double>(correctPreads) / totalBranches) * 100.0 : 0.0;
        }
    };

    explicit RLBranchPredictor(double alpha = 0.1, double gamma = 0.95, double epsilon = 0.05)
        : alpha_(alpha), gamma_(gamma), epsilon_(epsilon) {}

    inline PredictionResult predict(uint64_t pc, int32_t branchImmediate, bool isReturn, uint64_t lrVal) {
        PredictionResult pred;
        const uint32_t stateKey = hashState(pc);
        pred.stateHash = stateKey;

        const QState& state = qTable_[stateKey];
        pred.predictedTaken = (state.qTaken >= state.qNotTaken);

        if (isReturn) {
            pred.targetPC = lrVal;
            pred.predictedTaken = true;
        } else if (pred.predictedTaken) {
            pred.targetPC = pc + static_cast<uint64_t>(static_cast<int64_t>(branchImmediate) * 4);
        } else {
            pred.targetPC = pc + 4;
        }

        return pred;
    }

    inline void update(uint64_t pc, uint32_t stateKey, bool actualTaken, bool predictedTaken) {
        ++stats_.totalBranches;

        QState& state = qTable_[stateKey];
        const bool hit = (actualTaken == predictedTaken);

        if (hit) ++stats_.correctPreads;
        else     ++stats_.mispredictions;

        const double reward = hit ? 1.0 : -1.0;
        const uint8_t nextGhr = ((ghr_ << 1) | (actualTaken ? 1 : 0)) & 0xFF;
        const uint32_t nextStateKey = (static_cast<uint32_t>(pc >> 2) ^ nextGhr) & TABLE_MASK;
        const QState& nextState = qTable_[nextStateKey];
        const double maxNextQ = std::max(nextState.qNotTaken, nextState.qTaken);

        if (actualTaken) {
            state.qTaken += alpha_ * (reward + gamma_ * maxNextQ - state.qTaken);
        } else {
            state.qNotTaken += alpha_ * (reward + gamma_ * maxNextQ - state.qNotTaken);
        }

        ghr_ = nextGhr;
    }

    const RLStats& stats() const { return stats_; }

private:
    static constexpr size_t TABLE_SIZE = 256;
    static constexpr uint32_t TABLE_MASK = TABLE_SIZE - 1;

    inline uint32_t hashState(uint64_t pc) const {
        return (static_cast<uint32_t>(pc >> 2) ^ ghr_) & TABLE_MASK;
    }

    double alpha_;
    double gamma_;
    double epsilon_;
    uint8_t ghr_ = 0;
    std::array<QState, TABLE_SIZE> qTable_{};
    RLStats stats_;
};
