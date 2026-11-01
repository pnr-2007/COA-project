#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <random>

class QLearningBranchPredictor {
public:
    struct Prediction {
        bool predictTaken = false;
        size_t stateIndex = 0;
        uint8_t actionIndex = 0;
    };

    QLearningBranchPredictor(size_t tableSize = 1024, float alpha = 0.1f, float gamma = 0.9f, float epsilon = 0.05f)
        : tableSize_(tableSize), alpha_(alpha), gamma_(gamma), epsilon_(epsilon), gahr_(0) {
        
        // Q-Table initialized to 0.0 for states s, actions a in {0: Not-Taken, 1: Taken}
        qTable_.resize(tableSize_, std::vector<float>(2, 0.0f));
        rng_.seed(42);
    }

    // Evaluates state and generates direction prediction (IF Stage)
    Prediction predict(uint64_t pc) {
        Prediction pred;
        pred.stateIndex = calculateState(pc);

        // Epsilon-greedy exploration choice
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        if (dist(rng_) < epsilon_) {
            std::uniform_int_distribution<int> actionDist(0, 1);
            pred.actionIndex = static_cast<uint8_t>(actionDist(rng_));
        } else {
            // Argmax Q-value selection
            float qNT = qTable_[pred.stateIndex][0];
            float qT  = qTable_[pred.stateIndex][1];
            pred.actionIndex = (qT > qNT) ? 1 : 0;
        }

        pred.predictTaken = (pred.actionIndex == 1);
        return pred;
    }

    // Updates Q-table weights based on execution feedback (EX Stage)
    void update(uint64_t pc, const Prediction& pred, bool actualTaken, uint64_t nextPC) {
        size_t currentState = pred.stateIndex;
        uint8_t action = pred.actionIndex;
        size_t nextState = calculateState(nextPC);

        // Reward function evaluation
        float reward = (actualTaken == pred.predictTaken) ? 1.0f : -1.0f;

        // Max Q-value for next state: max_a' Q(s', a')
        float maxNextQ = std::max(qTable_[nextState][0], qTable_[nextState][1]);

        // Q-Learning Bellman Update Equation
        float currentQ = qTable_[currentState][action];
        qTable_[currentState][action] = currentQ + alpha_ * (reward + (gamma_ * maxNextQ) - currentQ);

        // Shift Global Branch History Register (BHR)
        gahr_ = ((gahr_ << 1) | (actualTaken ? 1 : 0)) & 0x0F;
    }

private:
    size_t calculateState(uint64_t pc) const {
        // Gshare indexing: PC hash XORed with Global History Register (BHR)
        uint64_t pcHash = (pc >> 2) & (tableSize_ - 1);
        return (pcHash ^ gahr_) & (tableSize_ - 1);
    }

    size_t tableSize_;
    float alpha_;   // Learning rate
    float gamma_;   // Discount factor
    float epsilon_; // Exploration rate
    uint8_t gahr_;  // 4-bit Global Branch History Register

    std::vector<std::vector<float>> qTable_;
    std::mt19937 rng_;
};
