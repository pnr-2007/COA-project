#pragma once

#include "forwarding_unit6.cpp"
#include "interlock_unit6.cpp"
#include "control_unit6.cpp"
#include "branch_predictor.cpp"
#include "interrupt_handler.cpp"

class HazardUnit6 {
public:
    struct Config {
        Stage6 branchResolveStage = Stage6::EX;
        RLBranchPredictor::Config predictor;
    };

    HazardUnit6() : HazardUnit6(Config()) {}
    explicit HazardUnit6(const Config& c) : predictor(c.predictor), cfg_(c) {}

    unsigned mispredictPenalty() const { return ControlUnit6::redirectPenalty(cfg_.branchResolveStage); }
    const Config& config() const { return cfg_; }

    ForwardingUnit6   forwarding;
    InterlockUnit6    interlock;
    RLBranchPredictor predictor;
    ReturnStack       returnStack;
    InterruptHandler  interrupts;

private:
    Config cfg_;
};
