#pragma once

#include "forwarding_unit.cpp"
#include "interlock_unit.cpp"
#include "branch_predictor.cpp"
#include "interrupt_handler.cpp"

class HazardUnit {
public:
    explicit HazardUnit(const RLBranchPredictor::Config& predictor) : predictor(predictor) {}

    ForwardingUnit    forwarding;
    InterlockUnit     interlock;
    RLBranchPredictor predictor;
    ReturnStack       returnStack;
    InterruptHandler interrupts;
};
