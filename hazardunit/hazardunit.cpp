#pragma once

#include "forwarding_unit.hpp"
#include "branch_predictor.hpp"
#include "interlock_unit.hpp"
#include "interrupt_handler.hpp"
#include "exception_handler.hpp"

class UnifiedHazardUnit {
public:
    ForwardingUnit   forwarding;
    RLBranchPredictor predictor;
    InterlockUnit    interlock;
    InterruptHandler interruptHandler;
    ExceptionHandler exceptionHandler;

    void reset() {
        exceptionHandler.clear();
    }
};
