#pragma once

#include "isa.cpp"
#include "exec.cpp"

struct InterlockSignals {
    bool stallFetch   = false;
    bool stallDecode  = false;
    bool stallExecute = false;
    bool flushDecode  = false;
    bool flushExecute = false;
};

class InterlockUnit {
public:
    static inline InterlockSignals evaluate(const Exec::RegUse& idUse,
                                            bool exMemRead, uint8_t exRd,
                                            bool matrixUnitBusy,
                                            bool aluBusy,
                                            bool branchMispredicted) {
        InterlockSignals sigs;

        // 1. Branch Misprediction Flush
        if (branchMispredicted) {
            sigs.flushDecode  = true;
            sigs.flushExecute = true;
            return sigs;
        }

        // 2. Load-Use Hazard Detection
        const bool loadUseHazard = exMemRead && (exRd != ISA::ZERO) &&
                                  ((idUse.readsA && idUse.a == exRd) ||
                                   (idUse.readsB && idUse.b == exRd) ||
                                   (idUse.readsC && idUse.c == exRd));

        if (loadUseHazard) {
            sigs.stallFetch   = true;
            sigs.stallDecode  = true;
            sigs.flushExecute = true; // Insert NOP bubble into EX
        }

        // 3. Structural & Multi-cycle Execution Interlocks
        if (matrixUnitBusy || aluBusy) {
            sigs.stallFetch   = true;
            sigs.stallDecode  = true;
            sigs.stallExecute = true;
        }

        return sigs;
    }
};
