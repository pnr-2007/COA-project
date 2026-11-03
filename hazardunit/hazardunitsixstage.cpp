#pragma once
#include "control_unit_signals.h"

struct HazardInputs {
    bool exception = false;
    bool interrupt = false;
    bool memFreeze = false;
    bool redirect = false;     // Branch misprediction
    bool loadUse = false;
};

struct PipelineControl {
    bool stallFetch = false;
    bool stallDecode = false;
    bool flushDecode = false;
    bool flushExecute = false;
    bool flushMem1 = false;
    bool flushMem2 = false;
};

class InterlockUnit {
public:
    bool detectLoadUse(uint8_t idRs1, uint8_t idRs2, bool readsA, bool readsB, 
                       const ExMem1Control& exMem1, const Mem1Mem2Control& mem1Mem2) {
        
        // Hazard if instruction in EX is a load writing to a required register
        bool exHazard = exMem1.valid && exMem1.memRead && exMem1.dest != 0 && 
                        ((readsA && exMem1.dest == idRs1) || (readsB && exMem1.dest == idRs2));
        
        // Hazard if instruction in MEM1 is a load writing to a required register
        bool mem1Hazard = mem1Mem2.valid && mem1Mem2.memRead && mem1Mem2.dest != 0 && 
                          ((readsA && mem1Mem2.dest == idRs1) || (readsB && mem1Mem2.dest == idRs2));
                          
        return exHazard || mem1Hazard;
    }
};

class UnifiedHazardUnit {
public:
    InterlockUnit interlock;

    PipelineControl resolve(const HazardInputs& in) {
        PipelineControl c;
        if (in.exception || in.interrupt) {
            // Flush all stages behind the committing stage
            c.flushDecode = true;
            c.flushExecute = true;
            c.flushMem1 = true;
            c.flushMem2 = true;
        } else if (in.redirect) {
            // Flush stages behind the branching stage (EX)
            c.flushDecode = true;
            c.flushExecute = true; 
        } else if (in.memFreeze) {
            // Stall fetch and decode if memory subsystem is busy
            c.stallFetch = true;
            c.stallDecode = true;
        } else if (in.loadUse) {
            // Stall early stages, inject bubble into EX
            c.stallFetch = true;
            c.stallDecode = true;
            c.flushExecute = true; 
        }
        return c;
    }
};
