#pragma once

#include "isa.cpp"
#include "exec.cpp"
#include <cstdint>

enum class ForwardSource : uint8_t {
    NONE = 0,   // Read from Register File
    EX_MA = 1,  // Forward from EX/MA pipeline register
    MA_RW = 2   // Forward from MA/RW pipeline register
};

struct ForwardingSignals {
    ForwardSource forwardA = ForwardSource::NONE;
    ForwardSource forwardB = ForwardSource::NONE;
    ForwardSource forwardC = ForwardSource::NONE;
};

class ForwardingUnit {
public:
    using Word = ISA::Word;

    static inline ForwardingSignals evaluate(const Exec::RegUse& idUse,
                                             bool exMaRegWrite, uint8_t exMaRd,
                                             bool maRwRegWrite, uint8_t maRwRd) {
        ForwardingSignals sigs;

        // Forwarding for Operand A (rs1)
        if (idUse.readsA && idUse.a != ISA::ZERO) {
            if (exMaRegWrite && exMaRd == idUse.a) {
                sigs.forwardA = ForwardSource::EX_MA;
            } else if (maRwRegWrite && maRwRd == idUse.a) {
                sigs.forwardA = ForwardSource::MA_RW;
            }
        }

        // Forwarding for Operand B (rs2)
        if (idUse.readsB && idUse.b != ISA::ZERO) {
            if (exMaRegWrite && exMaRd == idUse.b) {
                sigs.forwardB = ForwardSource::EX_MA;
            } else if (maRwRegWrite && maRwRd == idUse.b) {
                sigs.forwardB = ForwardSource::MA_RW;
            }
        }

        // Forwarding for Operand C (store data / matrix register)
        if (idUse.readsC && idUse.c != ISA::ZERO) {
            if (exMaRegWrite && exMaRd == idUse.c) {
                sigs.forwardC = ForwardSource::EX_MA;
            } else if (maRwRegWrite && maRwRd == idUse.c) {
                sigs.forwardC = ForwardSource::MA_RW;
            }
        }

        return sigs;
    }

    static inline Word selectOperand(ForwardSource source, Word regFileVal, Word exMaVal, Word maRwVal) {
        switch (source) {
            case ForwardSource::EX_MA: return exMaVal;
            case ForwardSource::MA_RW: return maRwVal;
            case ForwardSource::NONE:
            default:                   return regFileVal;
        }
    }
};
