#pragma once

#include "isa.cpp"
#include <cstdint>

class ExceptionHandler {
public:
    using Word = uint64_t;

    enum SR : unsigned { STATUS = 0, EPC, CAUSE, TVEC, IMASK, IPEND, BADADDR, FCSR, SR_COUNT };

    static constexpr Word STATUS_OVERFLOW = 1u << 0;
    static constexpr Word STATUS_DIVZERO  = 1u << 1;
    static constexpr Word STATUS_IE       = 1u << 2;
    static constexpr Word STATUS_PIE      = 1u << 3;
    static constexpr Word STATUS_TRAP     = 1u << 4;
    static constexpr Word STATUS_WRITABLE = 0xF;

    static constexpr Word FCSR_FLAGS      = 0x1F;
    static constexpr Word FCSR_WRITABLE   = 0xFF;
    static constexpr unsigned FCSR_RM_SHIFT = 5;

    static constexpr unsigned INTERRUPT_LINES = 8;
    static constexpr Word     VECTOR_STRIDE   = 16;

    static bool validSelect(int64_t sel) { return sel >= 0 && sel < static_cast<int64_t>(SR_COUNT); }

    Word read(unsigned sel) const { return sr_[sel]; }

    void write(unsigned sel, Word v) {
        switch (sel) {
            case STATUS: sr_[STATUS] = (sr_[STATUS] & ~STATUS_WRITABLE) | (v & STATUS_WRITABLE); break;
            case IMASK:  sr_[IMASK] = v & ((Word(1) << INTERRUPT_LINES) - 1); break;
            case IPEND:  sr_[IPEND] &= ~v; break;
            case FCSR:   sr_[FCSR] = v & FCSR_WRITABLE; break;
            default:     sr_[sel] = v; break;
        }
    }

    void setStatusBits(Word mask) { sr_[STATUS] |= mask & (STATUS_OVERFLOW | STATUS_DIVZERO); }

    void orFpFlags(Word flags) { sr_[FCSR] |= flags & FCSR_FLAGS; }
    unsigned roundingMode() const { return static_cast<unsigned>((sr_[FCSR] >> FCSR_RM_SHIFT) & 7); }

    bool handlerInstalled() const { return sr_[TVEC] != 0; }
    bool inTrap() const { return (sr_[STATUS] & STATUS_TRAP) != 0; }
    bool canTakeException() const { return handlerInstalled() && !inTrap(); }

    Word enterException(unsigned faultCode, Word faultingPc, Word badValue) {
        sr_[EPC]     = faultingPc;
        sr_[CAUSE]   = faultCode;
        sr_[BADADDR] = badValue;
        beginTrap();
        return sr_[TVEC] + VECTOR_STRIDE * faultCode;
    }

    Word returnFromTrap() {
        sr_[STATUS] &= ~(STATUS_TRAP | STATUS_IE);
        if (sr_[STATUS] & STATUS_PIE) sr_[STATUS] |= STATUS_IE;
        sr_[STATUS] &= ~STATUS_PIE;
        return sr_[EPC];
    }

protected:
    void beginTrap() {
        const bool ie = (sr_[STATUS] & STATUS_IE) != 0;
        sr_[STATUS] &= ~(STATUS_IE | STATUS_PIE);
        if (ie) sr_[STATUS] |= STATUS_PIE;
        sr_[STATUS] |= STATUS_TRAP;
    }

    Word sr_[SR_COUNT] = {};
};
