#pragma once

#include "exception_handler.cpp"

class InterruptHandler : public ExceptionHandler {
public:
    static constexpr Word     CAUSE_BASE  = 16;
    static constexpr unsigned VECTOR_BASE = 4;

    void raise(unsigned line) {
        if (line < INTERRUPT_LINES) sr_[IPEND] |= Word(1) << line;
    }

    int pendingInterrupt() const {
        if (!(sr_[STATUS] & STATUS_IE) || inTrap() || !handlerInstalled()) return -1;
        const Word p = sr_[IPEND] & sr_[IMASK];
        for (unsigned i = 0; i < INTERRUPT_LINES; ++i)
            if (p & (Word(1) << i)) return static_cast<int>(i);
        return -1;
    }

    Word enterInterrupt(unsigned line, Word nextPc) {
        sr_[IPEND] &= ~(Word(1) << line);
        sr_[EPC]   = nextPc;
        sr_[CAUSE] = CAUSE_BASE + line;
        beginTrap();
        return sr_[TVEC] + VECTOR_STRIDE * (VECTOR_BASE + line);
    }
};
