#pragma once

#include "trap_state.cpp"

class InterruptHandler {
public:
    using Word = TrapState::Word;

    static constexpr Word     CAUSE_BASE  = 16;
    static constexpr unsigned VECTOR_BASE = 4;

    static void raise(TrapState& t, unsigned line) {
        if (line < TrapState::INTERRUPT_LINES) t.set(TrapState::IPEND, t.read(TrapState::IPEND) | (Word(1) << line));
    }

    static int pending(const TrapState& t) {
        if (!(t.read(TrapState::STATUS) & TrapState::STATUS_IE) || t.inTrap() || !t.handlerInstalled()) return -1;
        const Word p = t.read(TrapState::IPEND) & t.read(TrapState::IMASK);
        for (unsigned i = 0; i < TrapState::INTERRUPT_LINES; ++i)
            if (p & (Word(1) << i)) return static_cast<int>(i);
        return -1;
    }

    static Word enter(TrapState& t, unsigned line, Word nextPc) {
        t.set(TrapState::IPEND, t.read(TrapState::IPEND) & ~(Word(1) << line));
        t.set(TrapState::EPC, nextPc);
        t.set(TrapState::CAUSE, CAUSE_BASE + line);
        t.beginTrap();
        return t.read(TrapState::TVEC) + TrapState::VECTOR_STRIDE * (VECTOR_BASE + line);
    }
};
