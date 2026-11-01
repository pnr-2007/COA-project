#pragma once

#include "trap_state.cpp"

class ExceptionHandler {
public:
    using Word = TrapState::Word;

    static bool canTake(const TrapState& t) { return t.handlerInstalled() && !t.inTrap(); }

    static Word enter(TrapState& t, unsigned faultCode, Word faultingPc, Word badValue) {
        t.set(TrapState::EPC, faultingPc);
        t.set(TrapState::CAUSE, faultCode);
        t.set(TrapState::BADADDR, badValue);
        t.beginTrap();
        return t.read(TrapState::TVEC) + TrapState::VECTOR_STRIDE * faultCode;
    }
};
