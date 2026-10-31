#pragma once

#include <cstdint>

class InterruptHandler {
public:
    static constexpr uint64_t IRQ_VECTOR_BASE = 0x00000100;

    struct InterruptState {
        bool pending           = false;
        uint8_t irqVector      = 0;
        uint64_t returnPC      = 0;
        bool interruptsEnabled = true;
    };

    inline void setInterruptEnable(bool enable) { state_.interruptsEnabled = enable; }
    inline bool isEnabled() const { return state_.interruptsEnabled; }
    inline bool isPending() const { return state_.pending && state_.interruptsEnabled; }

    inline void raiseIRQ(uint8_t vector, uint64_t currentPC) {
        if (!state_.interruptsEnabled) return;
        state_.pending   = true;
        state_.irqVector = vector;
        state_.returnPC  = currentPC;
    }

    inline uint64_t serviceIRQ() {
        if (!state_.pending) return 0;
        state_.pending = false;
        state_.interruptsEnabled = false; // Disable nested IRQs by default
        return IRQ_VECTOR_BASE + (static_cast<uint64_t>(state_.irqVector) * 16);
    }

    inline uint64_t returnFromInterrupt() {
        state_.interruptsEnabled = true;
        return state_.returnPC;
    }

    const InterruptState& state() const { return state_; }

private:
    InterruptState state_;
};
