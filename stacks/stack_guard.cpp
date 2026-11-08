#pragma once

// StackGuard: runtime stack-boundary protection (assignment 2f).
//
// Stack model: full-descending. The stack occupies the data-memory range [low, high).
// SP starts at `high` (empty); PUSH does SP -= 8 then stores at SP; POP loads from SP
// then SP += 8.  Valid stack words therefore live at addresses [low, high).
//
// Checks (made when a memory instruction computes its effective address, so the fault
// is precise and travels down the pipeline like any other MEMORY_FAULT):
//   * SP-based access below `low`            -> STACK_OVERFLOW
//   * SP-based access at/above `high`        -> STACK_UNDERFLOW  (pop of an empty stack)
//   * any non-SP *store* into the red zone   -> STACK_REDZONE    (the `redZone` bytes
//     just below `low`, protecting the neighbouring data from a runaway push)
// A violation is reported to the program as MEMORY_FAULT (code 2) with BADADDR = the
// offending address, so existing trap handlers (TVEC) work unchanged.  The guard also
// records *why* (kind, pc, sp) so the debugger can print a precise stack exception.

#include <cstdint>
#include <string>

class StackGuard {
public:
    enum class Kind : uint8_t { None, Overflow, Underflow, RedZone };

    struct Violation {
        Kind     kind = Kind::None;
        uint64_t pc = 0, addr = 0, sp = 0;
        bool     isStore = false;
    };

    struct Stats {
        uint64_t checks = 0;
        uint64_t violations = 0;
    };

    static const char* name(Kind k) {
        switch (k) {
            case Kind::Overflow:  return "STACK_OVERFLOW";
            case Kind::Underflow: return "STACK_UNDERFLOW";
            case Kind::RedZone:   return "STACK_REDZONE_WRITE";
            default:              return "NONE";
        }
    }

    void configure(uint64_t low, uint64_t high, uint64_t redZone = 64) {
        low_ = low; high_ = high; red_ = redZone; enabled_ = high > low;
        reset();
    }
    void disable() { enabled_ = false; }
    void reset() { last_ = Violation(); stats_ = Stats(); peakBytes_ = 0; }

    bool     enabled() const { return enabled_; }
    uint64_t low()     const { return low_; }
    uint64_t high()    const { return high_; }
    uint64_t redZone() const { return red_; }
    uint64_t capacity() const { return high_ - low_; }

    // Returns the kind of violation (None when the access is fine) and records it.
    Kind check(bool baseIsSp, uint64_t addr, bool isStore, uint64_t pc, uint64_t sp) {
        if (!enabled_) return Kind::None;
        ++stats_.checks;
        Kind k = Kind::None;
        if (baseIsSp) {
            if (addr < low_)        k = Kind::Overflow;
            else if (addr >= high_) k = Kind::Underflow;
        } else if (isStore && red_ != 0 && addr >= low_ - red_ && addr < low_) {
            k = Kind::RedZone;
        }
        if (k != Kind::None) {
            ++stats_.violations;
            last_ = {k, pc, addr, sp, isStore};
        }
        return k;
    }

    // Track the deepest SP seen (high-water mark), called by the debugger every cycle.
    void noteSp(uint64_t sp) {
        if (!enabled_ || sp > high_ || sp < low_) return;
        const uint64_t used = high_ - sp;
        if (used > peakBytes_) peakBytes_ = used;
    }

    const Violation& last() const { return last_; }
    const Stats& stats() const { return stats_; }
    uint64_t peakBytes() const { return peakBytes_; }

private:
    bool enabled_ = false;
    uint64_t low_ = 0, high_ = 0, red_ = 0;
    Violation last_;
    Stats stats_;
    uint64_t peakBytes_ = 0;
};
