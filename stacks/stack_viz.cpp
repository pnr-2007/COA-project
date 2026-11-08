#pragma once

// ASCII stack memory visualizer (assignment 2g).  Draws the stack region top (high
// addresses) to bottom, marks SP, the base/limit, the red zone, words changed since the
// previous frame, and the high-water mark.

#include "pipeline_core.cpp"
#include <cstdio>
#include <sstream>

struct StackVizOptions {
    unsigned freeRows = 3;      // empty slots shown below SP
    unsigned maxRows  = 24;     // cap on used rows drawn (middle is elided)
};

class StackViz {
public:
    using Options = StackVizOptions;

    static std::string render(const PipelineCore& p, const std::vector<ISA::Word>& mem,
                              const std::vector<ISA::Word>* prev, const Options& o = Options()) {
        const StackGuard& g = p.stackGuard();
        std::ostringstream os;
        if (!g.enabled()) return "stack protection is not configured (use `.stack N` in the program or `stack <low> <high>`)\n";
        const uint64_t sp = p.reg(ISA::SP), low = g.low(), high = g.high();
        const bool spOk = sp >= low && sp <= high && (sp % 8 == 0);
        const uint64_t usedBytes = spOk ? high - sp : 0;
        char b[160];
        std::snprintf(b, sizeof b, " STACK  [0x%llX .. 0x%llX)  full-descending, 8-byte slots\n",
                      (unsigned long long)low, (unsigned long long)high);
        os << b;
        const unsigned barW = 40;
        const unsigned fill = spOk ? static_cast<unsigned>(usedBytes * barW / g.capacity()) : barW;
        std::snprintf(b, sizeof b, " used %llu/%llu B (%llu slots)  peak %llu B  [",
                      (unsigned long long)usedBytes, (unsigned long long)g.capacity(),
                      (unsigned long long)(usedBytes / 8), (unsigned long long)g.peakBytes());
        os << b << std::string(fill, '#') << std::string(barW - fill, '.') << "]\n";
        const std::string rule = " +----------+--------------------+----------------------+--------\n";
        os << rule << " | address  | hex (64-bit)       | signed decimal       | \n" << rule;

        auto word = [&](uint64_t a) { return a / 8 < mem.size() ? mem[a / 8] : 0; };
        auto row = [&](uint64_t a, const std::string& tag) {
            const ISA::Word v = word(a);
            const bool changed = prev && a / 8 < prev->size() && (*prev)[a / 8] != v;
            std::snprintf(b, sizeof b, " |%c%08llX | 0x%016llX | %20lld | %s\n", changed ? '*' : ' ',
                          (unsigned long long)a, (unsigned long long)v, (long long)v, tag.c_str());
            os << b;
        };
        auto banner = [&](const std::string& t) {
            std::snprintf(b, sizeof b, " |==========| %-44s\n", t.c_str()); os << b;
        };

        if (sp > high) {                       // SP above the base: underflow state
            banner("!! SP ABOVE BASE: stack UNDERFLOW (SP=0x" + hex(sp) + ")");
        }
        banner("BASE 0x" + hex(high) + "  (empty stack: SP == BASE)");
        if (spOk) {
            const uint64_t slots = usedBytes / 8;
            const unsigned shown = static_cast<unsigned>(std::min<uint64_t>(slots, o.maxRows));
            const unsigned top = shown / 2 + shown % 2;
            for (unsigned i = 0; i < slots && i < top; ++i) row(high - 8 * (i + 1), i == slots - 1 ? "<= SP" : "");
            if (slots > shown) {
                std::snprintf(b, sizeof b, " |   ...    | %llu slots elided\n", (unsigned long long)(slots - shown)); os << b;
            }
            for (uint64_t i = std::max<uint64_t>(top, slots - (shown - top)); i < slots; ++i)
                row(high - 8 * (i + 1), i == slots - 1 ? "<= SP" : "");
            if (slots == 0) {
                std::snprintf(b, sizeof b, " |          | (empty)                                     <= SP\n"); os << b;
            }
            // a few free slots below SP
            for (unsigned i = 0; i < o.freeRows && sp - 8 * (i + 1) + 8 > low + 0 && sp >= low + 8 * (i + 1); ++i) {
                const uint64_t a = sp - 8 * (i + 1);
                std::snprintf(b, sizeof b, " | %08llX  | . . . free . . .\n", (unsigned long long)a); os << b;
            }
        } else if (sp < low) {
            banner("!! SP BELOW LIMIT: stack OVERFLOW (SP=0x" + hex(sp) + ")");
        } else {
            banner("!! SP misaligned (SP=0x" + hex(sp) + ")");
        }
        os << " |   ...    |\n";
        banner("LIMIT 0x" + hex(low) + "  (an SP-based access below this faults)");
        if (g.redZone()) banner("red zone: " + std::to_string(g.redZone()) + " B below LIMIT (non-SP stores fault)");
        os << rule;
        os << " ('*' = changed since the previous frame)\n";
        return os.str();
    }

private:
    static std::string hex(uint64_t v) { char b[24]; std::snprintf(b, sizeof b, "%llX", (unsigned long long)v); return b; }
};
