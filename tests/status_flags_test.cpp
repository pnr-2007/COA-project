#include PIPE_HEADER
#include "iss.cpp"
#include <iostream>
#include <random>

using Op   = ISA::Opcode;
using Word = ISA::Word;
using Prog = std::vector<ISA::InstrWord>;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { ++fails; std::cout << "  FAIL line " << __LINE__ << ": " #c "\n"; } } while (0)

static ISA::InstrWord RR(Op op, int rd, int rs1, int rs2) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; f.rs2 = rs2; return ISA::encode(f);
}
static ISA::InstrWord RI(Op op, int rd, int rs1, int32_t imm) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; f.modifier = 1; f.immediate = imm; return ISA::encode(f);
}
static ISA::InstrWord UN(Op op, int rd, int rs1) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; return ISA::encode(f);
}
static ISA::InstrWord HALT() { ISA::InstructionFields f; f.opcode = Op::HALT; return ISA::encode(f); }
static ISA::InstrWord NOP()  { ISA::InstructionFields f; f.opcode = Op::NOP; return ISA::encode(f); }
static ISA::InstrWord RDSR(int rd, int sel) {
    ISA::InstructionFields f; f.opcode = Op::RDSR; f.rd = rd; f.immediate = sel; return ISA::encode(f);
}
static ISA::InstrWord WRSR(int sel, int rs1) {
    ISA::InstructionFields f; f.opcode = Op::WRSR; f.rs1 = rs1; f.immediate = sel; return ISA::encode(f);
}

static constexpr int STATUS = ExceptionHandler::STATUS;
static constexpr Word OVF = 1, DZ = 2;

template <class P>
static Word runStatus(const Prog& p, std::vector<Word> data = {}) {
    typename P::Config c; c.memory.timing = false; c.longOpCycles = 3;
    P pl(c); pl.loadProgram(p);
    for (size_t i = 0; i < data.size(); ++i) pl.pokeDmem(i, data[i]);
    pl.run(100000);
    return pl.reg(1);
}

static Word issStatus(const Prog& p) {
    ISS s; s.loadProgram(p); s.run(100000);
    return s.reg(1);
}

template <class P>
static void directed(const char* name) {
    const Word MAXQ = static_cast<Word>(INT64_MAX);
    struct Case { const char* what; Prog prog; Word want; };
    const Case cases[] = {
        {"add no overflow",   { RI(Op::ADD, 2, 0, 5), RR(Op::ADD, 3, 2, 2), RDSR(1, STATUS), HALT() }, 0},
        {"add overflow",      { RI(Op::ADD, 2, 0, 1), RI(Op::LSL, 2, 2, 63), RR(Op::SUB, 3, 2, 2), RR(Op::ADD, 4, 2, 2), RDSR(1, STATUS), HALT() }, OVF},
        {"sub overflow",      { RI(Op::ADD, 2, 0, 1), RI(Op::LSL, 2, 2, 62), RI(Op::LSL, 3, 2, 1), RR(Op::SUB, 4, 0, 3), RDSR(1, STATUS), HALT() }, OVF},
        {"div by zero",       { RI(Op::ADD, 2, 0, 7), RR(Op::DIV, 3, 2, 0), RDSR(1, STATUS), HALT() }, OVF | DZ},
        {"mod by zero",       { RI(Op::ADD, 2, 0, 7), RR(Op::MOD, 3, 2, 0), RDSR(1, STATUS), HALT() }, OVF | DZ},
        {"log of zero",       { UN(Op::LOG, 3, 0), RDSR(1, STATUS), HALT() }, OVF | DZ},
        {"exp saturates",     { RI(Op::ADD, 2, 0, 40), RI(Op::LSL, 2, 2, 32), UN(Op::EXP, 3, 2), RDSR(1, STATUS), HALT() }, OVF},
        {"exp in range",      { RI(Op::ADD, 2, 0, 5), RI(Op::LSL, 2, 2, 32), UN(Op::EXP, 3, 2), RDSR(1, STATUS), HALT() }, 0},
        {"pow zero neg",      { RI(Op::ADD, 2, 0, 0), RI(Op::ADD, 3, 0, 1), RI(Op::LSL, 3, 3, 32), RR(Op::SUB, 3, 0, 3), RR(Op::POW, 4, 2, 3), RDSR(1, STATUS), HALT() }, OVF | DZ},
        {"pow negative base", { RI(Op::ADD, 2, 0, 1), RI(Op::LSL, 2, 2, 32), RR(Op::SUB, 2, 0, 2), RR(Op::POW, 4, 2, 2), RDSR(1, STATUS), HALT() }, OVF},
        {"pow ok",            { RI(Op::ADD, 2, 0, 2), RI(Op::LSL, 2, 2, 32), RR(Op::POW, 4, 2, 2), RDSR(1, STATUS), HALT() }, 0},
        {"sticky and clear",  { RR(Op::DIV, 3, 2, 0), RDSR(5, STATUS), RI(Op::ADD, 6, 0, 0), WRSR(STATUS, 6), RR(Op::ADD, 7, 7, 7), RDSR(1, STATUS), HALT() }, 0},
        {"flag after clear",  { RI(Op::ADD, 6, 0, 0), WRSR(STATUS, 6), RR(Op::DIV, 3, 2, 0), RDSR(1, STATUS), HALT() }, OVF | DZ},
        {"back to back",      { RI(Op::ADD, 2, 0, 1), RR(Op::DIV, 3, 2, 0), RR(Op::DIV, 4, 2, 0), RR(Op::ADD, 5, 3, 4), RDSR(1, STATUS), HALT() }, OVF | DZ},
    };
    (void)MAXQ;
    for (const Case& c : cases) {
        const Word iss = issStatus(c.prog);
        const Word got = runStatus<P>(c.prog);
        if (got != iss || (iss & 3) != c.want) {
            ++fails;
            std::cout << "  FAIL " << name << " / " << c.what << ": pipe " << got << " iss " << iss << " want " << c.want << "\n";
        }
    }
}

template <class P>
static void faultingNoFlags(const char* name) {
    Prog p = { RR(Op::DIV, 3, 2, 0), 0xFFFFFFFFu, HALT() };
    typename P::Config c; c.memory.timing = false;
    P pl(c); pl.loadProgram(p); pl.run(1000);
    ISS s; s.loadProgram(p); s.run(1000);
    CHECK(pl.fault() == s.fault());
    CHECK(pl.specialRegs().read(STATUS) == s.specialRegs().read(STATUS));
    Prog q = { UN(Op::LOG, 3, 0), 0xFFFFFFFFu, HALT() };
    typename P::Config c2; c2.memory.timing = false;
    P pq(c2); pq.loadProgram(q); pq.run(1000);
    ISS sq; sq.loadProgram(q); sq.run(1000);
    CHECK(pq.specialRegs().read(STATUS) == sq.specialRegs().read(STATUS));
    (void)name;
}

template <class P>
static void randomized(const char* name) {
    std::mt19937 g(77);
    auto pick = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(g); };
    const Op ops[] = { Op::ADD, Op::SUB, Op::MUL, Op::DIV, Op::MOD, Op::POW, Op::LOG, Op::EXP, Op::LSL, Op::AND };
    int withFlags = 0, programs = 0;
    for (int n = 0; n < 400; ++n) {
        Prog p;
        for (int r = 1; r <= 8; ++r) p.push_back(RI(Op::ADD, r, 0, pick(-3, 3)));
        for (int i = 0; i < 30; ++i) {
            const int k = pick(0, 9);
            if (k == 0) p.push_back(RDSR(pick(1, 8), STATUS));
            else if (k == 1) p.push_back(WRSR(STATUS, pick(0, 8)));
            else if (k == 2) p.push_back(RI(Op::LSL, pick(1, 8), pick(1, 8), pick(0, 40)));
            else p.push_back(RR(ops[pick(0, 9)], pick(1, 8), pick(0, 8), pick(0, 8)));
        }
        p.push_back(HALT());
        typename P::Config c; c.memory.timing = pick(0, 1) == 0; c.longOpCycles = 1 + pick(0, 3);
        P pl(c); pl.loadProgram(p); pl.run(200000);
        ISS s; s.loadProgram(p); s.run(200000);
        bool same = pl.halted() == s.halted() && pl.fault() == s.fault() && pl.specialRegs().read(STATUS) == s.specialRegs().read(STATUS);
        for (unsigned r = 0; r < 15; ++r) same = same && pl.reg(r) == s.reg(r);
        if (!same) { ++fails; std::cout << "  FAIL " << name << " random program " << n << "\n"; if (fails > 5) return; }
        if (s.specialRegs().read(STATUS) & 3) ++withFlags;
        ++programs;
    }
    CHECK(withFlags > 100);
    std::cout << name << ": random programs " << programs << ", with STATUS flags set " << withFlags << "\n";
}

int main() {
    directed<PIPE_CLASS>(PIPE_NAME);
    faultingNoFlags<PIPE_CLASS>(PIPE_NAME);
    randomized<PIPE_CLASS>(PIPE_NAME);

    {
        Word out; uint8_t fl;
        CHECK(Exec::alu(Op::ADD, static_cast<Word>(INT64_MAX), 1, out, fl) && fl == OVF);
        CHECK(Exec::alu(Op::ADD, 1, 2, out, fl) && fl == 0 && out == 3);
        CHECK(Exec::alu(Op::DIV, 1, 0, out, fl) && fl == (OVF | DZ));
        CHECK(Exec::alu(Op::MUL, static_cast<Word>(INT64_MAX), static_cast<Word>(INT64_MAX), out, fl) && fl == OVF);
        CHECK(Exec::alu(Op::AND, 1, 1, out, fl) && fl == 0);
    }

    std::cout << (fails ? "STATUS FLAG CHECKS FAILED\n" : "all status flag checks passed\n");
    return fails ? 1 : 0;
}
