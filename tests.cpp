#include "pipeline.cpp"
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
static ISA::InstrWord R1(Op op, int rd, int rs1) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; return ISA::encode(f);
}
static ISA::InstrWord LD(int rd, int base, int32_t off) {
    ISA::InstructionFields f; f.opcode = Op::LOAD; f.rd = rd; f.rs1 = base; f.immediate = off; return ISA::encode(f);
}
static ISA::InstrWord ST(int data, int base, int32_t off) {
    ISA::InstructionFields f; f.opcode = Op::STORE; f.rd = data; f.rs1 = base; f.immediate = off; return ISA::encode(f);
}
static ISA::InstrWord BR(Op op, int rs1, int rs2, int32_t imm) {
    ISA::InstructionFields f; f.opcode = op; f.rs1 = rs1; f.rs2 = rs2; f.immediate = imm; return ISA::encode(f);
}
static ISA::InstrWord JT(Op op, int32_t imm) {
    ISA::InstructionFields f; f.opcode = op; f.immediate = imm; return ISA::encode(f);
}
static ISA::InstrWord NOP()  { ISA::InstructionFields f; f.opcode = Op::NOP;  return ISA::encode(f); }
static ISA::InstrWord HALT() { ISA::InstructionFields f; f.opcode = Op::HALT; return ISA::encode(f); }
static const ISA::InstrWord ILLEGAL = 0xFFFFFFFFu;

struct Both {
    Pipeline p;
    ISS      s;
    Both(const Prog& prog, const Pipeline::Config& cfg) : p(cfg), s(cfg.imemBytes, cfg.dmemBytes) {
        p.loadProgram(prog); s.loadProgram(prog);
        p.run(5'000'000); s.run(5'000'000);
    }
};

static void expectSame(const char* name, const Both& b) {
    bool ok = b.p.halted() && b.s.halted() && b.p.fault() == b.s.fault();
    for (unsigned r = 0; r < 15; ++r) ok = ok && (b.p.reg(r) == b.s.reg(r));
    ok = ok && (b.p.dmem() == b.s.dmem());
    ok = ok && (b.p.stats().retired == b.s.retired() + b.p.stats().takenBranches);
    if (!ok) {
        ++fails;
        std::cout << "  FAIL (pipeline != reference): " << name << "\n";
        for (unsigned r = 0; r < 15; ++r)
            if (b.p.reg(r) != b.s.reg(r)) std::cout << "    r" << r << " pipe=" << b.p.reg(r) << " ref=" << b.s.reg(r) << "\n";
        std::cout << "    fault pipe=" << (int)b.p.fault() << " ref=" << (int)b.s.fault()
                  << "  retired pipe=" << b.p.stats().retired << " ref=" << b.s.retired()
                  << " taken=" << b.p.stats().takenBranches << "\n";
    }
}

static Pipeline::Config quick() { Pipeline::Config c; c.longOpCycles = 5; return c; }

int main() {
    {
        Prog p = { RI(Op::ADD, 1, 0, 5), NOP(), RI(Op::ADD, 2, 1, 7), NOP(), RR(Op::SUB, 3, 2, 1), NOP(), HALT() };
        Both b(p, quick());
        expectSame("alu", b);
        CHECK(b.p.reg(2) == 12 && b.p.reg(3) == 7);
        CHECK(b.p.stats().rawHazards == 0);
        CHECK(b.p.stats().cycles == b.p.stats().retired + 3);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 0x100), RI(Op::ADD, 2, 0, 1234), NOP(),
                   ST(2, 1, 8), LD(3, 1, 8), NOP(), RI(Op::ADD, 4, 3, 1), HALT() };
        Both b(p, quick());
        expectSame("load/store", b);
        CHECK(b.p.reg(3) == 1234 && b.p.reg(4) == 1235 && b.p.dmem()[0x108 / 8] == 1234);
        CHECK(b.p.stats().rawHazards == 0);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 5), RI(Op::ADD, 2, 0, 0), NOP(),
                   RR(Op::ADD, 2, 2, 1),
                   RI(Op::SUB, 1, 1, 1), NOP(),
                   BR(Op::JNZ, 1, 0, -3),
                   NOP(),
                   HALT() };
        Both b(p, quick());
        expectSame("loop", b);
        CHECK(b.p.reg(2) == 15 && b.p.reg(1) == 0);
        CHECK(b.p.stats().takenBranches == 4 && b.p.stats().rawHazards == 0);
        CHECK(b.p.stats().cycles == b.p.stats().retired + 3);
    }
    {
        Prog p = { JT(Op::CALL, 4), NOP(), RI(Op::ADD, 6, 6, 1), HALT(),
                   RI(Op::ADD, 1, 0, 42), JT(Op::RET, 0), NOP() };
        Both b(p, quick());
        expectSame("call/ret", b);
        CHECK(b.p.reg(1) == 42 && b.p.reg(6) == 1 && b.p.reg(14) == 4);
        CHECK(b.p.stats().takenBranches == 2);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 3), RI(Op::ADD, 2, 0, 4), NOP(),
                   RI(Op::LSL, 1, 1, 32), RI(Op::LSL, 2, 2, 32), NOP(),
                   RR(Op::MUL, 3, 1, 2), NOP(), RR(Op::DIV, 4, 3, 2), NOP(), HALT() };
        Both b(p, quick());
        expectSame("long ops", b);
        CHECK(b.p.reg(3) == (Word(12) << 32) && b.p.reg(4) == (Word(3) << 32));
        CHECK(b.p.stats().execBusyCycles == 2 * 4);
        CHECK(b.p.stats().cycles == b.p.stats().retired + 3 + b.p.stats().execBusyCycles);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 5), RI(Op::ADD, 2, 1, 1), HALT() };
        Pipeline pl(quick()); pl.loadProgram(p); pl.run(1000);
        ISS s; s.loadProgram(p); s.run(1000);
        CHECK(pl.reg(2) == 1);
        CHECK(s.reg(2) == 6);
        CHECK(pl.stats().rawHazards == 1);
    }
    {
        Prog p = { NOP(), NOP(), RI(Op::ADD, 1, 15, 0), HALT() };
        Both b(p, quick());
        expectSame("r15", b);
        CHECK(b.p.reg(1) == 8);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 1), NOP(), ILLEGAL, RI(Op::ADD, 2, 0, 2), HALT() };
        Both b(p, quick()); expectSame("illegal opcode", b);
        CHECK(b.p.fault() == Pipeline::Fault::IllegalInstruction && b.p.reg(1) == 1 && b.p.reg(2) == 0);
        CHECK(b.p.stats().retired == 2);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 7), NOP(), ST(1, 0, 4), RI(Op::ADD, 2, 0, 9), HALT() };
        Both b(p, quick()); expectSame("misaligned store", b);
        CHECK(b.p.fault() == Pipeline::Fault::MemoryFault && b.p.reg(2) == 0);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 1), NOP(), RI(Op::LSL, 1, 1, 40), NOP(), LD(3, 1, 0), RI(Op::ADD, 2, 0, 9), HALT() };
        Both b(p, quick()); expectSame("load out of range", b);
        CHECK(b.p.fault() == Pipeline::Fault::MemoryFault && b.p.reg(2) == 0 && b.p.reg(3) == 0);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 1), JT(Op::JMP, 70000), NOP() };
        Both b(p, quick()); expectSame("fetch past end", b);
        CHECK(b.p.fault() == Pipeline::Fault::FetchFault);
    }
    {
        Prog p = { RI(Op::ADD, 14, 0, 6), NOP(), JT(Op::RET, 0), NOP() };
        Both b(p, quick()); expectSame("misaligned RET", b);
        CHECK(b.p.fault() == Pipeline::Fault::FetchFault);
    }
    {
        ISA::InstructionFields badLoad; badLoad.opcode = Op::LOAD; badLoad.rd = 1; badLoad.modifier = 1;
        ISA::InstructionFields matAdd;  matAdd.opcode = Op::MATADD; matAdd.matrixRows = 2; matAdd.matrixCols = 2;
        const std::pair<const char*, ISA::InstrWord> illegal[] = {
            {"write to R15",  RI(Op::ADD, 15, 0, 1)},
            {"LOAD modifier", ISA::encode(badLoad)},
            {"matrix opcode", ISA::encode(matAdd)},
            {"EXP",           R1(Op::EXP, 1, 2)},
        };
        for (const auto& [name, word] : illegal) {
            Both b(Prog{ word, HALT() }, quick()); expectSame(name, b);
            CHECK(b.p.fault() == Pipeline::Fault::IllegalInstruction && b.p.stats().retired == 0);
        }
    }
    {
        std::mt19937 rng(12345);
        auto pick = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
        const Op binary[] = { Op::ADD, Op::SUB, Op::AND, Op::OR, Op::XOR, Op::LSL, Op::LSR, Op::ASR,
                              Op::CMP, Op::MUL, Op::DIV, Op::MOD };
        const Op unary[]  = { Op::NOT, Op::SIN, Op::COS, Op::LOG };
        int programs = 0;
        for (int n = 0; n < 150; ++n) {
            Prog p;
            for (int r = 1; r <= 8; ++r) { p.push_back(RI(Op::ADD, r, 0, pick(-60000, 60000))); p.push_back(NOP()); }
            for (int i = 0; i < 40; ++i) {
                const int rd = pick(1, 8), rs1 = pick(0, 8), rs2 = pick(0, 8);
                switch (pick(0, 5)) {
                    case 0: p.push_back(R1(unary[pick(0, 3)], rd, rs1)); break;
                    case 1: p.push_back(LD(rd, 0, 8 * pick(0, 255))); break;
                    case 2: p.push_back(ST(rd, 0, 8 * pick(0, 255))); break;
                    case 3: p.push_back(RI(binary[pick(0, 11)], rd, rs1, pick(-1000, 1000))); break;
                    default: p.push_back(RR(binary[pick(0, 11)], rd, rs1, rs2)); break;
                }
                p.push_back(NOP());
            }
            p.push_back(HALT());
            Pipeline::Config c; c.longOpCycles = 1 + pick(0, 4);
            Both b(p, c);
            expectSame("random", b);
            CHECK(b.p.stats().rawHazards == 0);
            CHECK(b.p.stats().cycles == b.p.stats().retired + 3 + b.p.stats().execBusyCycles);
            ++programs;
        }
        std::cout << "random programs checked: " << programs << "\n";
    }

    std::cout << (fails ? "FAILURES: " + std::to_string(fails) : std::string("all pipeline checks passed")) << "\n";
    return fails != 0;
}
