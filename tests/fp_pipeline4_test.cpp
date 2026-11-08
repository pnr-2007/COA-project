#include "pipeline4.cpp"
using Pipeline = Pipeline4;
#include "iss.cpp"
#include <cstring>
#include <iostream>
#include <random>

using Op   = ISA::Opcode;
using Word = ISA::Word;
using Prog = std::vector<ISA::InstrWord>;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { ++fails; std::cout << "  FAIL line " << __LINE__ << ": " #c "\n"; } } while (0)

static uint64_t bitsOf(double d) { uint64_t u; std::memcpy(&u, &d, 8); return u; }

static ISA::InstrWord RR(Op op, int rd, int rs1, int rs2) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; f.rs2 = rs2; return ISA::encode(f);
}
static ISA::InstrWord RI(Op op, int rd, int rs1, int32_t imm) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; f.modifier = 1; f.immediate = imm; return ISA::encode(f);
}
static ISA::InstrWord UN(Op op, int rd, int rs1) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = rs1; return ISA::encode(f);
}
static ISA::InstrWord MEM(Op op, int rd, int base, int32_t off) {
    ISA::InstructionFields f; f.opcode = op; f.rd = rd; f.rs1 = base; f.immediate = off; return ISA::encode(f);
}
static ISA::InstrWord BR(Op op, int rs1, int rs2, int32_t imm) {
    ISA::InstructionFields f; f.opcode = op; f.rs1 = rs1; f.rs2 = rs2; f.immediate = imm; return ISA::encode(f);
}
static ISA::InstrWord HALT() { ISA::InstructionFields f; f.opcode = Op::HALT; return ISA::encode(f); }
static ISA::InstrWord RDSR(int rd, int sel) {
    ISA::InstructionFields f; f.opcode = Op::RDSR; f.rd = rd; f.immediate = sel; return ISA::encode(f);
}
static ISA::InstrWord WRSR(int sel, int rs1) {
    ISA::InstructionFields f; f.opcode = Op::WRSR; f.rs1 = rs1; f.immediate = sel; return ISA::encode(f);
}

static constexpr int FCSR = ExceptionHandler::FCSR;

static Pipeline::Config quick() {
    Pipeline::Config c; c.memory.timing = false;
    c.longOpCycles = 5;
    return c;
}

static bool same(const char* name, const Pipeline& p, const ISS& s) {
    bool ok = p.halted() && s.halted() && p.fault() == s.fault() && p.stats().retired == s.retired() && p.dmem() == s.dmem();
    for (unsigned r = 0; r < 15; ++r) ok = ok && p.reg(r) == s.reg(r);
    for (unsigned r = 0; r < 16; ++r) ok = ok && p.freg(r) == s.freg(r);
    for (unsigned i = 0; i < InterruptHandler::SR_COUNT; ++i)
        if (i != InterruptHandler::IPEND) ok = ok && p.specialRegs().read(i) == s.specialRegs().read(i);
    if (!ok) {
        ++fails;
        std::cout << "  FAIL (pipeline != reference): " << name << "  retired " << p.stats().retired << "/" << s.retired()
                  << " fault " << int(p.fault()) << "/" << int(s.fault()) << "\n";
        for (unsigned r = 0; r < 15; ++r)
            if (p.reg(r) != s.reg(r)) std::cout << "    r" << r << " pipe=" << p.reg(r) << " ref=" << s.reg(r) << "\n";
        for (unsigned r = 0; r < 16; ++r)
            if (p.freg(r) != s.freg(r)) std::cout << "    f" << r << " pipe=" << std::hex << p.freg(r) << " ref=" << s.freg(r) << std::dec << "\n";
        for (unsigned i = 0; i < InterruptHandler::SR_COUNT; ++i)
            if (i != InterruptHandler::IPEND && p.specialRegs().read(i) != s.specialRegs().read(i))
                std::cout << "    sr" << i << " pipe=" << p.specialRegs().read(i) << " ref=" << s.specialRegs().read(i) << "\n";
    }
    return ok;
}

struct Both {
    Pipeline p;
    ISS s;
    Both(const Prog& prog, const std::vector<Word>& data, const Pipeline::Config& cfg = quick())
        : p(cfg), s(cfg.imemBytes, cfg.dmemBytes) {
        p.loadProgram(prog); s.loadProgram(prog);
        for (size_t i = 0; i < data.size(); ++i) { p.pokeDmem(i, data[i]); s.pokeDmem(i, data[i]); }
        p.run(5'000'000); s.run(5'000'000);
    }
};

static std::mt19937_64 rng(777);

static uint64_t interesting() {
    const unsigned cat = rng() % 8;
    const uint64_t sign = (rng() & 1) << 63;
    static const uint64_t exps[] = {0, 1, 0x3FD, 0x3FE, 0x3FF, 0x400, 0x401, 0x432, 0x433, 0x7FD, 0x7FE, 0x7FF, 0x3CA, 0x3CB, 0x41D, 0x41E};
    static const uint64_t fracs[] = {0, 1, (1ull << 52) - 1, 1ull << 51, 0x5555555555555ull};
    switch (cat) {
        case 0: return rng();
        case 1: return sign | (exps[rng() % 16] << 52) | (rng() & ((1ull << 52) - 1));
        case 2: return sign | (exps[rng() % 16] << 52) | fracs[rng() % 5];
        case 3: return sign | ((0x3F0 + rng() % 0x30) << 52) | (rng() & ((1ull << 52) - 1));
        case 4: return sign | ((0x3E0 + rng() % 0x60) << 52) | (rng() & ((1ull << 52) - 1));
        default: return sign | ((rng() % 2046 + 1) << 52) | (rng() & ((1ull << 52) - 1));
    }
}

static Prog randomFpProgram(unsigned n, bool allowBadRm) {
    Prog p;
    auto ir = [&] { return int(rng() % 15); };
    auto fr = [&] { return int(rng() % 16); };
    auto off = [&] { return int(8 * (rng() % 64)); };
    for (unsigned i = 0; i < n; ++i) {
        const unsigned k = rng() % 100;
        if (k < 26) {
            static const Op ops[] = {Op::FADD, Op::FSUB, Op::FMUL, Op::FDIV};
            p.push_back(RR(ops[rng() % 4], fr(), fr(), fr()));
        } else if (k < 31) p.push_back(UN(Op::FSQRT, fr(), fr()));
        else if (k < 36) p.push_back(RR(Op::FCMP, ir(), fr(), fr()));
        else if (k < 41) p.push_back(UN(Op::FCVTDQ, fr(), ir()));
        else if (k < 46) p.push_back(UN(Op::FCVTQD, ir(), fr()));
        else if (k < 50) p.push_back(UN(Op::FMVDX, fr(), ir()));
        else if (k < 54) p.push_back(UN(Op::FMVXD, ir(), fr()));
        else if (k < 66) p.push_back(MEM(Op::FLOAD, fr(), 0, off()));
        else if (k < 72) p.push_back(MEM(Op::FSTORE, fr(), 0, off()));
        else if (k < 78) p.push_back(MEM(Op::LOAD, ir(), 0, off()));
        else if (k < 82) p.push_back(MEM(Op::STORE, ir(), 0, off()));
        else if (k < 87) {
            static const Op ops[] = {Op::ADD, Op::SUB, Op::XOR, Op::OR, Op::AND};
            p.push_back(rng() % 2 ? RR(ops[rng() % 5], ir() + 1 > 14 ? 1 : ir() + 1, ir(), ir()) : RI(ops[rng() % 5], ir() + 1 > 14 ? 2 : ir() + 1, ir(), int(rng() % 200) - 100));
        } else if (k < 90) {
            unsigned rmv = rng() % 5;
            if (allowBadRm && rng() % 4 == 0) rmv = 5 + rng() % 3;
            const int tmp = 14;
            p.push_back(RI(Op::ADD, tmp, 0, int(rmv << 5)));
            p.push_back(WRSR(FCSR, tmp));
        } else if (k < 93) p.push_back(RDSR(ir() + 1 > 14 ? 3 : ir() + 1, FCSR));
        else if (k < 96 && i + 4 < n) p.push_back(BR(rng() % 2 ? Op::JZ : Op::JNZ, ir(), 0, 2 + int(rng() % 2)));
        else p.push_back(RR(Op::FADD, fr(), fr(), fr()));
    }
    p.push_back(HALT());
    return p;
}

static std::vector<Word> randomData() {
    std::vector<Word> d(64);
    for (auto& w : d) w = interesting();
    return d;
}

int main() {
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), MEM(Op::FLOAD, 2, 0, 8), RR(Op::FADD, 3, 1, 2), RR(Op::FMUL, 4, 3, 3),
                   MEM(Op::FSTORE, 4, 0, 16), HALT() };
        Both b(p, {bitsOf(1.5), bitsOf(2.25)});
        same("basic", b.p, b.s);
        CHECK(b.p.dmem()[2] == bitsOf(3.75 * 3.75));
        CHECK(b.p.freg(3) == bitsOf(3.75));
        CHECK(b.p.stats().fpOps == 2);
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), MEM(Op::FLOAD, 2, 0, 8), RR(Op::FDIV, 3, 1, 2), RDSR(1, FCSR), HALT() };
        Both b(p, {bitsOf(1.0), bitsOf(0.0)});
        same("divzero flag", b.p, b.s);
        CHECK(b.p.reg(1) == FPU::DZ);
        CHECK(b.p.freg(3) == bitsOf(1.0 / 0.0));
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), MEM(Op::FLOAD, 2, 0, 8), RR(Op::FADD, 3, 1, 2), RDSR(1, FCSR), HALT() };
        Both b(p, {bitsOf(1.0), bitsOf(std::ldexp(1.0, -60))});
        same("inexact flag rne", b.p, b.s);
        CHECK(b.p.reg(1) == FPU::NX && b.p.freg(3) == bitsOf(1.0));
    }
    {
        Prog p = { RI(Op::ADD, 5, 0, 3 << 5), WRSR(FCSR, 5), MEM(Op::FLOAD, 1, 0, 0), MEM(Op::FLOAD, 2, 0, 8),
                   RR(Op::FADD, 3, 1, 2), RDSR(1, FCSR), HALT() };
        Both b(p, {bitsOf(1.0), bitsOf(std::ldexp(1.0, -60))});
        same("round up mode", b.p, b.s);
        CHECK(b.p.freg(3) == bitsOf(1.0) + 1 && b.p.reg(1) == ((3u << 5) | FPU::NX));
    }
    {
        Prog p = { RI(Op::ADD, 5, 0, 5 << 5), WRSR(FCSR, 5), MEM(Op::FLOAD, 1, 0, 0), RR(Op::FADD, 3, 1, 1), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("illegal rounding mode", b.p, b.s);
        CHECK(b.p.fault() == Exec::Fault::IllegalInstruction && b.p.freg(3) == 0);
    }
    {
        Prog p = { RI(Op::ADD, 5, 0, 5 << 5), WRSR(FCSR, 5), MEM(Op::FLOAD, 1, 0, 0), RR(Op::FCMP, 2, 1, 1), UN(Op::FMVXD, 3, 1), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("moves and compare ignore rounding mode", b.p, b.s);
        CHECK(b.p.fault() == Exec::Fault::None && b.p.reg(2) == 0 && b.p.reg(3) == bitsOf(1.0));
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), RR(Op::FADD, 2, 1, 1), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("fp load-use", b.p, b.s);
        CHECK(b.p.interlockStats().loadUse == 1 && b.p.freg(2) == bitsOf(2.0));
    }
    {
        Prog p = { MEM(Op::LOAD, 1, 0, 0), RR(Op::FADD, 2, 1, 1), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("int load does not stall fp reader", b.p, b.s);
        CHECK(b.p.interlockStats().loadUse == 0);
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), RR(Op::ADD, 2, 1, 1), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("fp load does not stall int reader", b.p, b.s);
        CHECK(b.p.interlockStats().loadUse == 0 && b.p.reg(2) == 0);
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), RI(Op::ADD, 1, 0, 7), RR(Op::FADD, 2, 1, 1), UN(Op::FMVXD, 3, 2), HALT() };
        Both b(p, {bitsOf(1.0)});
        same("fp and int regs with same index do not forward", b.p, b.s);
        CHECK(b.p.reg(3) == bitsOf(2.0) && b.p.reg(1) == 7);
    }
    {
        Prog p = { MEM(Op::FLOAD, 0, 0, 0), RR(Op::FADD, 0, 0, 0), UN(Op::FMVXD, 1, 0), HALT() };
        Both b(p, {bitsOf(1.25)});
        same("f0 is a normal register", b.p, b.s);
        CHECK(b.p.reg(1) == bitsOf(2.5));
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 3), UN(Op::FCVTDQ, 1, 1), UN(Op::FCVTQD, 2, 1), HALT() };
        Both b(p, {});
        same("convert int to fp and back", b.p, b.s);
        CHECK(b.p.freg(1) == bitsOf(3.0 / 4294967296.0) && b.p.reg(2) == 3);
    }
    {
        Prog p = { MEM(Op::FLOAD, 1, 0, 0), RR(Op::FDIV, 2, 1, 1), RR(Op::FSQRT, 3, 2, 2), HALT() };
        p[2] = UN(Op::FSQRT, 3, 2);
        Pipeline::Config slow = quick(), fast = quick();
        slow.fp.div = 40; fast.fp.div = 2; fast.fp.sqrt = 2;
        Both a(p, {bitsOf(9.0)}, slow), b(p, {bitsOf(9.0)}, fast);
        same("fp latency slow", a.p, a.s);
        same("fp latency fast", b.p, b.s);
        CHECK(a.p.stats().cycles > b.p.stats().cycles + 30);
    }
    {
        int bad = 0;
        uint64_t fp = 0, fwd = 0, lu = 0;
        for (int t = 0; t < 600; ++t) {
            const Prog p = randomFpProgram(70, false);
            Pipeline::Config c = quick();
            c.fp.add = 1 + rng() % 6; c.fp.mul = 1 + rng() % 8; c.fp.div = 1 + rng() % 30; c.fp.sqrt = 1 + rng() % 30;
            c.fp.convert = 1 + rng() % 5; c.fp.compare = 1 + rng() % 3; c.fp.move = 1 + rng() % 2;
            Both b(p, randomData(), c);
            if (!same("random fp", b.p, b.s)) ++bad;
            fp += b.p.stats().fpOps;
            fwd += b.p.forwardStats().fromExMa;
            lu += b.p.interlockStats().loadUse;
            if (bad > 3) break;
        }
        CHECK(fp > 5000 && fwd > 500 && lu > 50);
        std::cout << "random fp programs checked: 600, fp ops " << fp << ", forwards " << fwd << ", load-use stalls " << lu << "\n";
    }
    {
        int bad = 0, faults = 0;
        for (int t = 0; t < 300; ++t) {
            const Prog p = randomFpProgram(70, true);
            Both b(p, randomData());
            if (!same("random fp with bad rounding", b.p, b.s)) ++bad;
            if (b.p.fault() != Exec::Fault::None) ++faults;
            if (bad > 3) break;
        }
        CHECK(faults > 10);
        std::cout << "random fp programs with illegal rounding modes: 300, faults " << faults << "\n";
    }
    {
        int bad = 0;
        for (int t = 0; t < 80; ++t) {
            const Prog p = randomFpProgram(70, false);
            Pipeline::Config c;
            Both b(p, randomData(), c);
            if (!same("random fp with cache timing", b.p, b.s)) ++bad;
            if (bad > 3) break;
        }
        std::cout << "random fp programs with cache timing: 80\n";
    }
    std::cout << (fails ? "FP PIPELINE CHECKS FAILED\n" : "all fp pipeline checks passed\n");
    return fails ? 1 : 0;
}
