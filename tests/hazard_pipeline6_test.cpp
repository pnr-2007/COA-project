#include "pipeline6.cpp"
using Pipeline = Pipeline6;
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
static ISA::InstrWord RETI() { ISA::InstructionFields f; f.opcode = Op::RETI; return ISA::encode(f); }
static ISA::InstrWord RDSR(int rd, int sel) {
    ISA::InstructionFields f; f.opcode = Op::RDSR; f.rd = rd; f.immediate = sel; return ISA::encode(f);
}
static ISA::InstrWord WRSR(int sel, int rs1) {
    ISA::InstructionFields f; f.opcode = Op::WRSR; f.rs1 = rs1; f.immediate = sel; return ISA::encode(f);
}
static const ISA::InstrWord ILLEGAL = 0xFFFFFFFFu;

using SRn = InterruptHandler;

static Pipeline::Config quick() {
    Pipeline::Config c; c.memory.timing = false;
    c.longOpCycles = 5;
    return c;
}

static bool same(const char* name, const Pipeline& p, const ISS& s) {
    bool ok = p.halted() && s.halted() && p.fault() == s.fault() && p.stats().retired == s.retired() && p.dmem() == s.dmem();
    for (unsigned r = 0; r < 15; ++r) ok = ok && p.reg(r) == s.reg(r);
    for (unsigned i = 0; i < InterruptHandler::SR_COUNT; ++i)
        if (i != InterruptHandler::IPEND) ok = ok && p.specialRegs().read(i) == s.specialRegs().read(i);
    if (!ok) {
        ++fails;
        std::cout << "  FAIL (pipeline != reference): " << name << "  retired " << p.stats().retired << "/" << s.retired()
                  << " fault " << int(p.fault()) << "/" << int(s.fault()) << "\n";
        for (unsigned r = 0; r < 15; ++r)
            if (p.reg(r) != s.reg(r)) std::cout << "    r" << r << " pipe=" << p.reg(r) << " ref=" << s.reg(r) << "\n";
    }
    return ok;
}

using Segs = std::vector<std::pair<uint64_t, Prog>>;
static const uint64_t TBL = 1024, BODY = 2048;
static Segs vec(unsigned index, const Prog& body, uint64_t bodyAddr) {
    const uint64_t entry = TBL + 16 * index;
    return { {entry, {JT(Op::JMP, int32_t((int64_t(bodyAddr) - int64_t(entry)) / 4))}}, {bodyAddr, body} };
}

struct Both {
    Pipeline p;
    ISS s;
    Both(const Prog& prog, const Pipeline::Config& cfg = quick(), const Segs& segs = Segs()) : p(cfg), s(cfg.imemBytes, cfg.dmemBytes) {
        p.loadProgram(prog); s.loadProgram(prog);
        for (const auto& [addr, body] : segs) { p.loadProgram(body, addr); s.loadProgram(body, addr); }
        p.run(5'000'000); s.run(5'000'000);
    }
};

static Prog randomProgram(std::mt19937& g, int n) {
    Prog p;
    auto reg = [&](int lo) { return int(g() % (10 - lo)) + lo; };
    for (int i = 0; i < n; ++i) {
        switch (g() % 9) {
            case 0: p.push_back(RR(Op::ADD, reg(1), reg(0), reg(0))); break;
            case 1: p.push_back(RR(Op::SUB, reg(1), reg(0), reg(0))); break;
            case 2: p.push_back(RI(Op::ADD, reg(1), reg(0), int(g() % 2001) - 1000)); break;
            case 3: p.push_back(RR(Op::XOR, reg(1), reg(0), reg(0))); break;
            case 4: p.push_back(RR(Op::MUL, reg(1), reg(0), reg(0))); break;
            case 5: p.push_back(ST(reg(0), 0, 8 * int(g() % 8))); break;
            case 6: p.push_back(LD(reg(1), 0, 8 * int(g() % 8))); break;
            case 7: {
                const Op ops[] = {Op::JEQ, Op::JGT, Op::JNZ, Op::JZ};
                p.push_back(BR(ops[g() % 4], reg(0), reg(0), 2 + int(g() % 3)));
                break;
            }
            default: p.push_back(RR(Op::CMP, reg(1), reg(0), reg(0))); break;
        }
    }
    for (int i = 0; i < 6; ++i) p.push_back(HALT());
    return p;
}

int main() {
    {
        Prog p = { RI(Op::ADD, 1, 0, 5), RI(Op::ADD, 2, 1, 1), RR(Op::ADD, 3, 2, 1), RR(Op::SUB, 4, 3, 2), HALT() };
        Both b(p);
        same("forward chain", b.p, b.s);
        CHECK(b.p.reg(4) == 5 && b.p.reg(3) == 11);
        CHECK(b.p.forwardStats().fromMem1 >= 3);
        CHECK(b.p.stats().cycles == b.p.stats().retired + 5);
        CHECK(b.p.interlockStats().loadUse == 0);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 77), ST(1, 0, 16), LD(2, 0, 16), RR(Op::ADD, 3, 2, 2), HALT() };
        Both b(p);
        same("load-use", b.p, b.s);
        CHECK(b.p.reg(3) == 154);
        CHECK(b.p.interlockStats().loadUse == 2);
        CHECK(b.p.stats().cycles == b.p.stats().retired + 5 + 2);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 3), LD(2, 0, 0), RI(Op::ADD, 4, 0, 9), RR(Op::ADD, 3, 2, 1), HALT() };
        Both b(p);
        same("load then independent then use", b.p, b.s);
        CHECK(b.p.interlockStats().loadUse == 1);
    }
    {
        Prog p = { JT(Op::JMP, 2), RI(Op::ADD, 1, 0, 1), RI(Op::ADD, 2, 0, 2), HALT() };
        Both b(p);
        same("no delay slot", b.p, b.s);
        CHECK(b.p.reg(1) == 0 && b.p.reg(2) == 2);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 1), BR(Op::JNZ, 1, 0, 2), RI(Op::ADD, 2, 0, 1), RI(Op::ADD, 3, 0, 1), HALT() };
        Both b(p);
        same("taken branch flush", b.p, b.s);
        CHECK(b.p.reg(2) == 0 && b.p.reg(3) == 1);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 400), RI(Op::SUB, 1, 1, 1), BR(Op::JNZ, 1, 0, -1), HALT() };
        Pipeline::Config off = quick();
        off.predictor.enabled = false;
        Both rl(p), st(p, off);
        same("loop rl", rl.p, rl.s);
        same("loop static", st.p, st.s);
        CHECK(rl.p.stats().mispredicts <= 6);
        CHECK(st.p.stats().mispredicts >= 399);
        CHECK(rl.p.stats().cycles < st.p.stats().cycles);
        CHECK(rl.p.predictorStats().accuracy() > 0.98);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 600), RI(Op::XOR, 2, 2, 1), BR(Op::JZ, 2, 0, 2), RI(Op::ADD, 3, 3, 1),
                   RI(Op::SUB, 1, 1, 1), BR(Op::JNZ, 1, 0, -4), HALT() };
        Pipeline::Config off = quick();
        off.predictor.enabled = false;
        Both rl(p), st(p, off);
        same("alternating rl", rl.p, rl.s);
        CHECK(rl.p.predictorStats().accuracy() > 0.9);
        CHECK(rl.p.stats().mispredicts < st.p.stats().mispredicts / 2);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 100), JT(Op::CALL, 4), RI(Op::SUB, 1, 1, 1), BR(Op::JNZ, 1, 0, -2), HALT(),
                   RI(Op::ADD, 5, 5, 3), JT(Op::RET, 0) };
        Both b(p);
        same("call/ret ras", b.p, b.s);
        CHECK(b.p.reg(5) == 300);
        CHECK(b.p.stats().mispredicts <= 8);
    }
    {
        std::mt19937 g(1234);
        int bad = 0;
        uint64_t fwd = 0, lu = 0;
        for (int t = 0; t < 400; ++t) {
            Both b(randomProgram(g, 60));
            if (!same("random no-nop", b.p, b.s)) ++bad;
            CHECK(b.p.forwardStats().blocked == 0);
            fwd += b.p.forwardStats().fromMem1;
            lu += b.p.interlockStats().loadUse;
            if (bad > 3) break;
        }
        CHECK(fwd > 1000 && lu > 10);
        std::cout << "random no-NOP programs checked: 400, forwards " << fwd << ", load-use stalls " << lu << "\n";
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), ILLEGAL, RI(Op::ADD, 2, 0, 7), HALT() };
        Segs sg = vec(1, { RDSR(3, SRn::CAUSE), RDSR(4, SRn::EPC), RDSR(5, SRn::BADADDR), RI(Op::ADD, 4, 4, 4),
                           WRSR(SRn::EPC, 4), RETI() }, BODY);
        Both b(p, quick(), sg);
        same("illegal handler", b.p, b.s);
        CHECK(b.p.reg(3) == 1 && b.p.reg(4) == 12 && b.p.reg(2) == 7 && b.p.reg(5) == 0xFFFFFFFFull);
        CHECK(b.p.stats().exceptionsTaken == 1);
        CHECK(b.p.fault() == Exec::Fault::None);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), RI(Op::ADD, 2, 0, 3), LD(3, 2, 0), RI(Op::ADD, 4, 0, 5), HALT() };
        Segs sg = vec(2, { RDSR(5, SRn::CAUSE), RDSR(6, SRn::BADADDR), RDSR(7, SRn::EPC), RI(Op::ADD, 7, 7, 4),
                           WRSR(SRn::EPC, 7), RETI() }, BODY);
        Both b(p, quick(), sg);
        same("memory fault handler", b.p, b.s);
        CHECK(b.p.reg(5) == 2 && b.p.reg(6) == 3 && b.p.reg(4) == 5 && b.p.reg(3) == 0);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), RI(Op::ADD, 14, 0, 2), JT(Op::RET, 0), HALT() };
        Segs sg = vec(3, { RDSR(3, SRn::CAUSE), RDSR(4, SRn::BADADDR), RI(Op::ADD, 6, 0, 16), WRSR(SRn::EPC, 6), RETI() }, BODY);
        Both b(p, quick(), sg);
        same("fetch fault handler", b.p, b.s);
        CHECK(b.p.reg(3) == 3 && b.p.reg(4) == 2 && b.p.fault() == Exec::Fault::None);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), ILLEGAL, HALT() };
        Segs sg = vec(1, { ILLEGAL }, BODY);
        Both b(p, quick(), sg);
        same("double fault", b.p, b.s);
        CHECK(b.p.fault() == Exec::Fault::IllegalInstruction);
    }
    {
        Prog p = { RETI(), HALT() };
        Both b(p);
        same("reti outside trap", b.p, b.s);
        CHECK(b.p.fault() == Exec::Fault::IllegalInstruction);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, 5), WRSR(SRn::STATUS, 1), RDSR(2, SRn::STATUS), RDSR(3, 9), HALT() };
        Both b(p);
        same("bad sr select", b.p, b.s);
        CHECK(b.p.fault() == Exec::Fault::IllegalInstruction);
        CHECK(b.p.reg(2) == 5);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), RI(Op::ADD, 2, 0, 1), WRSR(SRn::IMASK, 2),
                   RI(Op::ADD, 2, 0, 4), WRSR(SRn::STATUS, 2), RI(Op::ADD, 5, 0, 0), RI(Op::ADD, 3, 0, 300),
                   RI(Op::ADD, 5, 5, 1), RR(Op::MUL, 8, 5, 5), ST(5, 0, 8), LD(9, 0, 8), RR(Op::ADD, 10, 10, 9),
                   RR(Op::SUB, 4, 3, 5), BR(Op::JNZ, 4, 0, -6), HALT() };
        Segs sg = vec(4, { RI(Op::ADD, 6, 6, 1), RETI() }, BODY);
        std::mt19937 g(99);
        int bad = 0;
        for (int t = 0; t < 40; ++t) {
            Pipeline pl(quick());
            pl.loadProgram(p);
            for (auto& [addr, body] : sg) pl.loadProgram(body, addr);
            std::vector<uint64_t> events;
            const int n = 1 + g() % 4;
            std::vector<uint64_t> at;
            for (int i = 0; i < n; ++i) at.push_back(5 + g() % 1500);
            uint64_t taken = 0;
            while (!pl.halted() && pl.stats().cycles < 200000) {
                for (uint64_t c : at) if (pl.stats().cycles == c) pl.raiseInterrupt(0);
                pl.step();
                if (pl.stats().interruptsTaken != taken) {
                    taken = pl.stats().interruptsTaken;
                    events.push_back(pl.stats().lastInterruptRetired);
                }
            }
            ISS s(quick().imemBytes, quick().dmemBytes);
            s.loadProgram(p);
            for (auto& [addr, body] : sg) s.loadProgram(body, addr);
            for (uint64_t e : events) s.scheduleInterrupt(e, 0);
            s.run(5'000'000);
            if (!same("interrupt random", pl, s)) ++bad;
            CHECK(pl.reg(6) == Word(events.size()));
            if (bad > 3) break;
        }
        std::cout << "interrupt scenarios checked: 40\n";
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), RI(Op::ADD, 2, 0, 2), WRSR(SRn::IMASK, 2),
                   RI(Op::ADD, 2, 0, 4), WRSR(SRn::STATUS, 2), RI(Op::ADD, 3, 0, 200), RI(Op::SUB, 3, 3, 1),
                   BR(Op::JNZ, 3, 0, -1), HALT() };
        Segs sg = vec(5, { RDSR(7, SRn::CAUSE), RI(Op::ADD, 6, 6, 11), RETI() }, BODY);
        Pipeline pl(quick());
        pl.loadProgram(p);
        for (auto& [addr, body] : sg) pl.loadProgram(body, addr);
        for (int i = 0; i < 60; ++i) pl.step();
        pl.raiseInterrupt(1);
        pl.run(100000);
        CHECK(pl.halted() && pl.stats().interruptsTaken == 1 && pl.reg(7) == 17 && pl.reg(6) == 11);
    }
    {
        Prog p = { RI(Op::ADD, 1, 0, TBL), WRSR(SRn::TVEC, 1), RI(Op::ADD, 2, 0, 1), WRSR(SRn::IMASK, 2),
                   RI(Op::ADD, 2, 0, 4), WRSR(SRn::STATUS, 2), RI(Op::ADD, 3, 0, 200), RI(Op::SUB, 3, 3, 1),
                   BR(Op::JNZ, 3, 0, -1), HALT() };
        Segs sg = vec(4, { RI(Op::ADD, 6, 6, 1), RETI() }, BODY);
        Pipeline pl(quick());
        pl.loadProgram(p);
        for (auto& [addr, body] : sg) pl.loadProgram(body, addr);
        pl.raiseInterrupt(3);
        pl.run(100000);
        CHECK(pl.halted() && pl.stats().interruptsTaken == 0 && pl.reg(6) == 0);
    }
    if (fails == 0) std::cout << "all hazard checks passed\n";
    else std::cout << "FAILURES: " << fails << "\n";
    return fails != 0;
}
