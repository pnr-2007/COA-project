#include "control_unit.cpp"
#include <iostream>
#include <random>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { ++fails; std::cout << "  FAIL line " << __LINE__ << ": " #c "\n"; } } while (0)

using Op = ISA::Opcode;

int main() {
    uint64_t combos = 0;
    const uint8_t regs[] = {0, 3, 13, 14, 15};
    const int32_t imms[] = {-5, 0, 1, 5, 9};
    for (uint8_t o = 0; o < ISA::opcodeValue(Op::OPCODE_COUNT); ++o)
        for (uint8_t rd : regs) for (uint8_t rs1 : regs) for (uint8_t rs2 : regs)
        for (uint8_t mod : {0, 1, 2}) for (int32_t imm : imms)
        {
            ISA::InstructionFields f;
            f.opcode = static_cast<Op>(o); f.rd = rd; f.rs1 = rs1; f.rs2 = rs2; f.modifier = mod; f.immediate = imm;
            const ControlWord w = ControlUnit::decode(f);
            const Exec::RegUse u = Exec::regUse(f);
            ++combos;
            bool ok = w.readsA == u.readsA && w.readsB == u.readsB && w.regWrite == u.writes;
            ok = ok && (!u.readsA || w.srcA == u.a) && (!u.readsB || w.srcB == u.b);
            ok = ok && (!u.writes || w.dest == u.dest);
            ok = ok && (!u.readsA || w.srcAFp == u.aFp) && (!u.readsB || w.srcBFp == u.bFp) && (!u.writes || w.destFp == u.destFp);
            ok = ok && w.fpOp == (f.type() == ISA::InstructionType::FP);
            ok = ok && ControlUnit::validate(f) == Exec::validate(f);
            ok = ok && (f.type() != ISA::InstructionType::ALU || w.longOp == Exec::isLongOp(f.opcode));
            ok = ok && w.serializing == Exec::isSerializing(f.opcode);
            if (!ok) { CHECK(ok); if (fails > 5) return 1; }
        }
    std::cout << "decode combinations checked against Exec: " << combos << "\n";

    auto mk = [](Op op) { ISA::InstructionFields f; f.opcode = op; return f; };
    {
        ISA::InstructionFields f = mk(Op::LOAD); f.rd = 4; f.rs1 = 2;
        const ControlWord w = ControlUnit::decode(f);
        CHECK(w.memRead && !w.memWrite && w.regWrite && w.dest == 4 && w.result == ResultSel::LOAD && w.readsA && !w.readsB);
        f = mk(Op::STORE); f.rd = 4; f.rs1 = 2;
        const ControlWord s = ControlUnit::decode(f);
        CHECK(s.memWrite && !s.regWrite && s.readsB && s.srcB == 4 && s.readsA && s.srcA == 2);
        f = mk(Op::CALL);
        const ControlWord c = ControlUnit::decode(f);
        CHECK(c.branch == BranchKind::CALL && c.regWrite && c.dest == ISA::LR && c.result == ResultSel::LINK && !c.conditionalBranch);
        f = mk(Op::RET);
        const ControlWord r = ControlUnit::decode(f);
        CHECK(r.branch == BranchKind::RET && r.readsA && r.srcA == ISA::LR && !r.regWrite);
        f = mk(Op::JNZ); f.rs1 = 3;
        CHECK(ControlUnit::decode(f).conditionalBranch);
        f = mk(Op::MUL); f.rd = 1; f.rs1 = 2; f.rs2 = 3;
        const ControlWord m = ControlUnit::decode(f);
        CHECK(m.alu == AluOp::MUL && m.longOp && m.readsB && !m.immB);
        f.modifier = 1; f.immediate = 7;
        const ControlWord mi = ControlUnit::decode(f);
        CHECK(mi.immB && !mi.readsB && mi.immediate == 7);
        f = mk(Op::HALT);
        CHECK(ControlUnit::decode(f).halt);
        f = mk(Op::RETI);
        const ControlWord t = ControlUnit::decode(f);
        CHECK(t.serializing && t.branch == BranchKind::RETI);
    }
    {
        std::mt19937 g(3);
        for (int i = 0; i < 200000; ++i) {
            const ISA::InstrWord word = static_cast<ISA::InstrWord>(g());
            const ControlWord w = ControlUnit::decodeWord(word, 0x40, false);
            ISA::InstructionFields f;
            Exec::Fault expect = Exec::Fault::None;
            try { f = ISA::decode(word); expect = Exec::validate(f); }
            catch (const std::invalid_argument&) { expect = Exec::Fault::IllegalInstruction; }
            CHECK(w.fault == expect);
            if (expect != Exec::Fault::None) CHECK(w.faultValue == word && !w.regWrite && !w.memRead && !w.memWrite);
            if (fails > 5) return 1;
        }
        const ControlWord ff = ControlUnit::decodeWord(0, 0x1234, true);
        CHECK(ff.fault == Exec::Fault::FetchFault && ff.faultValue == 0x1234);
        std::cout << "random words checked: 200000\n";
    }
    {
        HazardInputs in;
        CHECK(ControlUnit::resolve(in).reason == PipelineControl::Reason::None);
        in.loadUse = in.redirect = in.squash = in.memFreeze = in.interrupt = in.exception = true;
        PipelineControl c = ControlUnit::resolve(in);
        CHECK(c.reason == PipelineControl::Reason::Exception && c.takeTrap && c.flushAll);
        in.exception = false;
        c = ControlUnit::resolve(in); CHECK(c.reason == PipelineControl::Reason::Interrupt && c.takeTrap);
        in.interrupt = false;
        c = ControlUnit::resolve(in); CHECK(c.reason == PipelineControl::Reason::MemFreeze && c.freezeAll && !c.takeTrap);
        in.memFreeze = false;
        c = ControlUnit::resolve(in); CHECK(c.reason == PipelineControl::Reason::Squash && c.squashYounger);
        in.squash = false;
        c = ControlUnit::resolve(in); CHECK(c.reason == PipelineControl::Reason::Redirect && c.redirect && !c.holdFetch);
        in.redirect = false;
        c = ControlUnit::resolve(in); CHECK(c.reason == PipelineControl::Reason::LoadUse && c.holdFetch);
    }
    {
        ISA::InstructionFields f = mk(Op::LOAD); f.rd = 5;
        const ExMaControl e = ControlUnit::toExMa(ControlUnit::decode(f));
        const MaRwControl m = ControlUnit::toMaRw(e);
        CHECK(e.memRead && e.regWrite && e.rd == 5 && m.regWrite && m.rd == 5 && !m.halt);
    }
    if (fails == 0) std::cout << "all control unit checks passed\n";
    else std::cout << "FAILURES: " << fails << "\n";
    return fails != 0;
}
