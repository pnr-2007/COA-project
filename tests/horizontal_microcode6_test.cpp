#include "horizontal_microcode6.cpp"
#include <iostream>
#include <random>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { ++fails; std::cout << "  FAIL line " << __LINE__ << ": " #c "\n"; } } while (0)

using Op = ISA::Opcode;
using U = HorizontalMicrocodeUnit6;

static bool sameWord(const ControlWord& a, const ControlWord& b) {
    bool ok = a.valid == b.valid && a.opcode == b.opcode && a.fault == b.fault && a.faultValue == b.faultValue &&
              a.immediate == b.immediate && a.readsA == b.readsA && a.readsB == b.readsB &&
              a.immB == b.immB && a.alu == b.alu && a.longOp == b.longOp && a.fpOp == b.fpOp && a.fpRounds == b.fpRounds &&
              a.regWrite == b.regWrite && a.result == b.result && a.memRead == b.memRead && a.memWrite == b.memWrite &&
              a.branch == b.branch && a.conditionalBranch == b.conditionalBranch &&
              a.halt == b.halt && a.serializing == b.serializing && a.readsSR == b.readsSR && a.writesSR == b.writesSR;
    if (a.readsA) ok = ok && a.srcA == b.srcA && a.srcAFp == b.srcAFp;
    if (a.readsB) ok = ok && a.srcB == b.srcB && a.srcBFp == b.srcBFp;
    if (a.regWrite) ok = ok && a.dest == b.dest && a.destFp == b.destFp;
    if (a.fpOp) ok = ok && a.fpKind == b.fpKind;
    return ok;
}

int main() {
    U u;
    CHECK(u.selfCheck());
    CHECK(u.storeBits() <= 64);
    CHECK(u.storeWords() == U::OP_COUNT + 3);
    std::cout << "control store: " << u.storeWords() << " rows x " << u.storeBits() << " bits\n";

    uint64_t combos = 0;
    const uint8_t regs[] = {0, 3, 13, 14, 15};
    const int32_t imms[] = {-5, 0, 1, 5, 9};
    for (uint8_t o = 0; o < ISA::opcodeValue(Op::OPCODE_COUNT); ++o)
        for (uint8_t rd : regs) for (uint8_t rs1 : regs) for (uint8_t rs2 : regs)
        for (uint8_t mod : {0, 1, 2}) for (int32_t imm : imms)
        {
            ISA::InstructionFields f;
            f.opcode = static_cast<Op>(o); f.rd = rd; f.rs1 = rs1; f.rs2 = rs2; f.modifier = mod; f.immediate = imm;
            const U::Result m = u.translate(f, 0x1234);
            ++combos;
            bool ok;
            if (ControlUnit::validate(f) != Exec::Fault::None) {
                ok = m.word.fault == ControlUnit::validate(f) && m.word.faultValue == 0x1234 && m.microCycles == 1;
            } else {
                const ControlWord ref = ControlUnit::decode(f);
                const ControlUnit6 *dummy = nullptr; (void)dummy;
                const OperandNeeds6 n = ControlUnit6::needs(ref);
                const bool twoRows = ref.serializing;
                ok = sameWord(m.word, ref) && m.needs.a == n.a && m.needs.b == n.b &&
                     m.stage.resultReady == ControlUnit6::resultReady(ref) && m.stage.mem == ControlUnit6::memAction(ref) &&
                     m.microCycles == (twoRows ? 2u : 1u) && m.done;
                const StageControl6 s = ControlUnit6::toStage(ref);
                ok = ok && m.stage.regWrite == s.regWrite && m.stage.destFp == s.destFp && m.stage.memRead == s.memRead &&
                     m.stage.memWrite == s.memWrite && m.stage.halt == s.halt && m.stage.rd == s.rd;
            }
            if (!ok) { CHECK(ok); std::cout << "    opcode " << ISA::mnemonicOf(f.opcode) << " rd " << int(rd) << " rs1 " << int(rs1) << " rs2 " << int(rs2) << " mod " << int(mod) << "\n"; if (fails > 5) return 1; }
        }
    std::cout << "microcode decode combinations checked against ControlUnit: " << combos << "\n";

    std::mt19937 g(99);
    uint64_t words = 0, faults = 0;
    for (int i = 0; i < 300000; ++i) {
        const ISA::InstrWord w = g();
        const U::Result m = u.translateWord(w, 0x400);
        const ControlWord ref = ControlUnit::decodeWord(w, 0x400, false);
        ++words;
        if (ref.fault != Exec::Fault::None) ++faults;
        const bool ok = ref.fault != Exec::Fault::None ? (m.word.fault == ref.fault && m.word.faultValue == ref.faultValue)
                                                       : sameWord(m.word, ref);
        if (!ok) { CHECK(ok); if (fails > 5) return 1; }
    }
    {
        const U::Result m = u.translateWord(0, 0x808, true);
        CHECK(m.word.fault == Exec::Fault::FetchFault && m.word.faultValue == 0x808);
    }
    std::cout << "random words checked: " << words << " (" << faults << " faulting)\n";

    {
        U s;
        ISA::InstructionFields f; f.opcode = Op::RDSR; f.rd = 4; f.immediate = 0;
        s.begin(ISA::encode(f), 0);
        CHECK(s.running() && s.microPc() == ISA::opcodeValue(Op::RDSR));
        U::Result r = s.step();
        CHECK(!r.done && s.running() && r.microCycles == 1 && s.microPc() >= U::OP_COUNT);
        r = s.step();
        CHECK(r.done && !s.running() && r.microCycles == 2 && s.word().serializing && s.word().regWrite && s.word().dest == 4);
        CHECK(s.step().microCycles == 0);

        f = ISA::InstructionFields(); f.opcode = Op::ADD; f.rd = 1; f.rs1 = 2; f.rs2 = 3;
        s.begin(ISA::encode(f), 4);
        r = s.step();
        CHECK(r.done && r.microCycles == 1 && s.word().alu == AluOp::ADD && !s.running());

        s.begin(0xFFFFFFFFu, 8);
        r = s.step();
        CHECK(r.done && s.word().fault == Exec::Fault::IllegalInstruction);

        CHECK(s.stats().instructions == 3 && s.stats().faults == 1 && s.stats().microCycles == 4);
        CHECK(s.stats().opcodeCount[ISA::opcodeValue(Op::RDSR)] == 1 && s.stats().opcodeCycles[ISA::opcodeValue(Op::RDSR)] == 2);
        CHECK(s.stats().avgCycles() > 1.3 && s.stats().avgCycles() < 1.4);
    }
    {
        U s;
        std::cout << s.disassembleRow(ISA::opcodeValue(Op::LOAD)) << "\n";
        std::cout << s.disassembleRow(ISA::opcodeValue(Op::FSTORE)) << "\n";
    }
    std::cout << (fails ? "MICROCODE CHECKS FAILED\n" : "all microcode checks passed\n");
    return fails ? 1 : 0;
}
