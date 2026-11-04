#pragma once

#include "control_unit.cpp"
#include <array>
#include <cstdint>
#include <sstream>
#include <string>

namespace MicroField {
struct F { unsigned lo, w; };
constexpr F after(F p, unsigned w) { return F{p.lo + p.w, w}; }

constexpr F READ_A       {0, 1};
constexpr F READ_B       = after(READ_A, 1);
constexpr F ALLOW_IMM    = after(READ_B, 1);
constexpr F SRC_A_SEL    = after(ALLOW_IMM, 1);
constexpr F SRC_B_SEL    = after(SRC_A_SEL, 1);
constexpr F SRC_A_FP     = after(SRC_B_SEL, 1);
constexpr F SRC_B_FP     = after(SRC_A_FP, 1);
constexpr F DEST_FP      = after(SRC_B_FP, 1);
constexpr F DEST_SEL     = after(DEST_FP, 1);
constexpr F REG_WRITE    = after(DEST_SEL, 1);
constexpr F RESULT       = after(REG_WRITE, 3);
constexpr F ALU_OP       = after(RESULT, 5);
constexpr F LONG_OP      = after(ALU_OP, 1);
constexpr F FP_OP        = after(LONG_OP, 1);
constexpr F FP_ROUNDS    = after(FP_OP, 1);
constexpr F FP_KIND      = after(FP_ROUNDS, 4);
constexpr F MEM_READ     = after(FP_KIND, 1);
constexpr F MEM_WRITE    = after(MEM_READ, 1);
constexpr F BRANCH       = after(MEM_WRITE, 4);
constexpr F COND         = after(BRANCH, 1);
constexpr F HALT         = after(COND, 1);
constexpr F SERIAL       = after(HALT, 1);
constexpr F READS_SR     = after(SERIAL, 1);
constexpr F WRITES_SR    = after(READS_SR, 1);
constexpr F NEXT         = after(WRITES_SR, 6);
constexpr F LAST         = after(NEXT, 1);
constexpr unsigned WIDTH = LAST.lo + LAST.w;

constexpr uint64_t mask(F f) { return ((uint64_t(1) << f.w) - 1) << f.lo; }
constexpr uint64_t get(uint64_t row, F f) { return (row >> f.lo) & ((uint64_t(1) << f.w) - 1); }
constexpr uint64_t put(uint64_t row, F f, uint64_t v) { return (row & ~mask(f)) | ((v << f.lo) & mask(f)); }
constexpr uint64_t SEQUENCER_MASK = mask(NEXT) | mask(LAST);
}

static_assert(MicroField::WIDTH <= 64, "microword must fit in 64 bits");

class HorizontalMicrocodeUnit4 {
public:
    using Fault = Exec::Fault;
    using Op = ISA::Opcode;
    static constexpr unsigned OP_COUNT = ISA::opcodeValue(Op::OPCODE_COUNT);
    static constexpr unsigned STORE_ROWS = 64;

    struct Result {
        ControlWord word{};
        ExMaControl exMa{};
        MaRwControl maRw{};
        unsigned microCycles = 0;
        bool done = false;
    };

    struct Stats {
        uint64_t instructions = 0;
        uint64_t microCycles = 0;
        uint64_t faults = 0;
        std::array<uint64_t, OP_COUNT> opcodeCount{};
        std::array<uint64_t, OP_COUNT> opcodeCycles{};
        double avgCycles() const {
            return instructions == 0 ? 0.0 : static_cast<double>(microCycles) / instructions;
        }
    };

    HorizontalMicrocodeUnit4() { buildStore(); reset(); }

    void reset() {
        active_ = false;
        upc_ = 0;
        acc_ = 0;
        cycles_ = 0;
        pending_ = {};
        current_ = {};
        stats_ = {};
    }

    Result translateWord(ISA::InstrWord raw, uint64_t pc, bool fetchFault = false) const {
        if (fetchFault) return faultResult(Fault::FetchFault, pc, Op::NOP);
        ISA::InstructionFields f;
        try { f = ISA::decode(raw); }
        catch (const std::invalid_argument&) { return faultResult(Fault::IllegalInstruction, raw, Op::NOP); }
        return translate(f, raw);
    }

    Result translate(const ISA::InstructionFields& f, ISA::InstrWord raw = 0) const {
        const Fault bad = ControlUnit::validate(f);
        if (bad != Fault::None) return faultResult(bad, raw, f.opcode);
        uint64_t acc = 0;
        unsigned addr = entryPoint(f.opcode), cycles = 0;
        for (;;) {
            const uint64_t row = store_[addr];
            acc |= row & ~MicroField::SEQUENCER_MASK;
            ++cycles;
            if (MicroField::get(row, MicroField::LAST)) break;
            addr = unsigned(MicroField::get(row, MicroField::NEXT));
        }
        return assemble(acc, f, cycles);
    }

    void begin(ISA::InstrWord raw, uint64_t pc, bool fetchFault = false) {
        pending_ = {};
        current_ = {};
        acc_ = 0;
        cycles_ = 0;
        fields_ = {};
        if (fetchFault) { current_ = faultResult(Fault::FetchFault, pc, Op::NOP); active_ = true; direct_ = true; upc_ = 0; return; }
        try { fields_ = ISA::decode(raw); }
        catch (const std::invalid_argument&) { current_ = faultResult(Fault::IllegalInstruction, raw, Op::NOP); active_ = true; direct_ = true; upc_ = 0; return; }
        const Fault bad = ControlUnit::validate(fields_);
        if (bad != Fault::None) { current_ = faultResult(bad, raw, fields_.opcode); active_ = true; direct_ = true; upc_ = 0; return; }
        direct_ = false;
        active_ = true;
        upc_ = entryPoint(fields_.opcode);
    }

    Result step() {
        if (!active_) return {};
        if (direct_) {
            active_ = false;
            finish(current_);
            return current_;
        }
        const uint64_t row = store_[upc_];
        acc_ |= row & ~MicroField::SEQUENCER_MASK;
        ++cycles_;
        if (MicroField::get(row, MicroField::LAST)) {
            active_ = false;
            current_ = assemble(acc_, fields_, cycles_);
            finish(current_);
            return current_;
        }
        upc_ = unsigned(MicroField::get(row, MicroField::NEXT));
        Result partial;
        partial.microCycles = cycles_;
        partial.done = false;
        return partial;
    }

    bool running() const { return active_; }
    const ControlWord& word() const { return current_.word; }
    const Result& result() const { return current_; }
    unsigned microPc() const { return upc_; }
    unsigned entryPoint(Op opcode) const { return ISA::opcodeValue(opcode); }
    unsigned storeWords() const { return used_; }
    unsigned storeBits() const { return MicroField::WIDTH; }
    uint64_t row(unsigned address) const { return store_[address]; }
    const Stats& stats() const { return stats_; }

    std::string disassemble(const ControlWord& w) const {
        std::ostringstream out;
        out << "VALID=" << w.valid << " OPCODE=" << ISA::mnemonicOf(w.opcode)
            << " READ_A=" << w.readsA << " READ_B=" << w.readsB << " IMM_B=" << w.immB
            << " SRC_A=" << unsigned(w.srcA) << " SRC_B=" << unsigned(w.srcB)
            << " SRC_A_FP=" << w.srcAFp << " SRC_B_FP=" << w.srcBFp << " DEST_FP=" << w.destFp
            << " ALU=" << unsigned(w.alu) << " LONG_OP=" << w.longOp
            << " FP_OP=" << w.fpOp << " FP_ROUNDS=" << w.fpRounds
            << " REG_WRITE=" << w.regWrite << " DEST=" << unsigned(w.dest) << " RESULT=" << unsigned(w.result)
            << " MEM_READ=" << w.memRead << " MEM_WRITE=" << w.memWrite
            << " BRANCH=" << unsigned(w.branch) << " CONDITIONAL=" << w.conditionalBranch
            << " HALT=" << w.halt << " SERIALIZING=" << w.serializing
            << " READ_SR=" << w.readsSR << " WRITE_SR=" << w.writesSR
            << " FAULT=" << unsigned(w.fault) << " FAULT_VALUE=" << w.faultValue;
        return out.str();
    }

    std::string disassemble() const { return disassemble(current_.word); }

    std::string disassembleRow(unsigned address) const {
        using namespace MicroField;
        const uint64_t r = store_[address];
        std::ostringstream out;
        out << "uROM[" << address << "] =";
        static const struct { const char* n; F f; } names[] = {
            {"READ_A", READ_A}, {"READ_B", READ_B}, {"ALLOW_IMM", ALLOW_IMM},
            {"SRC_A_SEL", SRC_A_SEL}, {"SRC_B_SEL", SRC_B_SEL}, {"SRC_A_FP", SRC_A_FP}, {"SRC_B_FP", SRC_B_FP},
            {"DEST_FP", DEST_FP}, {"DEST_SEL", DEST_SEL}, {"REG_WRITE", REG_WRITE}, {"RESULT", RESULT}, {"ALU_OP", ALU_OP},
            {"LONG_OP", LONG_OP}, {"FP_OP", FP_OP}, {"FP_ROUNDS", FP_ROUNDS}, {"FP_KIND", FP_KIND},
            {"MEM_READ", MEM_READ}, {"MEM_WRITE", MEM_WRITE}, {"BRANCH", BRANCH}, {"COND", COND},
            {"HALT", HALT}, {"SERIAL", SERIAL},
            {"READS_SR", READS_SR}, {"WRITES_SR", WRITES_SR},
            {"NEXT", NEXT}, {"LAST", LAST}};
        for (const auto& n : names)
            if (get(r, n.f)) out << " " << n.n << "=" << get(r, n.f);
        return out.str();
    }

    bool selfCheck() const {
        for (unsigned op = 0; op < OP_COUNT; ++op) {
            unsigned addr = op, hops = 0;
            for (;;) {
                if (addr >= used_) return false;
                const uint64_t r = store_[addr];
                if (MicroField::get(r, MicroField::LAST)) break;
                addr = unsigned(MicroField::get(r, MicroField::NEXT));
                if (addr < OP_COUNT || ++hops > 4) return false;
            }
        }
        for (unsigned a = 0; a < used_; ++a)
            if (store_[a] >> MicroField::WIDTH) return false;
        return true;
    }

private:
    using F = MicroField::F;

    Result faultResult(Fault fault, uint64_t value, Op op) const {
        Result r;
        r.word.valid = true;
        r.word.opcode = op;
        r.word.fault = fault;
        r.word.faultValue = value;
        r.exMa = ControlUnit::toExMa(r.word);
        r.maRw = ControlUnit::toMaRw(r.exMa);
        r.microCycles = 1;
        r.done = true;
        return r;
    }

    void finish(const Result& r) {
        ++stats_.instructions;
        stats_.microCycles += r.microCycles;
        if (r.word.fault != Fault::None) { ++stats_.faults; return; }
        const unsigned i = ISA::opcodeValue(r.word.opcode);
        if (i < OP_COUNT) { ++stats_.opcodeCount[i]; stats_.opcodeCycles[i] += r.microCycles; }
    }

    static Result assemble(uint64_t acc, const ISA::InstructionFields& f, unsigned cycles) {
        using namespace MicroField;
        Result r;
        ControlWord& w = r.word;
        w.valid = true;
        w.opcode = f.opcode;
        w.immediate = f.immediate;
        w.readsA = get(acc, READ_A);
        w.immB = get(acc, ALLOW_IMM) && f.modifier != 0;
        w.readsB = get(acc, READ_B) && !w.immB;
        w.srcAFp = get(acc, SRC_A_FP);
        w.srcBFp = get(acc, SRC_B_FP) && w.readsB;
        if (w.readsA) w.srcA = get(acc, SRC_A_SEL) ? ISA::LR : f.rs1;
        if (w.readsB) w.srcB = get(acc, SRC_B_SEL) ? f.rd : f.rs2;
        w.alu = static_cast<AluOp>(get(acc, ALU_OP));
        w.longOp = get(acc, LONG_OP);
        w.fpOp = get(acc, FP_OP);
        w.fpRounds = get(acc, FP_ROUNDS);
        if (w.fpOp) w.fpKind = static_cast<FPU::Op>(get(acc, FP_KIND));
        w.regWrite = get(acc, REG_WRITE);
        if (w.regWrite) {
            w.dest = get(acc, DEST_SEL) ? ISA::LR : f.rd;
            w.destFp = get(acc, DEST_FP);
        }
        w.result = static_cast<ResultSel>(get(acc, RESULT));
        w.memRead = get(acc, MEM_READ);
        w.memWrite = get(acc, MEM_WRITE);
        w.branch = static_cast<BranchKind>(get(acc, BRANCH));
        w.conditionalBranch = get(acc, COND);
        w.halt = get(acc, HALT);
        w.serializing = get(acc, SERIAL);
        w.readsSR = get(acc, READS_SR);
        w.writesSR = get(acc, WRITES_SR);

        r.exMa = ControlUnit::toExMa(w);
        r.maRw = ControlUnit::toMaRw(r.exMa);
        r.microCycles = cycles;
        r.done = true;
        return r;
    }

    struct RowBuilder {
        uint64_t bits = 0;
        RowBuilder& set(F f, uint64_t v = 1) { bits = MicroField::put(bits, f, v); return *this; }
    };

    static uint64_t aluCode(Op op) {
        switch (op) {
            case Op::ADD: return uint64_t(AluOp::ADD);
            case Op::SUB: return uint64_t(AluOp::SUB);
            case Op::MUL: return uint64_t(AluOp::MUL);
            case Op::DIV: return uint64_t(AluOp::DIV);
            case Op::MOD: return uint64_t(AluOp::MOD);
            case Op::AND: return uint64_t(AluOp::AND);
            case Op::OR:  return uint64_t(AluOp::OR);
            case Op::XOR: return uint64_t(AluOp::XOR);
            case Op::NOT: return uint64_t(AluOp::NOT);
            case Op::LSL: return uint64_t(AluOp::LSL);
            case Op::LSR: return uint64_t(AluOp::LSR);
            case Op::ASR: return uint64_t(AluOp::ASR);
            case Op::SIN: return uint64_t(AluOp::SIN);
            case Op::COS: return uint64_t(AluOp::COS);
            case Op::LOG: return uint64_t(AluOp::LOG);
            case Op::CMP: return uint64_t(AluOp::CMP);
            default:      return uint64_t(AluOp::NONE);
        }
    }

    unsigned append(uint64_t row) { store_[used_] = row; return used_++; }

    void chain(Op op, uint64_t first, uint64_t second) {
        using namespace MicroField;
        const unsigned cont = append(put(second, LAST, 1));
        store_[ISA::opcodeValue(op)] = put(put(first, NEXT, cont), LAST, 0);
    }

    void single(Op op, const RowBuilder& b) {
        store_[ISA::opcodeValue(op)] = MicroField::put(b.bits, MicroField::LAST, 1);
    }

    void buildStore() {
        using namespace MicroField;
        store_.fill(0);
        used_ = OP_COUNT;

        for (unsigned i = 0; i < OP_COUNT; ++i) {
            const Op op = static_cast<Op>(i);
            RowBuilder b;
            switch (ISA::typeOf(op)) {
                case ISA::InstructionType::ALU:
                    b.set(READ_A).set(REG_WRITE).set(RESULT, uint64_t(ResultSel::ALU)).set(ALU_OP, aluCode(op));
                    if (!ISA::isUnary(op)) b.set(READ_B).set(ALLOW_IMM);
                    if (Exec::isLongOp(op)) b.set(LONG_OP);
                    single(op, b);
                    break;
                case ISA::InstructionType::MEMORY:
                    b.set(READ_A);
                    if (op == Op::LOAD || op == Op::FLOAD) {
                        b.set(REG_WRITE).set(MEM_READ).set(RESULT, uint64_t(ResultSel::LOAD));
                        if (op == Op::FLOAD) b.set(DEST_FP);
                    } else {
                        b.set(READ_B).set(SRC_B_SEL).set(MEM_WRITE);
                        if (op == Op::FSTORE) b.set(SRC_B_FP);
                    }
                    single(op, b);
                    break;
                case ISA::InstructionType::BRANCH:
                    switch (op) {
                        case Op::JEQ: b.set(READ_A).set(READ_B).set(BRANCH, uint64_t(BranchKind::JEQ)).set(COND); break;
                        case Op::JGT: b.set(READ_A).set(READ_B).set(BRANCH, uint64_t(BranchKind::JGT)).set(COND); break;
                        case Op::JZ:  b.set(READ_A).set(BRANCH, uint64_t(BranchKind::JZ)).set(COND); break;
                        case Op::JNZ: b.set(READ_A).set(BRANCH, uint64_t(BranchKind::JNZ)).set(COND); break;
                        case Op::JMP: b.set(BRANCH, uint64_t(BranchKind::JMP)); break;
                        case Op::CALL:
                            b.set(BRANCH, uint64_t(BranchKind::CALL)).set(REG_WRITE).set(DEST_SEL)
                             .set(RESULT, uint64_t(ResultSel::LINK));
                            break;
                        default:
                            b.set(BRANCH, uint64_t(BranchKind::RET)).set(READ_A).set(SRC_A_SEL);
                            break;
                    }
                    single(op, b);
                    break;
                case ISA::InstructionType::SYSTEM: {
                    RowBuilder drain, body;
                    drain.set(SERIAL);
                    switch (op) {
                        case Op::RDSR:
                            body.set(REG_WRITE).set(RESULT, uint64_t(ResultSel::SPECIAL)).set(READS_SR);
                            chain(op, drain.bits, body.bits);
                            break;
                        case Op::WRSR:
                            body.set(READ_A).set(WRITES_SR);
                            chain(op, drain.bits, body.bits);
                            break;
                        case Op::RETI:
                            body.set(BRANCH, uint64_t(BranchKind::RETI));
                            chain(op, drain.bits, body.bits);
                            break;
                        case Op::HALT:    single(op, b.set(HALT)); break;
                        default:          single(op, b); break;
                    }
                    break;
                }
                case ISA::InstructionType::FP: {
                    const FPU::Op k = Exec::fpOpOf(op);
                    b.set(READ_A).set(SRC_A_FP).set(REG_WRITE).set(DEST_FP).set(RESULT, uint64_t(ResultSel::FPU))
                     .set(FP_OP).set(FP_KIND, uint64_t(k));
                    if (FPU::usesRounding(k)) b.set(FP_ROUNDS);
                    switch (op) {
                        case Op::FADD: case Op::FSUB: case Op::FMUL: case Op::FDIV:
                            b.set(READ_B).set(SRC_B_FP); break;
                        case Op::FCMP:
                            b.set(READ_B).set(SRC_B_FP).set(DEST_FP, 0); break;
                        case Op::FCVTQD: case Op::FMVXD:
                            b.set(DEST_FP, 0); break;
                        case Op::FCVTDQ: case Op::FMVDX:
                            b.set(SRC_A_FP, 0); break;
                        default: break;
                    }
                    single(op, b);
                    break;
                }
            }
        }
    }

    std::array<uint64_t, STORE_ROWS> store_{};
    unsigned used_ = 0;

    bool active_ = false, direct_ = false;
    unsigned upc_ = 0, cycles_ = 0;
    uint64_t acc_ = 0;
    ISA::InstructionFields fields_{};
    Result pending_{};
    Result current_{};
    Stats stats_{};
};

