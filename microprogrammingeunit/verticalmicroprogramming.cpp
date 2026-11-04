#pragma once

#include "exec.cpp"
#include "floating_point_unit.cpp"
#include "control_unit_signals.cpp"
#include "exception_handler.cpp"
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

class MicroprogrammedControlUnit {
public:
    using Fault = Exec::Fault;
    using Op    = ISA::Opcode;

    enum class UOp : uint8_t {
        NOP,
        END,
        SRCA,
        SRCB,
        SRCC,
        IMMB,
        DEST,
        ALU,
        LONG,
        RES,
        MEMRD,
        MEMWR,
        BR,
        FPOP,
        MAT,
        MATWAIT,
        HALT,
        SERIAL,
        RDSR,
        WRSR,
        JMOD,
        JMP,
        UCALL,
        URET,
        CHK,
        COUNT
    };

    enum class Sel : uint8_t { RS1 = 0, RS2 = 1, RD = 2, LR = 3 };
    static constexpr unsigned FP_BIT = 4;

    enum class Check : uint8_t { MatDims, MatInner, DetSquare, NoModifier, SrSelect, Illegal };

    using MicroWord = uint16_t;
    static constexpr unsigned UOP_SHIFT  = 11;
    static constexpr unsigned ARG_MASK   = (1u << UOP_SHIFT) - 1;
    static constexpr unsigned MAX_STORE  = 1u << UOP_SHIFT;
    static constexpr unsigned WATCHDOG   = 64;
    static constexpr unsigned OP_COUNT   = ISA::opcodeValue(Op::OPCODE_COUNT);

    static constexpr MicroWord pack(UOp u, unsigned arg = 0) {
        return static_cast<MicroWord>((static_cast<unsigned>(u) << UOP_SHIFT) | (arg & ARG_MASK));
    }
    static constexpr UOp     uopOf(MicroWord m) { return static_cast<UOp>(m >> UOP_SHIFT); }
    static constexpr unsigned argOf(MicroWord m) { return m & ARG_MASK; }
    static constexpr unsigned reg(Sel s, bool fp = false) { return static_cast<unsigned>(s) | (fp ? FP_BIT : 0); }

    struct Result {
        ControlWord word;
        unsigned    microCycles = 0;
    };

    struct Stats {
        uint64_t instructions = 0;
        uint64_t microCycles  = 0;
        uint64_t faults       = 0;
        std::array<uint64_t, OP_COUNT> count{};
        std::array<uint64_t, OP_COUNT> cycles{};
        double avgCycles() const { return instructions ? double(microCycles) / double(instructions) : 0.0; }
    };

    MicroprogrammedControlUnit() { build(); }

    Result translateWord(ISA::InstrWord word, uint64_t pc, bool fetchFault) {
        ++stats_.instructions;
        if (fetchFault) return failEarly(Fault::FetchFault, pc);
        ISA::InstructionFields f;
        try { f = ISA::decode(word); }
        catch (const std::invalid_argument&) { return failEarly(Fault::IllegalInstruction, word); }
        return translate(f, word);
    }

    Result translate(const ISA::InstructionFields& f, ISA::InstrWord rawWord = 0) {
        begin(f, rawWord);
        while (!step()) {}
        return Result{cw_, steps_};
    }

    void begin(const ISA::InstructionFields& f, ISA::InstrWord rawWord = 0) {
        f_         = f;
        rawWord_   = rawWord;
        cw_        = ControlWord();
        cw_.valid  = true;
        cw_.opcode = f.opcode;
        cw_.immediate = f.immediate;
        upc_       = entry_[ISA::opcodeValue(f.opcode)];
        retUpc_    = 0;
        steps_     = 0;
        running_   = true;
    }

    bool step() {
        if (!running_) return true;
        if (++steps_ > WATCHDOG || upc_ >= store_.size()) return abort(Fault::IllegalInstruction);
        ++stats_.microCycles;

        const MicroWord m = store_[upc_++];
        const unsigned  a = argOf(m);
        switch (uopOf(m)) {
            case UOp::NOP:   break;
            case UOp::END:
                if (cw_.regWrite && !cw_.destFp && cw_.dest == ISA::PC) return abort(Fault::IllegalInstruction);
                return finish();
            case UOp::SRCA:  cw_.readsA = true; cw_.srcA = pick(a); cw_.srcAFp = (a & FP_BIT) != 0; break;
            case UOp::SRCB:  cw_.readsB = true; cw_.srcB = pick(a); cw_.srcBFp = (a & FP_BIT) != 0; break;
            case UOp::SRCC:  cw_.readsC = true; cw_.srcC = pick(a); break;
            case UOp::IMMB:  cw_.immB = true; break;
            case UOp::DEST:  cw_.regWrite = true; cw_.dest = pick(a); cw_.destFp = (a & FP_BIT) != 0; break;
            case UOp::ALU:   cw_.alu = static_cast<AluOp>(a); break;
            case UOp::LONG:  cw_.longOp = true; break;
            case UOp::RES:   cw_.result = static_cast<ResultSel>(a); break;
            case UOp::MEMRD: cw_.memRead = true; break;
            case UOp::MEMWR: cw_.memWrite = true; break;
            case UOp::BR:    cw_.branch = static_cast<BranchKind>(a & 0xF); cw_.conditionalBranch = (a & 0x10) != 0; break;
            case UOp::FPOP:
                cw_.fpOp     = true;
                cw_.fpKind   = static_cast<FPU::Op>(a);
                cw_.fpRounds = FPU::usesRounding(cw_.fpKind);
                break;
            case UOp::MAT:
                cw_.matrixOp = true;
                cw_.matRows  = f_.matrixRows;
                cw_.matInner = f_.matrixInner;
                cw_.matCols  = f_.matrixCols;
                break;
            case UOp::MATWAIT: cw_.matWait = true; break;
            case UOp::HALT:    cw_.halt = true; break;
            case UOp::SERIAL:  cw_.serializing = true; break;
            case UOp::RDSR:    cw_.readsSR = true; break;
            case UOp::WRSR:    cw_.writesSR = true; break;
            case UOp::JMOD:    if (f_.modifier != 0) upc_ = a; break;
            case UOp::JMP:     upc_ = a; break;
            case UOp::UCALL:   retUpc_ = upc_; upc_ = a; break;
            case UOp::URET:    upc_ = retUpc_; break;
            case UOp::CHK:
                if (!checkOk(static_cast<Check>(a))) return abort(Fault::IllegalInstruction);
                break;
            default:           return abort(Fault::IllegalInstruction);
        }
        return !running_;
    }

    bool                running() const { return running_; }
    const ControlWord&  word()    const { return cw_; }
    unsigned            microPc() const { return upc_; }

    unsigned entryPoint(Op op) const { return entry_[ISA::opcodeValue(op)]; }
    size_t   storeWords() const { return store_.size(); }
    size_t   storeBits()  const { return store_.size() * 16; }
    const Stats& stats()  const { return stats_; }

    std::string disassemble(unsigned addr) const {
        static const char* names[] = {"NOP","END","SRCA","SRCB","SRCC","IMMB","DEST","ALU","LONG","RES","MEMRD",
                                      "MEMWR","BR","FPOP","MAT","MATWAIT","HALT","SERIAL","RDSR","WRSR","JMOD",
                                      "JMP","UCALL","URET","CHK"};
        if (addr >= store_.size()) return "<out of range>";
        const unsigned u = static_cast<unsigned>(uopOf(store_[addr]));
        return std::to_string(addr) + ": " + (u < static_cast<unsigned>(UOp::COUNT) ? names[u] : "???") +
               " " + std::to_string(argOf(store_[addr]));
    }

    unsigned selfCheck() {
        unsigned bad = 0;
        auto cmp = [&](ISA::InstrWord w) {
            const ControlWord ref = ControlUnit::decodeWord(w, 0x1000, false);
            const ControlWord got = translateWord(w, 0x1000, false).word;
            if (!equal(ref, got)) ++bad;
        };

        for (unsigned o = 0; o < OP_COUNT; ++o)
            for (unsigned mod = 0; mod < 2; ++mod)
                for (unsigned r = 0; r < 16; r += 5) {
                    ISA::InstructionFields f;
                    f.opcode = static_cast<Op>(o);
                    f.rd = r; f.rs1 = (r + 3) & 15; f.rs2 = (r + 7) & 15;
                    f.modifier = static_cast<uint8_t>(mod);
                    f.immediate = (f.opcode == Op::RDSR || f.opcode == Op::WRSR) ? int32_t(r % 8) : int32_t(r) - 4;
                    f.matrixRows = 3; f.matrixCols = 3; f.matrixInner = 4;
                    try { cmp(ISA::encode(f)); } catch (const std::invalid_argument&) {}
                }

        uint32_t x = 0x1234ABCDu;
        for (unsigned i = 0; i < 200000; ++i) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            cmp(x);
        }

        if (!equal(ControlUnit::decodeWord(0, 0x40, true), translateWord(0, 0x40, true).word)) ++bad;
        return bad;
    }

private:

    uint8_t pick(unsigned a) const {
        switch (a & 3) {
            case 0:  return f_.rs1;
            case 1:  return f_.rs2;
            case 2:  return f_.rd;
            default: return ISA::LR;
        }
    }

    bool checkOk(Check c) const {
        auto dimOk = [](unsigned d) { return d >= 1 && d <= ISA::MAX_MATRIX_ROWS; };
        switch (c) {
            case Check::MatDims:    return dimOk(f_.matrixRows) && dimOk(f_.matrixCols);
            case Check::MatInner:   return dimOk(f_.matrixInner);
            case Check::DetSquare:  return f_.matrixRows == f_.matrixCols;
            case Check::NoModifier: return f_.modifier == 0;
            case Check::SrSelect:   return ExceptionHandler::validSelect(f_.immediate);
            case Check::Illegal:    return false;
        }
        return false;
    }

    static ControlWord faulted(Fault fault, uint64_t value, Op op) {
        ControlWord w;
        w.valid = true;
        w.opcode = op;
        w.fault = fault;
        w.faultValue = value;
        return w;
    }

    bool abort(Fault fault) {
        cw_ = faulted(fault, rawWord_, f_.opcode);
        ++stats_.faults;
        return finish();
    }

    bool finish() {
        running_ = false;
        const unsigned o = ISA::opcodeValue(f_.opcode);
        stats_.count[o]  += 1;
        stats_.cycles[o] += steps_;
        return true;
    }

    Result failEarly(Fault fault, uint64_t value) {
        ++stats_.faults;
        ++stats_.microCycles;
        cw_ = faulted(fault, value, Op::NOP);
        return Result{cw_, 1};
    }

    static bool equal(const ControlWord& a, const ControlWord& b) {
        return a.valid == b.valid && a.opcode == b.opcode &&
               a.readsA == b.readsA && a.readsB == b.readsB && a.readsC == b.readsC && a.immB == b.immB &&
               a.srcAFp == b.srcAFp && a.srcBFp == b.srcBFp && a.destFp == b.destFp &&
               a.srcA == b.srcA && a.srcB == b.srcB && a.srcC == b.srcC && a.immediate == b.immediate &&
               a.alu == b.alu && a.longOp == b.longOp &&
               a.fpOp == b.fpOp && a.fpRounds == b.fpRounds && a.fpKind == b.fpKind &&
               a.regWrite == b.regWrite && a.dest == b.dest && a.result == b.result &&
               a.memRead == b.memRead && a.memWrite == b.memWrite &&
               a.branch == b.branch && a.conditionalBranch == b.conditionalBranch &&
               a.matrixOp == b.matrixOp && a.matWait == b.matWait &&
               a.matRows == b.matRows && a.matInner == b.matInner && a.matCols == b.matCols &&
               a.halt == b.halt && a.serializing == b.serializing &&
               a.readsSR == b.readsSR && a.writesSR == b.writesSR &&
               a.fault == b.fault && a.faultValue == b.faultValue;
    }

    unsigned emit(UOp u, unsigned arg = 0) {
        if (store_.size() >= MAX_STORE) throw std::length_error("MicroprogrammedControlUnit: control store full");
        store_.push_back(pack(u, arg));
        return static_cast<unsigned>(store_.size() - 1);
    }
    unsigned here() const { return static_cast<unsigned>(store_.size()); }
    void patch(unsigned at, unsigned target) { store_[at] = pack(uopOf(store_[at]), target); }

    static AluOp aluOf(Op op) {
        switch (op) {
            case Op::ADD: return AluOp::ADD;  case Op::SUB: return AluOp::SUB;
            case Op::MUL: return AluOp::MUL;  case Op::DIV: return AluOp::DIV;
            case Op::MOD: return AluOp::MOD;  case Op::AND: return AluOp::AND;
            case Op::OR:  return AluOp::OR;   case Op::XOR: return AluOp::XOR;
            case Op::NOT: return AluOp::NOT;  case Op::LSL: return AluOp::LSL;
            case Op::LSR: return AluOp::LSR;  case Op::ASR: return AluOp::ASR;
            case Op::SIN: return AluOp::SIN;  case Op::COS: return AluOp::COS;
            case Op::LOG: return AluOp::LOG;  case Op::CMP: return AluOp::CMP;
            default:      return AluOp::NONE;
        }
    }

    static BranchKind branchOf(Op op) {
        switch (op) {
            case Op::JEQ: return BranchKind::JEQ;   case Op::JGT: return BranchKind::JGT;
            case Op::JZ:  return BranchKind::JZ;    case Op::JNZ: return BranchKind::JNZ;
            case Op::JMP: return BranchKind::JMP;   case Op::CALL: return BranchKind::CALL;
            case Op::RET: return BranchKind::RET;   default: return BranchKind::NONE;
        }
    }

    static unsigned br(BranchKind k, bool cond) { return static_cast<unsigned>(k) | (cond ? 0x10u : 0u); }

    void build() {
        store_.reserve(256);

        aluCommon_ = here();
        emit(UOp::SRCA, reg(Sel::RS1));
        emit(UOp::DEST, reg(Sel::RD));
        emit(UOp::RES,  static_cast<unsigned>(ResultSel::ALU));
        emit(UOp::URET);

        for (unsigned o = 0; o < OP_COUNT; ++o) {
            entry_[o] = static_cast<uint16_t>(here());
            routine(static_cast<Op>(o));
        }
    }

    void routine(Op op) {
        switch (ISA::typeOf(op)) {
            case ISA::InstructionType::ALU:    aluRoutine(op); break;
            case ISA::InstructionType::MEMORY: memRoutine(op); break;
            case ISA::InstructionType::BRANCH: branchRoutine(op); break;
            case ISA::InstructionType::MATRIX: matrixRoutine(op); break;
            case ISA::InstructionType::SYSTEM: systemRoutine(op); break;
            case ISA::InstructionType::FP:     fpRoutine(op); break;
        }
    }

    void aluRoutine(Op op) {
        if (op == Op::EXP || op == Op::POW) { emit(UOp::CHK, static_cast<unsigned>(Check::Illegal)); emit(UOp::END); return; }
        emit(UOp::ALU, static_cast<unsigned>(aluOf(op)));
        if (Exec::isLongOp(op)) emit(UOp::LONG);
        emit(UOp::UCALL, aluCommon_);
        if (!ISA::isUnary(op)) {
            const unsigned j = emit(UOp::JMOD);
            emit(UOp::SRCB, reg(Sel::RS2));
            emit(UOp::END);
            patch(j, here());
            emit(UOp::IMMB);
        }
        emit(UOp::END);
    }

    void memRoutine(Op op) {
        const bool fp = (op == Op::FLOAD || op == Op::FSTORE);
        emit(UOp::CHK, static_cast<unsigned>(Check::NoModifier));
        emit(UOp::SRCA, reg(Sel::RS1));
        if (op == Op::LOAD || op == Op::FLOAD) {
            emit(UOp::MEMRD);
            emit(UOp::DEST, reg(Sel::RD, fp));
            emit(UOp::RES, static_cast<unsigned>(ResultSel::LOAD));
        } else {
            emit(UOp::MEMWR);
            emit(UOp::SRCB, reg(Sel::RD, fp));
        }
        emit(UOp::END);
    }

    void branchRoutine(Op op) {
        switch (op) {
            case Op::JEQ: case Op::JGT:
                emit(UOp::SRCA, reg(Sel::RS1));
                emit(UOp::SRCB, reg(Sel::RS2));
                emit(UOp::BR, br(branchOf(op), true));
                break;
            case Op::JZ: case Op::JNZ:
                emit(UOp::SRCA, reg(Sel::RS1));
                emit(UOp::BR, br(branchOf(op), true));
                break;
            case Op::JMP:
                emit(UOp::BR, br(BranchKind::JMP, false));
                break;
            case Op::CALL:
                emit(UOp::BR, br(BranchKind::CALL, false));
                emit(UOp::DEST, reg(Sel::LR));
                emit(UOp::RES, static_cast<unsigned>(ResultSel::LINK));
                break;
            case Op::RET:
                emit(UOp::SRCA, reg(Sel::LR));
                emit(UOp::BR, br(BranchKind::RET, false));
                break;
            default: break;
        }
        emit(UOp::END);
    }

    void matrixRoutine(Op op) {
        emit(UOp::CHK, static_cast<unsigned>(Check::MatDims));
        if (op == Op::MATMUL)          emit(UOp::CHK, static_cast<unsigned>(Check::MatInner));
        if (op == Op::MATDETERMINANT)  emit(UOp::CHK, static_cast<unsigned>(Check::DetSquare));
        emit(UOp::SRCC, reg(Sel::RD));
        emit(UOp::SRCA, reg(Sel::RS1));
        if (op != Op::MATDETERMINANT) emit(UOp::SRCB, reg(Sel::RS2));
        emit(UOp::MAT);
        emit(UOp::END);
    }

    void systemRoutine(Op op) {
        switch (op) {
            case Op::HALT:    emit(UOp::HALT); break;
            case Op::MATWAIT: emit(UOp::MATWAIT); break;
            case Op::RDSR:
                emit(UOp::CHK, static_cast<unsigned>(Check::SrSelect));
                emit(UOp::DEST, reg(Sel::RD));
                emit(UOp::RES, static_cast<unsigned>(ResultSel::SPECIAL));
                emit(UOp::RDSR);
                emit(UOp::SERIAL);
                break;
            case Op::WRSR:
                emit(UOp::CHK, static_cast<unsigned>(Check::SrSelect));
                emit(UOp::SRCA, reg(Sel::RS1));
                emit(UOp::WRSR);
                emit(UOp::SERIAL);
                break;
            case Op::RETI:
                emit(UOp::BR, br(BranchKind::RETI, false));
                emit(UOp::SERIAL);
                break;
            default: break;
        }
        emit(UOp::END);
    }

    void fpRoutine(Op op) {
        const bool binary = !ISA::isUnary(op);
        if (binary) emit(UOp::CHK, static_cast<unsigned>(Check::NoModifier));
        emit(UOp::FPOP, static_cast<unsigned>(Exec::fpOpOf(op)));
        const bool srcAfp = !(op == Op::FCVTDQ || op == Op::FMVDX);
        const bool dstFp  = !(op == Op::FCMP || op == Op::FCVTQD || op == Op::FMVXD);
        emit(UOp::SRCA, reg(Sel::RS1, srcAfp));
        if (binary) emit(UOp::SRCB, reg(Sel::RS2, true));
        emit(UOp::DEST, reg(Sel::RD, dstFp));
        emit(UOp::RES, static_cast<unsigned>(ResultSel::FPU));
        emit(UOp::END);
    }

    std::vector<MicroWord>          store_;
    std::array<uint16_t, OP_COUNT>  entry_{};
    unsigned                        aluCommon_ = 0;

    ISA::InstructionFields f_{};
    ISA::InstrWord         rawWord_ = 0;
    ControlWord            cw_;
    unsigned               upc_ = 0, retUpc_ = 0, steps_ = 0;
    bool                   running_ = false;
    Stats                  stats_;
};
