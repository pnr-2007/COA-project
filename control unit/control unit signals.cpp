
#pragma once

#include "exec.cpp"
#include "exception_handler.cpp"
#include <cstdint>

enum class AluOp : uint8_t { NONE, ADD, SUB, MUL, DIV, MOD, AND, OR, XOR, NOT, LSL, LSR, ASR, SIN, COS, LOG, CMP };
enum class ResultSel : uint8_t { NONE, ALU, LOAD, LINK, SPECIAL };
enum class BranchKind : uint8_t { NONE, JEQ, JGT, JZ, JNZ, JMP, CALL, RET, RETI };

struct ControlWord {
    using Fault = Exec::Fault;

    bool valid = false;
    ISA::Opcode opcode = ISA::Opcode::NOP;

    bool readsA = false, readsB = false, readsC = false, immB = false;
    uint8_t srcA = 0, srcB = 0, srcC = 0;
    int32_t immediate = 0;

    AluOp alu = AluOp::NONE;
    bool longOp = false;

    bool regWrite = false;
    uint8_t dest = 0;
    ResultSel result = ResultSel::NONE;

    bool memRead = false, memWrite = false;

    BranchKind branch = BranchKind::NONE;
    bool conditionalBranch = false;

    bool matrixOp = false, matWait = false;
    uint8_t matRows = 0, matInner = 0, matCols = 0;

    bool halt = false, serializing = false, readsSR = false, writesSR = false;

    Fault fault = Fault::None;
    uint64_t faultValue = 0;
};

struct ExMaControl {
    bool valid = false, regWrite = false, memRead = false, memWrite = false, halt = false, matrixOp = false;
    uint8_t rd = 0;
    Exec::Fault fault = Exec::Fault::None;
};

struct MaRwControl {
    bool valid = false, regWrite = false, halt = false;
    uint8_t rd = 0;
    Exec::Fault fault = Exec::Fault::None;
};

struct HazardInputs {
    bool exception = false, interrupt = false, memFreeze = false, squash = false, redirect = false;
    bool loadUse = false, matrixBusy = false, serialize = false, longOpBusy = false;
};

struct PipelineControl {
    enum class Reason : uint8_t { None, Exception, Interrupt, MemFreeze, Squash, Redirect, LoadUse, MatrixBusy, Serialize, LongOpBusy };

    Reason reason = Reason::None;
    bool takeTrap = false;
    bool flushAll = false;
    bool freezeAll = false;
    bool squashYounger = false;
    bool redirect = false;
    bool holdFetch = false;
    bool bubbleEx = false;
};

class ControlUnit {
public:
    using Fault = Exec::Fault;

    static ControlWord decode(const ISA::InstructionFields& f) {
        ControlWord w;
        w.valid = true;
        w.opcode = f.opcode;
        w.immediate = f.immediate;
        switch (f.type()) {
            case ISA::InstructionType::ALU:    decodeAlu(f, w); break;
            case ISA::InstructionType::MEMORY: decodeMemory(f, w); break;
            case ISA::InstructionType::BRANCH: decodeBranch(f, w); break;
            case ISA::InstructionType::MATRIX: decodeMatrix(f, w); break;
            case ISA::InstructionType::SYSTEM: decodeSystem(f, w); break;
        }
        return w;
    }

    static ControlWord decodeWord(ISA::InstrWord word, uint64_t pc, bool fetchFault) {
        if (fetchFault) return faulted(Fault::FetchFault, pc, ISA::Opcode::NOP);
        ISA::InstructionFields f;
        try { f = ISA::decode(word); }
        catch (const std::invalid_argument&) { return faulted(Fault::IllegalInstruction, word, ISA::Opcode::NOP); }
        const Fault bad = validate(f);
        if (bad != Fault::None) return faulted(bad, word, f.opcode);
        return decode(f);
    }

    static Fault validate(const ISA::InstructionFields& f) {
        if (f.type() == ISA::InstructionType::MATRIX) {
            auto dimOk = [](unsigned d) { return d >= 1 && d <= ISA::MAX_MATRIX_ROWS; };
            if (!dimOk(f.matrixRows) || !dimOk(f.matrixCols)) return Fault::IllegalInstruction;
            if (f.opcode == ISA::Opcode::MATMUL && !dimOk(f.matrixInner)) return Fault::IllegalInstruction;
            if (f.opcode == ISA::Opcode::MATDETERMINANT && f.matrixRows != f.matrixCols) return Fault::IllegalInstruction;
        }
        if (f.opcode == ISA::Opcode::EXP || f.opcode == ISA::Opcode::POW) return Fault::IllegalInstruction;
        if ((f.opcode == ISA::Opcode::LOAD || f.opcode == ISA::Opcode::STORE) && f.modifier != 0) return Fault::IllegalInstruction;
        if ((f.opcode == ISA::Opcode::RDSR || f.opcode == ISA::Opcode::WRSR) && !ExceptionHandler::validSelect(f.immediate))
            return Fault::IllegalInstruction;
        const ControlWord w = decode(f);
        if (w.regWrite && w.dest == ISA::PC) return Fault::IllegalInstruction;
        return Fault::None;
    }

    static PipelineControl resolve(const HazardInputs& in) {
        PipelineControl c;
        using R = PipelineControl::Reason;
        if (in.exception)      { c.reason = R::Exception;  c.takeTrap = true; c.flushAll = true; }
        else if (in.interrupt) { c.reason = R::Interrupt;  c.takeTrap = true; c.flushAll = true; }
        else if (in.memFreeze) { c.reason = R::MemFreeze;  c.freezeAll = true; }
        else if (in.squash)    { c.reason = R::Squash;     c.squashYounger = true; }
        else if (in.redirect)  { c.reason = R::Redirect;   c.redirect = true; }
        else if (in.matrixBusy){ c.reason = R::MatrixBusy; c.holdFetch = true; c.bubbleEx = true; }
        else if (in.serialize) { c.reason = R::Serialize;  c.holdFetch = true; c.bubbleEx = true; }
        else if (in.loadUse)   { c.reason = R::LoadUse;    c.holdFetch = true; c.bubbleEx = true; }
        else if (in.longOpBusy){ c.reason = R::LongOpBusy; c.holdFetch = true; c.bubbleEx = true; }
        return c;
    }

    static ExMaControl toExMa(const ControlWord& w) {
        ExMaControl e;
        e.valid = w.valid;
        e.regWrite = w.regWrite;
        e.memRead = w.memRead;
        e.memWrite = w.memWrite;
        e.halt = w.halt;
        e.matrixOp = w.matrixOp;
        e.rd = w.dest;
        e.fault = w.fault;
        return e;
    }

    static MaRwControl toMaRw(const ExMaControl& e) {
        MaRwControl m;
        m.valid = e.valid;
        m.regWrite = e.regWrite;
        m.halt = e.halt;
        m.rd = e.rd;
        m.fault = e.fault;
        return m;
    }

private:
    using Op = ISA::Opcode;

    static ControlWord faulted(Fault fault, uint64_t value, Op op) {
        ControlWord w;
        w.valid = true;
        w.opcode = op;
        w.fault = fault;
        w.faultValue = value;
        return w;
    }

    static AluOp aluOpOf(Op op) {
        switch (op) {
            case Op::ADD: return AluOp::ADD;
            case Op::SUB: return AluOp::SUB;
            case Op::MUL: return AluOp::MUL;
            case Op::DIV: return AluOp::DIV;
            case Op::MOD: return AluOp::MOD;
            case Op::AND: return AluOp::AND;
            case Op::OR:  return AluOp::OR;
            case Op::XOR: return AluOp::XOR;
            case Op::NOT: return AluOp::NOT;
            case Op::LSL: return AluOp::LSL;
            case Op::LSR: return AluOp::LSR;
            case Op::ASR: return AluOp::ASR;
            case Op::SIN: return AluOp::SIN;
            case Op::COS: return AluOp::COS;
            case Op::LOG: return AluOp::LOG;
            case Op::CMP: return AluOp::CMP;
            default:      return AluOp::NONE;
        }
    }

    static void decodeAlu(const ISA::InstructionFields& f, ControlWord& w) {
        w.alu = aluOpOf(f.opcode);
        w.longOp = Exec::isLongOp(f.opcode);
        w.readsA = true;
        w.srcA = f.rs1;
        if (!ISA::isUnary(f.opcode)) {
            if (f.modifier) w.immB = true;
            else { w.readsB = true; w.srcB = f.rs2; }
        }
        w.regWrite = true;
        w.dest = f.rd;
        w.result = ResultSel::ALU;
    }

    static void decodeMemory(const ISA::InstructionFields& f, ControlWord& w) {
        w.readsA = true;
        w.srcA = f.rs1;
        if (f.opcode == Op::LOAD) {
            w.memRead = true;
            w.regWrite = true;
            w.dest = f.rd;
            w.result = ResultSel::LOAD;
        } else {
            w.memWrite = true;
            w.readsB = true;
            w.srcB = f.rd;
        }
    }

    static void decodeBranch(const ISA::InstructionFields& f, ControlWord& w) {
        switch (f.opcode) {
            case Op::JEQ: w.branch = BranchKind::JEQ; w.conditionalBranch = true; w.readsA = w.readsB = true; w.srcA = f.rs1; w.srcB = f.rs2; break;
            case Op::JGT: w.branch = BranchKind::JGT; w.conditionalBranch = true; w.readsA = w.readsB = true; w.srcA = f.rs1; w.srcB = f.rs2; break;
            case Op::JZ:  w.branch = BranchKind::JZ;  w.conditionalBranch = true; w.readsA = true; w.srcA = f.rs1; break;
            case Op::JNZ: w.branch = BranchKind::JNZ; w.conditionalBranch = true; w.readsA = true; w.srcA = f.rs1; break;
            case Op::JMP: w.branch = BranchKind::JMP; break;
            case Op::CALL:
                w.branch = BranchKind::CALL;
                w.regWrite = true;
                w.dest = ISA::LR;
                w.result = ResultSel::LINK;
                break;
            case Op::RET:
                w.branch = BranchKind::RET;
                w.readsA = true;
                w.srcA = ISA::LR;
                break;
            default: break;
        }
    }

    static void decodeMatrix(const ISA::InstructionFields& f, ControlWord& w) {
        w.matrixOp = true;
        w.readsC = true;
        w.srcC = f.rd;
        w.readsA = true;
        w.srcA = f.rs1;
        if (f.opcode != Op::MATDETERMINANT) { w.readsB = true; w.srcB = f.rs2; }
        w.matRows = f.matrixRows;
        w.matInner = f.matrixInner;
        w.matCols = f.matrixCols;
    }

    static void decodeSystem(const ISA::InstructionFields& f, ControlWord& w) {
        switch (f.opcode) {
            case Op::HALT:    w.halt = true; break;
            case Op::MATWAIT: w.matWait = true; break;
            case Op::RDSR:
                w.regWrite = true;
                w.dest = f.rd;
                w.result = ResultSel::SPECIAL;
                w.readsSR = true;
                w.serializing = true;
                break;
            case Op::WRSR:
                w.readsA = true;
                w.srcA = f.rs1;
                w.writesSR = true;
                w.serializing = true;
                break;
            case Op::RETI:
                w.branch = BranchKind::RETI;
                w.serializing = true;
                break;
            default: break;
        }
    }
};
