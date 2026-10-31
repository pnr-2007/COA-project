#pragma once

#include "alu.cpp"

class Exec {
public:
    using Word   = ISA::Word;
    using Opcode = ISA::Opcode;

    enum class Fault : uint8_t { None = 0, IllegalInstruction = 1, MemoryFault = 2, FetchFault = 3 };

    struct RegUse {
        bool readsA = false; uint8_t a = 0;
        bool readsB = false; uint8_t b = 0;
        bool writes = false; uint8_t dest = 0;
    };

    static RegUse regUse(const ISA::InstructionFields& f) {
        RegUse u;
        switch (f.type()) {
            case ISA::InstructionType::ALU:
                u.readsA = true; u.a = f.rs1;
                if (!ISA::isUnary(f.opcode) && f.modifier == 0) { u.readsB = true; u.b = f.rs2; }
                u.writes = true; u.dest = f.rd;
                break;
            case ISA::InstructionType::MEMORY:
                u.readsA = true; u.a = f.rs1;
                if (f.opcode == Opcode::LOAD) { u.writes = true; u.dest = f.rd; }
                else                          { u.readsB = true; u.b = f.rd; }
                break;
            case ISA::InstructionType::BRANCH:
                switch (f.opcode) {
                    case Opcode::JEQ:
                    case Opcode::JGT:  u.readsA = true; u.a = f.rs1; u.readsB = true; u.b = f.rs2; break;
                    case Opcode::JZ:
                    case Opcode::JNZ:  u.readsA = true; u.a = f.rs1; break;
                    case Opcode::CALL: u.writes = true; u.dest = ISA::LR; break;
                    case Opcode::RET:  u.readsA = true; u.a = ISA::LR; break;
                    default: break;
                }
                break;
            default: break;
        }
        return u;
    }

    static Fault validate(const ISA::InstructionFields& f) {
        if (f.type() == ISA::InstructionType::MATRIX) return Fault::IllegalInstruction;
        if (f.opcode == Opcode::EXP || f.opcode == Opcode::POW) return Fault::IllegalInstruction;
        if ((f.opcode == Opcode::LOAD || f.opcode == Opcode::STORE) && f.modifier != 0) return Fault::IllegalInstruction;
        const RegUse u = regUse(f);
        if (u.writes && u.dest == ISA::PC) return Fault::IllegalInstruction;
        return Fault::None;
    }

    static constexpr bool isLongOp(Opcode op) {
        return op == Opcode::MUL || op == Opcode::DIV || op == Opcode::MOD ||
               op == Opcode::SIN || op == Opcode::COS || op == Opcode::LOG;
    }

    static bool alu(Opcode op, Word a, Word b, Word& out) {
        bool ovf = false;
        switch (op) {
            case Opcode::ADD: out = ALU::add(a, b).value; return true;
            case Opcode::SUB: out = ALU::sub(a, b).value; return true;
            case Opcode::MUL: out = ALU::fixedMul(a, b, ovf); return true;
            case Opcode::DIV: out = ALU::fixedDiv(a, b, ovf); return true;
            case Opcode::MOD: {
                auto neg  = [](Word w) { return static_cast<int64_t>(w) < 0; };
                auto absw = [&](Word w) { return neg(w) ? Word(0) - w : w; };
                const Word rem = ALU::modulus(absw(a), absw(b)).value;
                out = neg(a) ? Word(0) - rem : rem;
                return true;
            }
            case Opcode::AND: out = ALU::bitwiseAnd(a, b).value; return true;
            case Opcode::OR:  out = ALU::bitwiseOr(a, b).value;  return true;
            case Opcode::XOR: out = ALU::bitwiseXor(a, b).value; return true;
            case Opcode::NOT: out = ALU::bitwiseNot(a).value;    return true;
            case Opcode::LSL: out = ALU::lsl(a, b).value;        return true;
            case Opcode::LSR: out = ALU::lsr(a, b).value;        return true;
            case Opcode::ASR: out = ALU::asr(a, b).value;        return true;
            case Opcode::SIN: out = ALU::evaluateSineSigned(a).value;   return true;
            case Opcode::COS: out = ALU::evaluateCosineSigned(a).value; return true;
            case Opcode::LOG: out = ALU::evaluateLog2(a).value;         return true;
            case Opcode::CMP: {
                const ALU::Result d = ALU::sub(a, b);
                const bool less = d.negative != d.overflow;
                out = d.zero ? Word(0) : (less ? ~Word(0) : Word(1));
                return true;
            }
            default: return false;
        }
    }

    static bool branchTaken(Opcode op, Word a, Word b) {
        switch (op) {
            case Opcode::JEQ: return a == b;
            case Opcode::JGT: return static_cast<int64_t>(a) > static_cast<int64_t>(b);
            case Opcode::JZ:  return a == 0;
            case Opcode::JNZ: return a != 0;
            case Opcode::JMP:
            case Opcode::CALL:
            case Opcode::RET: return true;
            default:          return false;
        }
    }
};
