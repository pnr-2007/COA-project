#pragma once

// Operand syntax of every RISC201 instruction, shared by the assembler and disassembler
// so the two can never disagree.

#include "isa.cpp"
#include "exception_handler.cpp"
#include <cctype>
#include <string>

struct AsmSyntax {
    enum class Form : uint8_t {
        NONE,     // NOP HALT RETI RET
        D_S,      // op rd, rs1                 (unary ALU, FSQRT, converts, moves)
        D_S_T,    // op rd, rs1, rs2|imm        (binary ALU; FP forms take no immediate)
        MEM,      // op rd, imm(rs1)            (LOAD STORE FLOAD FSTORE)
        B_S_T,    // op rs1, rs2, target        (JEQ JGT)
        B_S,      // op rs1, target             (JZ JNZ)
        J,        // op target                  (JMP CALL)
        RDSR,     // RDSR rd, sel
        WRSR      // WRSR sel, rs1
    };
    Form form = Form::NONE;
    bool fpD = false, fpS = false, fpT = false;   // register class of each operand
    bool allowImm = false;                         // D_S_T only
};

inline AsmSyntax syntaxOf(ISA::Opcode op) {
    using O = ISA::Opcode;
    using F = AsmSyntax::Form;
    AsmSyntax s;
    switch (op) {
        case O::LOAD: case O::STORE:   s.form = F::MEM; break;
        case O::FLOAD: case O::FSTORE: s.form = F::MEM; s.fpD = true; break;
        case O::JEQ: case O::JGT:      s.form = F::B_S_T; break;
        case O::JZ: case O::JNZ:       s.form = F::B_S; break;
        case O::JMP: case O::CALL:     s.form = F::J; break;
        case O::RET: case O::NOP: case O::HALT: case O::RETI: s.form = F::NONE; break;
        case O::RDSR:                  s.form = F::RDSR; break;
        case O::WRSR:                  s.form = F::WRSR; break;
        case O::FADD: case O::FSUB: case O::FMUL: case O::FDIV:
            s.form = F::D_S_T; s.fpD = s.fpS = s.fpT = true; break;
        case O::FSQRT:  s.form = F::D_S; s.fpD = s.fpS = true; break;
        case O::FCMP:   s.form = F::D_S_T; s.fpS = s.fpT = true; break;
        case O::FCVTDQ: case O::FMVDX: s.form = F::D_S; s.fpD = true; break;
        case O::FCVTQD: case O::FMVXD: s.form = F::D_S; s.fpS = true; break;
        default:
            if (ISA::isUnary(op)) s.form = F::D_S;
            else { s.form = F::D_S_T; s.allowImm = true; }
    }
    return s;
}

inline const char* specialRegName(unsigned sel) {
    static const char* n[] = {"STATUS", "EPC", "CAUSE", "TVEC", "IMASK", "IPEND", "BADADDR", "FCSR"};
    return sel < 8 ? n[sel] : "?";
}

inline std::string upperStr(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
