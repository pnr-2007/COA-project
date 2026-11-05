#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

using namespace std;

class ISA {
public:
    static constexpr uint8_t OPCODE_BITS     = 6;
    static constexpr uint8_t REGISTER_BITS   = 4;
    static constexpr uint8_t REGISTER_COUNT  = 16;

    using InstrWord    = uint32_t;
    using Word         = uint64_t;
    using RegisterFile = std::array<Word, REGISTER_COUNT>;

    static constexpr uint8_t ZERO = 0;
    static constexpr uint8_t SP   = 13;
    static constexpr uint8_t LR   = 14;
    static constexpr uint8_t PC   = 15;

    enum class InstructionType   : uint8_t { ALU, MEMORY, BRANCH, SYSTEM, FP };
    enum class InstructionFormat : uint8_t { R_TYPE, I_TYPE, B_TYPE, J_TYPE };

    enum class Opcode : uint8_t {
        ADD = 0, SUB, MUL, DIV, MOD, AND, OR, XOR, NOT, LSL, LSR, ASR,
        SIN, COS, LOG, EXP, POW, CMP,
        LOAD, STORE,
        JEQ, JGT, JZ, JNZ, JMP, CALL, RET,
        NOP, HALT, RDSR, WRSR, RETI,
        FADD, FSUB, FMUL, FDIV, FSQRT, FCMP, FCVTDQ, FCVTQD, FMVDX, FMVXD, FLOAD, FSTORE,
        OPCODE_COUNT
    };

    struct OpcodeMeta {
        InstructionFormat format;
        InstructionType   type;
        std::string_view  mnemonic;
        bool              isUnary;
    };

    static constexpr uint8_t opcodeValue(Opcode op) { return static_cast<uint8_t>(op); }
    static constexpr bool isValidOpcode(uint8_t val) { return val < opcodeValue(Opcode::OPCODE_COUNT); }

    static constexpr OpcodeMeta getMeta(Opcode op) {
        if (!isValidOpcode(opcodeValue(op))) {
            throw std::invalid_argument("ISA: invalid opcode");
        }
        constexpr OpcodeMeta table[] = {
            // ALU
            {InstructionFormat::R_TYPE, InstructionType::ALU, "ADD", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "SUB", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "MUL", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "DIV", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "MOD", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "AND", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "OR",  false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "XOR", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "NOT", true},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "LSL", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "LSR", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "ASR", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "SIN", true},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "COS", true},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "LOG", true},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "EXP", true},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "POW", false},
            {InstructionFormat::R_TYPE, InstructionType::ALU, "CMP", false},
            // MEMORY
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "LOAD", false},
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "STORE", false},
            // BRANCH
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JEQ", false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JGT", false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JZ",  false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JNZ", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "JMP", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "CALL", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "RET", false},
            // SYSTEM
            {InstructionFormat::R_TYPE, InstructionType::SYSTEM, "NOP", false},
            {InstructionFormat::R_TYPE, InstructionType::SYSTEM, "HALT", false},
            {InstructionFormat::I_TYPE, InstructionType::SYSTEM, "RDSR", false},
            {InstructionFormat::I_TYPE, InstructionType::SYSTEM, "WRSR", false},
            {InstructionFormat::R_TYPE, InstructionType::SYSTEM, "RETI", false},
            // FP
            {InstructionFormat::R_TYPE, InstructionType::FP, "FADD", false},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FSUB", false},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FMUL", false},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FDIV", false},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FSQRT", true},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FCMP", false},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FCVTDQ", true},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FCVTQD", true},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FMVDX", true},
            {InstructionFormat::R_TYPE, InstructionType::FP, "FMVXD", true},
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "FLOAD", false},
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "FSTORE", false}
        };
        return table[opcodeValue(op)];
    }

    static constexpr InstructionFormat formatOf(Opcode op) { return getMeta(op).format; }
    static constexpr InstructionType   typeOf(Opcode op)   { return getMeta(op).type; }
    static constexpr std::string_view  mnemonicOf(Opcode op) { return getMeta(op).mnemonic; }
    static constexpr bool              isUnary(Opcode op)  { return getMeta(op).isUnary; }

    static Opcode opcodeFromMnemonic(std::string_view mnemonic) {
        for (uint8_t i = 0; i < opcodeValue(Opcode::OPCODE_COUNT); ++i) {
            Opcode op = static_cast<Opcode>(i);
            if (mnemonicOf(op) == mnemonic) return op;
        }
        throw std::invalid_argument("ISA::opcodeFromMnemonic: unknown mnemonic '" + std::string(mnemonic) + "'");
    }

    struct InstructionFields {
        Opcode   opcode     = Opcode::NOP;
        uint8_t  rd         = 0;
        uint8_t  rs1        = 0;
        uint8_t  rs2        = 0;
        int32_t  immediate  = 0;
        uint8_t  modifier   = 0;

        InstructionFormat format() const { return ISA::formatOf(opcode); }
        InstructionType   type()   const { return ISA::typeOf(opcode); }
    };

    static InstrWord encode(const InstructionFields& f) {
        if (!isValidOpcode(opcodeValue(f.opcode))) {
            throw std::invalid_argument("ISA::encode: invalid opcode");
        }

        InstrWord word = static_cast<InstrWord>(f.opcode) << 26;

        switch (formatOf(f.opcode)) {
            case InstructionFormat::R_TYPE:
                word |= (f.rd & 0xF) << 22 | (f.rs1 & 0xF) << 18;
                if (!isUnary(f.opcode)) {
                    if (f.modifier) word |= (1u << 17) | (f.immediate & 0x1FFFF);
                    else            word |= (f.rs2 & 0xF) << 13;
                }
                break;
            case InstructionFormat::I_TYPE:
                word |= (f.rd & 0xF) << 22 | (f.rs1 & 0xF) << 18 | (f.modifier & 0x3) << 16 | (f.immediate & 0xFFFF);
                break;
            case InstructionFormat::B_TYPE:
                word |= (f.rs1 & 0xF) << 22 | (f.rs2 & 0xF) << 18 | (f.immediate & 0x3FFFF);
                break;
            case InstructionFormat::J_TYPE:
                word |= f.immediate & 0x3FFFFFF;
                break;
        }
        return word;
    }

    static InstructionFields decode(InstrWord word) {
        uint8_t opVal = static_cast<uint8_t>((word >> 26) & 0x3F);
        if (!isValidOpcode(opVal)) {
            throw std::invalid_argument("ISA::decode: invalid opcode in instruction word");
        }

        InstructionFields f;
        f.opcode = static_cast<Opcode>(opVal);

        switch (formatOf(f.opcode)) {
            case InstructionFormat::R_TYPE:
                f.rd  = (word >> 22) & 0xF;
                f.rs1 = (word >> 18) & 0xF;
                if (!isUnary(f.opcode)) {
                    f.modifier = (word >> 17) & 0x1;
                    if (f.modifier) f.immediate = signExtend(word & 0x1FFFF, 17);
                    else            f.rs2 = (word >> 13) & 0xF;
                }
                break;

                
            case InstructionFormat::I_TYPE:
                f.rd        = (word >> 22) & 0xF;
                f.rs1       = (word >> 18) & 0xF;
                f.modifier  = (word >> 16) & 0x3;
                f.immediate = signExtend(word & 0xFFFF, 16);
                break;
            case InstructionFormat::B_TYPE:
                f.rs1       = (word >> 22) & 0xF;
                f.rs2       = (word >> 18) & 0xF;
                f.immediate = signExtend(word & 0x3FFFF, 18);
                break;
            case InstructionFormat::J_TYPE:
                f.immediate = signExtend(word & 0x3FFFFFF, 26);
                break;
        }
        return f;
    }

private:
    static constexpr int32_t signExtend(uint32_t value, uint8_t bits) {
        uint32_t signBit = 1u << (bits - 1);
        return static_cast<int32_t>((value ^ signBit) - signBit);
    }
};

static_assert((1u << ISA::REGISTER_BITS) == ISA::REGISTER_COUNT, "");
static_assert((1u << ISA::OPCODE_BITS) >= ISA::opcodeValue(ISA::Opcode::OPCODE_COUNT), "");



