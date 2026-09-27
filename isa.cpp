#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <map>
#include <string>

class ISA {
public:
    static constexpr uint8_t OPCODE_BITS     = 6;
    static constexpr uint8_t REGISTER_BITS   = 4;
    static constexpr uint8_t REGISTER_COUNT  = 16;
    static constexpr uint8_t MATRIX_DIM_BITS = 5;
    static constexpr uint8_t MAX_MATRIX_ROWS = 20;
    static constexpr uint8_t MAX_MATRIX_COLS = 20;

    using InstrWord    = uint32_t;
    using Word         = uint64_t;
    using RegisterFile = std::array<Word, REGISTER_COUNT>;

    static constexpr uint8_t ZERO = 0;
    static constexpr uint8_t SP   = 13;
    static constexpr uint8_t LR   = 14;
    static constexpr uint8_t PC   = 15;

    enum class InstructionType : uint8_t {
        ALU,
        MATRIX,
        MEMORY,
        BRANCH,
        SYSTEM
    };

    enum class InstructionFormat : uint8_t {
        R_TYPE,
        I_TYPE,
        B_TYPE,
        J_TYPE,
        M_TYPE
    };

    enum class Opcode : uint8_t {
        ADD = 0, SUB, MUL, DIV, MOD, AND, OR, XOR, NOT, LSL, LSR, ASR,
        SIN, COS, LOG, EXP, POW,

        MATMUL, MATADD, MATTRANSPOSE, MATINVERSE, MATADJOINT,
        MATDETERMINANT,

        LOAD, STORE, ATOMIC_MEMORY_OPS, SEQUENTIAL_MEMORY_OPS,
        STRING_MEMORY_OPS,

        JEQ, JGT, JZ, JNZ, LOOPZ, LOOPNZ, JMP, CALL, RET,

        NOP, HALT,

        OPCODE_COUNT
    };

    static constexpr uint8_t opcodeValue(Opcode opcode) {
        return static_cast<uint8_t>(opcode);
    }

    static constexpr bool isValidOpcode(uint8_t value) {
        return value < opcodeValue(Opcode::OPCODE_COUNT);
    }

    static InstructionFormat formatOf(Opcode op) {
        switch (op) {
            case Opcode::ADD: case Opcode::SUB: case Opcode::MUL: case Opcode::DIV:
            case Opcode::MOD: case Opcode::AND: case Opcode::OR:  case Opcode::XOR:
            case Opcode::NOT: case Opcode::LSL: case Opcode::LSR: case Opcode::ASR:
            case Opcode::SIN: case Opcode::COS: case Opcode::LOG: case Opcode::EXP:
            case Opcode::POW:
            case Opcode::NOP: case Opcode::HALT:
                return InstructionFormat::R_TYPE;

            case Opcode::LOAD: case Opcode::STORE:
            case Opcode::ATOMIC_MEMORY_OPS: case Opcode::SEQUENTIAL_MEMORY_OPS:
            case Opcode::STRING_MEMORY_OPS:
                return InstructionFormat::I_TYPE;

            case Opcode::JEQ: case Opcode::JGT: case Opcode::JZ: case Opcode::JNZ:
            case Opcode::LOOPZ: case Opcode::LOOPNZ:
                return InstructionFormat::B_TYPE;

            case Opcode::JMP: case Opcode::CALL: case Opcode::RET:
                return InstructionFormat::J_TYPE;

            case Opcode::MATMUL: case Opcode::MATADD: case Opcode::MATTRANSPOSE:
            case Opcode::MATINVERSE: case Opcode::MATADJOINT:
            case Opcode::MATDETERMINANT:
                return InstructionFormat::M_TYPE;

            default:
                throw std::invalid_argument("ISA::formatOf: unknown opcode");
        }
    }

    static InstructionType typeOf(Opcode op) {
        switch (op) {
            case Opcode::ADD: case Opcode::SUB: case Opcode::MUL: case Opcode::DIV:
            case Opcode::MOD: case Opcode::AND: case Opcode::OR:  case Opcode::XOR:
            case Opcode::NOT: case Opcode::LSL: case Opcode::LSR: case Opcode::ASR:
            case Opcode::SIN: case Opcode::COS: case Opcode::LOG: case Opcode::EXP:
            case Opcode::POW:
                return InstructionType::ALU;

            case Opcode::MATMUL: case Opcode::MATADD: case Opcode::MATTRANSPOSE:
            case Opcode::MATINVERSE: case Opcode::MATADJOINT:
            case Opcode::MATDETERMINANT:
                return InstructionType::MATRIX;

            case Opcode::LOAD: case Opcode::STORE:
            case Opcode::ATOMIC_MEMORY_OPS: case Opcode::SEQUENTIAL_MEMORY_OPS:
            case Opcode::STRING_MEMORY_OPS:
                return InstructionType::MEMORY;

            case Opcode::JEQ: case Opcode::JGT: case Opcode::JZ: case Opcode::JNZ:
            case Opcode::LOOPZ: case Opcode::LOOPNZ:
            case Opcode::JMP: case Opcode::CALL: case Opcode::RET:
                return InstructionType::BRANCH;

            case Opcode::NOP: case Opcode::HALT:
                return InstructionType::SYSTEM;

            default:
                throw std::invalid_argument("ISA::typeOf: unknown opcode");
        }
    }

    static std::string mnemonicOf(Opcode op) {
        switch (op) {
            case Opcode::ADD: return "ADD";   case Opcode::SUB: return "SUB";
            case Opcode::MUL: return "MUL";   case Opcode::DIV: return "DIV";
            case Opcode::MOD: return "MOD";   case Opcode::AND: return "AND";
            case Opcode::OR:  return "OR";    case Opcode::XOR: return "XOR";
            case Opcode::NOT: return "NOT";   case Opcode::LSL: return "LSL";
            case Opcode::LSR: return "LSR";   case Opcode::ASR: return "ASR";
            case Opcode::SIN: return "SIN";   case Opcode::COS: return "COS";
            case Opcode::LOG: return "LOG";   case Opcode::EXP: return "EXP";
            case Opcode::POW: return "POW";

            case Opcode::MATMUL: return "MATMUL";
            case Opcode::MATADD: return "MATADD";
            case Opcode::MATTRANSPOSE: return "MATTRANSPOSE";
            case Opcode::MATINVERSE: return "MATINVERSE";
            case Opcode::MATADJOINT: return "MATADJOINT";
            case Opcode::MATDETERMINANT: return "MATDETERMINANT";

            case Opcode::LOAD: return "LOAD";   case Opcode::STORE: return "STORE";
            case Opcode::ATOMIC_MEMORY_OPS: return "ATOMIC_MEMORY_OPS";
            case Opcode::SEQUENTIAL_MEMORY_OPS: return "SEQUENTIAL_MEMORY_OPS";
            case Opcode::STRING_MEMORY_OPS: return "STRING_MEMORY_OPS";

            case Opcode::JEQ: return "JEQ";     case Opcode::JGT: return "JGT";
            case Opcode::JZ:  return "JZ";      case Opcode::JNZ: return "JNZ";
            case Opcode::LOOPZ: return "LOOPZ"; case Opcode::LOOPNZ: return "LOOPNZ";
            case Opcode::JMP: return "JMP";     case Opcode::CALL: return "CALL";
            case Opcode::RET: return "RET";

            case Opcode::NOP:  return "NOP";
            case Opcode::HALT: return "HALT";

            default:
                throw std::invalid_argument("ISA::mnemonicOf: unknown opcode");
        }
    }

    static Opcode opcodeFromMnemonic(const std::string& mnemonic) {
        static const std::map<std::string, Opcode> table = buildMnemonicTable();
        auto it = table.find(mnemonic);
        if (it == table.end()) {
            throw std::invalid_argument("ISA::opcodeFromMnemonic: unknown mnemonic '" + mnemonic + "'");
        }
        return it->second;
    }

    struct InstructionFields {
        Opcode  opcode     = Opcode::NOP;
        uint8_t rd         = 0;
        uint8_t rs1        = 0;
        uint8_t rs2        = 0;
        int32_t immediate  = 0;
        uint8_t modifier   = 0;
        uint8_t matrixRows = 0;
        uint8_t matrixCols = 0;

        InstructionFormat format() const { return ISA::formatOf(opcode); }
        InstructionType   type()   const { return ISA::typeOf(opcode); }
    };

    static InstrWord encode(const InstructionFields& f) {
        if (!isValidOpcode(opcodeValue(f.opcode))) {
            throw std::invalid_argument("ISA::encode: invalid opcode");
        }
        InstrWord word = static_cast<InstrWord>(opcodeValue(f.opcode)) << 26;

        switch (formatOf(f.opcode)) {
            case InstructionFormat::R_TYPE: {
                word |= static_cast<InstrWord>(f.rd  & 0xF) << 22;
                word |= static_cast<InstrWord>(f.rs1 & 0xF) << 18;
                uint8_t mod = f.modifier ? 1 : 0;
                word |= static_cast<InstrWord>(mod) << 17;
                if (mod) {
                    word |= static_cast<InstrWord>(f.immediate) & 0x1FFFF;
                } else {
                    word |= static_cast<InstrWord>(f.rs2 & 0xF) << 13;
                }
                break;
            }
            case InstructionFormat::I_TYPE: {
                word |= static_cast<InstrWord>(f.rd  & 0xF) << 22;
                word |= static_cast<InstrWord>(f.rs1 & 0xF) << 18;
                word |= static_cast<InstrWord>(f.modifier & 0x3) << 16;
                word |= static_cast<InstrWord>(f.immediate) & 0xFFFF;
                break;
            }
            case InstructionFormat::B_TYPE: {
                word |= static_cast<InstrWord>(f.rs1 & 0xF) << 22;
                word |= static_cast<InstrWord>(f.rs2 & 0xF) << 18;
                word |= static_cast<InstrWord>(f.immediate) & 0x3FFFF;
                break;
            }
            case InstructionFormat::J_TYPE: {
                word |= static_cast<InstrWord>(f.immediate) & 0x3FFFFFF;
                break;
            }
            case InstructionFormat::M_TYPE: {
                word |= static_cast<InstrWord>(f.rd  & 0xF) << 22;
                word |= static_cast<InstrWord>(f.rs1 & 0xF) << 18;
                word |= static_cast<InstrWord>(f.rs2 & 0xF) << 14;
                word |= static_cast<InstrWord>(f.matrixRows & 0x1F) << 9;
                word |= static_cast<InstrWord>(f.matrixCols & 0x1F) << 4;
                break;
            }
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
            case InstructionFormat::R_TYPE: {
                f.rd  = (word >> 22) & 0xF;
                f.rs1 = (word >> 18) & 0xF;
                f.modifier = (word >> 17) & 0x1;
                if (f.modifier) {
                    f.immediate = signExtend(word & 0x1FFFF, 17);
                } else {
                    f.rs2 = (word >> 13) & 0xF;
                }
                break;
            }
            case InstructionFormat::I_TYPE: {
                f.rd  = (word >> 22) & 0xF;
                f.rs1 = (word >> 18) & 0xF;
                f.modifier = (word >> 16) & 0x3;
                f.immediate = signExtend(word & 0xFFFF, 16);
                break;
            }
            case InstructionFormat::B_TYPE: {
                f.rs1 = (word >> 22) & 0xF;
                f.rs2 = (word >> 18) & 0xF;
                f.immediate = signExtend(word & 0x3FFFF, 18);
                break;
            }
            case InstructionFormat::J_TYPE: {
                f.immediate = signExtend(word & 0x3FFFFFF, 26);
                break;
            }
            case InstructionFormat::M_TYPE: {
                f.rd  = (word >> 22) & 0xF;
                f.rs1 = (word >> 18) & 0xF;
                f.rs2 = (word >> 14) & 0xF;
                f.matrixRows = (word >> 9) & 0x1F;
                f.matrixCols = (word >> 4) & 0x1F;
                break;
            }
        }
        return f;
    }

private:
    static int32_t signExtend(uint32_t value, uint8_t bits) {
        uint32_t signBit = 1u << (bits - 1);
        return static_cast<int32_t>((value ^ signBit) - signBit);
    }

    static std::map<std::string, Opcode> buildMnemonicTable() {
        std::map<std::string, Opcode> table;
        for (uint8_t v = 0; v < opcodeValue(Opcode::OPCODE_COUNT); ++v) {
            Opcode op = static_cast<Opcode>(v);
            table[mnemonicOf(op)] = op;
        }
        return table;
    }
};

static_assert((1u << ISA::REGISTER_BITS) == ISA::REGISTER_COUNT, "");
static_assert((1u << ISA::MATRIX_DIM_BITS) - 1 >= ISA::MAX_MATRIX_ROWS, "");
static_assert((1u << ISA::MATRIX_DIM_BITS) - 1 >= ISA::MAX_MATRIX_COLS, "");
static_assert((1u << ISA::OPCODE_BITS) >= ISA::opcodeValue(ISA::Opcode::OPCODE_COUNT), "");

