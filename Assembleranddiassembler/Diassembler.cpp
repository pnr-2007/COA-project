
#include "isa.cpp"
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

class Disassembler {
public:
    static std::string disassembleWord(ISA::InstrWord word, uint64_t pc = 0) {
        ISA::InstructionFields f;
        try {
            f = ISA::decode(word);
        } catch (...) {
            return "<illegal_instruction: 0x" + toHex(word, 8) + ">";
        }
        return formatFields(f, pc);
    }

    static std::vector<std::string> disassembleBlock(const std::vector<ISA::InstrWord>& words, uint64_t basePc = 0) {
        std::vector<std::string> lines;
        lines.reserve(words.size());
        for (size_t i = 0; i < words.size(); ++i) {
            uint64_t pc = basePc + i * 4;
            std::ostringstream line;
            line << "0x" << toHex(pc, 8) << ":  "
                 << toHex(words[i], 8) << "    "
                 << disassembleWord(words[i], pc);
            lines.push_back(line.str());
        }
        return lines;
    }

private:
    static std::string regName(uint8_t r, bool fp = false) {
        if (r == 0 && !fp) return "zero";
        if (r == ISA::SP && !fp) return "sp";
        if (r == ISA::LR && !fp) return "lr";
        if (r == ISA::PC && !fp) return "pc";
        return (fp ? "f" : "r") + std::to_string(r);
    }

    static std::string formatFields(const ISA::InstructionFields& f, uint64_t pc) {
        std::string_view m = ISA::mnemonicOf(f.opcode);
        std::ostringstream ss;
        ss << m;

        ISA::InstructionFormat fmt = ISA::formatOf(f.opcode);
        bool isFp = (ISA::typeOf(f.opcode) == ISA::InstructionType::FP) ||
                    (f.opcode == ISA::Opcode::FLOAD) || (f.opcode == ISA::Opcode::FSTORE);

        switch (fmt) {
            case ISA::InstructionFormat::R_TYPE: {
                if (f.opcode == ISA::Opcode::NOP || f.opcode == ISA::Opcode::HALT || f.opcode == ISA::Opcode::RETI) {
                    return ss.str();
                }
                ss << " ";
                if (ISA::isUnary(f.opcode)) {
                    bool destFp = isFp && (f.opcode != ISA::Opcode::FCVTQD && f.opcode != ISA::Opcode::FMVXD);
                    bool srcFp = isFp && (f.opcode != ISA::Opcode::FCVTDQ && f.opcode != ISA::Opcode::FMVDX);
                    ss << regName(f.rd, destFp) << ", " << regName(f.rs1, srcFp);
                } else {
                    bool destFp = isFp && (f.opcode != ISA::Opcode::FCMP);
                    ss << regName(f.rd, destFp) << ", " << regName(f.rs1, isFp) << ", ";
                    if (f.modifier) {
                        ss << f.immediate;
                    } else {
                        ss << regName(f.rs2, isFp);
                    }
                }
                break;
            }

            case ISA::InstructionFormat::I_TYPE: {
                ss << " ";
                if (f.opcode == ISA::Opcode::LOAD || f.opcode == ISA::Opcode::FLOAD ||
                    f.opcode == ISA::Opcode::STORE || f.opcode == ISA::Opcode::FSTORE) {
                    bool dataFp = (f.opcode == ISA::Opcode::FLOAD || f.opcode == ISA::Opcode::FSTORE);
                    ss << regName(f.rd, dataFp) << ", ["
                       << regName(f.rs1, false);
                    if (f.immediate != 0) ss << ", " << f.immediate;
                    ss << "]";
                } else if (f.opcode == ISA::Opcode::RDSR) {
                    ss << regName(f.rd, false) << ", " << f.immediate;
                } else if (f.opcode == ISA::Opcode::WRSR) {
                    ss << regName(f.rs1, false) << ", " << f.immediate;
                }
                break;
            }

            case ISA::InstructionFormat::B_TYPE: {
                ss << " ";
                if (f.opcode == ISA::Opcode::JZ || f.opcode == ISA::Opcode::JNZ) {
                    ss << regName(f.rs1, false) << ", ";
                } else {
                    ss << regName(f.rs1, false) << ", " << regName(f.rs2, false) << ", ";
                }
                uint64_t target = pc + static_cast<int64_t>(f.immediate) * 4;
                ss << "0x" << toHex(target, 0);
                break;
            }

            case ISA::InstructionFormat::J_TYPE: {
                if (f.opcode != ISA::Opcode::RET) {
                    ss << " ";
                    uint64_t target = pc + static_cast<int64_t>(f.immediate) * 4;
                    ss << "0x" << toHex(target, 0);
                }
                break;
            }
        }
        return ss.str();
    }

    static std::string toHex(uint64_t val, int width) {
        std::ostringstream ss;
        ss << std::hex << std::uppercase;
        if (width > 0) ss << std::setfill('0') << std::setw(width);
        ss << val;
        return ss.str();
    }
};
