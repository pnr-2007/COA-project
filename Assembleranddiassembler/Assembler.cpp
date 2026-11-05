#include "isa.cpp"
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

class Assembler {
public:
    struct AssemblyResult {
        std::vector<ISA::InstrWord> words;
        std::vector<std::string> errors;
        bool success = false;
    };

    static AssemblyResult assemble(const std::string& source) {
        AssemblyResult result;
        std::unordered_map<std::string, uint64_t> labels;
        std::vector<ParsedLine> parsedLines;

        std::istringstream stream(source);
        std::string line;
        uint64_t currentPc = 0;

        while (std::getline(stream, line)) {
            std::string cleaned = stripComments(line);
            cleaned = trim(cleaned);
            if (cleaned.empty()) continue;

            size_t colonPos = cleaned.find(':');
            while (colonPos != std::string::npos) {
                std::string labelName = trim(cleaned.substr(0, colonPos));
                if (labelName.empty() || !isValidLabelName(labelName)) {
                    result.errors.push_back("Invalid label declaration: " + labelName);
                    return result;
                }
                if (labels.find(labelName) != labels.end()) {
                    result.errors.push_back("Duplicate label declaration: " + labelName);
                    return result;
                }
                labels[labelName] = currentPc;
                cleaned = trim(cleaned.substr(colonPos + 1));
                colonPos = cleaned.find(':');
            }

            if (cleaned.empty()) continue;

            ParsedLine pl;
            pl.raw = cleaned;
            pl.pc = currentPc;
            tokenize(cleaned, pl.mnemonic, pl.operands);
            parsedLines.push_back(pl);
            currentPc += 4;
        }

        for (const auto& pl : parsedLines) {
            try {
                ISA::InstructionFields fields = parseInstruction(pl, labels);
                result.words.push_back(ISA::encode(fields));
            } catch (const std::exception& e) {
                result.errors.push_back("Error at PC 0x" + toHex(pl.pc) + ": " + e.what());
            }
        }

        result.success = result.errors.empty();
        return result;
    }

private:
    struct ParsedLine {
        uint64_t pc = 0;
        std::string raw;
        std::string mnemonic;
        std::vector<std::string> operands;
    };

    static std::string stripComments(const std::string& s) {
        size_t p = s.find_first_of(";#");
        if (p != std::string::npos) return s.substr(0, p);
        return s;
    }

    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    static bool isValidLabelName(const std::string& s) {
        if (s.empty() || (!std::isalpha(s[0]) && s[0] != '_')) return false;
        for (char c : s) {
            if (!std::isalnum(c) && c != '_') return false;
        }
        return true;
    }

    static void tokenize(const std::string& line, std::string& mnemonic, std::vector<std::string>& ops) {
        size_t firstSpace = line.find_first_of(" \t");
        if (firstSpace == std::string::npos) {
            mnemonic = toUpper(line);
            return;
        }

        mnemonic = toUpper(line.substr(0, firstSpace));
        std::string rest = line.substr(firstSpace + 1);

        std::string cur;
        bool inBracket = false;
        for (char c : rest) {
            if (c == '[') inBracket = true;
            else if (c == ']') inBracket = false;

            if (c == ',' && !inBracket) {
                std::string t = trim(cur);
                if (!t.empty()) ops.push_back(t);
                cur.clear();
            } else {
                cur += c;
            }
        }
        std::string t = trim(cur);
        if (!t.empty()) ops.push_back(t);
    }

    static std::string toUpper(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
        return s;
    }

    static uint8_t parseRegister(const std::string& s) {
        std::string u = toUpper(trim(s));
        if (u == "ZERO" || u == "R0" || u == "F0") return 0;
        if (u == "SP" || u == "R13" || u == "F13") return ISA::SP;
        if (u == "LR" || u == "R14" || u == "F14") return ISA::LR;
        if (u == "PC" || u == "R15" || u == "F15") return ISA::PC;

        if (u.size() >= 2 && (u[0] == 'R' || u[0] == 'F')) {
            size_t idx = 0;
            int num = std::stoi(u.substr(1), &idx);
            if (idx == u.size() - 1 && num >= 0 && num < ISA::REGISTER_COUNT) {
                return static_cast<uint8_t>(num);
            }
        }
        throw std::invalid_argument("Invalid register: " + s);
    }

    static int64_t parseImmediateOrLabel(const std::string& s, uint64_t pc, const std::unordered_map<std::string, uint64_t>& labels, bool relative) {
        std::string t = trim(s);
        auto it = labels.find(t);
        if (it != labels.end()) {
            if (relative) {
                int64_t diff = static_cast<int64_t>(it->second) - static_cast<int64_t>(pc);
                if (diff % 4 != 0) throw std::invalid_argument("Target address not 4-byte aligned");
                return diff / 4;
            }
            if (it->second % 4 != 0) throw std::invalid_argument("Target address not 4-byte aligned");
            return static_cast<int64_t>(it->second) / 4;
        }

        size_t idx = 0;
        int base = 10;
        if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) base = 16;
        else if (t.rfind("0b", 0) == 0 || t.rfind("0B", 0) == 0) {
            t = t.substr(2);
            base = 2;
        }

        return std::stoll(t, &idx, base);
    }

    static void checkRange(int64_t val, int bits, const std::string& ctx) {
        int64_t minVal = -(1LL << (bits - 1));
        int64_t maxVal = (1LL << (bits - 1)) - 1;
        if (val < minVal || val > maxVal) {
            throw std::out_of_range("Immediate out of range for " + ctx);
        }
    }

    static void parseMemoryOperand(const std::string& s, uint8_t& baseReg, int32_t& offset, uint64_t pc, const std::unordered_map<std::string, uint64_t>& labels) {
        std::string t = trim(s);
        size_t ob = t.find('[');
        size_t cb = t.find(']');
        if (ob == std::string::npos || cb == std::string::npos || ob >= cb) {
            throw std::invalid_argument("Malformed memory operand: " + s);
        }

        std::string inside = trim(t.substr(ob + 1, cb - ob - 1));
        std::string pre = trim(t.substr(0, ob));

        size_t comma = inside.find(',');
        if (comma != std::string::npos) {
            baseReg = parseRegister(inside.substr(0, comma));
            int64_t offVal = parseImmediateOrLabel(inside.substr(comma + 1), pc, labels, false);
            checkRange(offVal, 16, "memory offset");
            offset = static_cast<int32_t>(offVal);
        } else {
            baseReg = parseRegister(inside);
            if (!pre.empty()) {
                int64_t offVal = parseImmediateOrLabel(pre, pc, labels, false);
                checkRange(offVal, 16, "memory offset");
                offset = static_cast<int32_t>(offVal);
            } else {
                offset = 0;
            }
        }
    }

    static ISA::InstructionFields parseInstruction(const ParsedLine& pl, const std::unordered_map<std::string, uint64_t>& labels) {
        ISA::InstructionFields f;
        f.opcode = ISA::opcodeFromMnemonic(pl.mnemonic);
        ISA::InstructionFormat fmt = ISA::formatOf(f.opcode);

        switch (fmt) {
            case ISA::InstructionFormat::R_TYPE: {
                if (f.opcode == ISA::Opcode::NOP || f.opcode == ISA::Opcode::HALT || f.opcode == ISA::Opcode::RETI) {
                    return f;
                }
                if (ISA::isUnary(f.opcode)) {
                    if (pl.operands.size() < 2) throw std::invalid_argument("Unary requires 2 operands (rd, rs1)");
                    f.rd = parseRegister(pl.operands[0]);
                    f.rs1 = parseRegister(pl.operands[1]);
                    return f;
                }
                if (pl.operands.size() < 3) throw std::invalid_argument("Binary R-type requires 3 operands (rd, rs1, rs2/imm)");
                f.rd = parseRegister(pl.operands[0]);
                f.rs1 = parseRegister(pl.operands[1]);

                std::string op2 = pl.operands[2];
                try {
                    f.rs2 = parseRegister(op2);
                    f.modifier = 0;
                } catch (...) {
                    f.modifier = 1;
                    int64_t imm = parseImmediateOrLabel(op2, pl.pc, labels, false);
                    checkRange(imm, 17, "R-type immediate");
                    f.immediate = static_cast<int32_t>(imm);
                }
                break;
            }

            case ISA::InstructionFormat::I_TYPE: {
                if (f.opcode == ISA::Opcode::LOAD || f.opcode == ISA::Opcode::FLOAD ||
                    f.opcode == ISA::Opcode::STORE || f.opcode == ISA::Opcode::FSTORE) {
                    if (pl.operands.size() < 2) throw std::invalid_argument("Memory instruction requires 2 operands (rd, [rs1, offset])");
                    f.rd = parseRegister(pl.operands[0]);
                    parseMemoryOperand(pl.operands[1], f.rs1, f.immediate, pl.pc, labels);
                    f.modifier = 0;
                } else if (f.opcode == ISA::Opcode::RDSR) {
                    if (pl.operands.size() < 2) throw std::invalid_argument("RDSR requires 2 operands (rd, sr)");
                    f.rd = parseRegister(pl.operands[0]);
                    int64_t sr = parseImmediateOrLabel(pl.operands[1], pl.pc, labels, false);
                    checkRange(sr, 16, "status register index");
                    f.immediate = static_cast<int32_t>(sr);
                    f.modifier = 0;
                } else if (f.opcode == ISA::Opcode::WRSR) {
                    if (pl.operands.size() < 2) throw std::invalid_argument("WRSR requires 2 operands (rs1, sr)");
                    f.rs1 = parseRegister(pl.operands[0]);
                    int64_t sr = parseImmediateOrLabel(pl.operands[1], pl.pc, labels, false);
                    checkRange(sr, 16, "status register index");
                    f.immediate = static_cast<int32_t>(sr);
                    f.modifier = 0;
                }
                break;
            }

            case ISA::InstructionFormat::B_TYPE: {
                if (f.opcode == ISA::Opcode::JZ || f.opcode == ISA::Opcode::JNZ) {
                    if (pl.operands.size() < 2) throw std::invalid_argument("Branch requires 2 operands (rs1, target)");
                    f.rs1 = parseRegister(pl.operands[0]);
                    f.rs2 = 0;
                    int64_t off = parseImmediateOrLabel(pl.operands[1], pl.pc, labels, true);
                    checkRange(off, 18, "branch offset");
                    f.immediate = static_cast<int32_t>(off);
                } else {
                    if (pl.operands.size() < 3) throw std::invalid_argument("Branch requires 3 operands (rs1, rs2, target)");
                    f.rs1 = parseRegister(pl.operands[0]);
                    f.rs2 = parseRegister(pl.operands[1]);
                    int64_t off = parseImmediateOrLabel(pl.operands[2], pl.pc, labels, true);
                    checkRange(off, 18, "branch offset");
                    f.immediate = static_cast<int32_t>(off);
                }
                break;
            }

            case ISA::InstructionFormat::J_TYPE: {
                if (f.opcode == ISA::Opcode::RET) {
                    f.immediate = 0;
                } else {
                    if (pl.operands.empty()) throw std::invalid_argument("Jump requires target operand");
                    int64_t off = parseImmediateOrLabel(pl.operands[0], pl.pc, labels, true);
                    checkRange(off, 26, "jump target offset");
                    f.immediate = static_cast<int32_t>(off);
                }
                break;
            }
        }
        return f;
    }

    static std::string toHex(uint64_t val) {
        std::ostringstream ss;
        ss << std::hex << std::uppercase << val;
        return ss.str();
    }
};

