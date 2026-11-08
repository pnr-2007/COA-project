#pragma once

// Disassembler: 32-bit instruction word -> assembly text (assignment 2b).

#include "asm_syntax.cpp"
#include <cstdio>
#include <map>
#include <sstream>

class Disassembler {
public:
    // `symbols` (address -> label) is optional; when given, branch targets print as labels.
    static std::string regName(unsigned r, bool fp = false) {
        if (fp) return "F" + std::to_string(r);
        if (r == ISA::SP) return "SP";
        if (r == ISA::LR) return "LR";
        if (r == ISA::PC) return "PC";
        return "R" + std::to_string(r);
    }

    static std::string format(ISA::InstrWord word, uint64_t pc = 0, const std::map<uint64_t, std::string>* symbols = nullptr) {
        ISA::InstructionFields f;
        try { f = ISA::decode(word); }
        catch (const std::invalid_argument&) { return hexWord(word); }

        using F = AsmSyntax::Form;
        const AsmSyntax s = syntaxOf(f.opcode);
        std::string out(ISA::mnemonicOf(f.opcode));
        auto target = [&](int32_t imm) {
            const uint64_t t = pc + static_cast<uint64_t>(static_cast<int64_t>(imm) * 4);
            if (symbols) { auto it = symbols->find(t); if (it != symbols->end()) return it->second; }
            char b[32]; std::snprintf(b, sizeof b, "0x%llX", (unsigned long long)t);
            return std::string(b) + " (" + (imm >= 0 ? "+" : "") + std::to_string(imm) + ")";
        };
        switch (s.form) {
            case F::NONE: break;
            case F::D_S:
                out += " " + regName(f.rd, s.fpD) + ", " + regName(f.rs1, s.fpS); break;
            case F::D_S_T:
                out += " " + regName(f.rd, s.fpD) + ", " + regName(f.rs1, s.fpS) + ", ";
                out += f.modifier ? std::to_string(f.immediate) : regName(f.rs2, s.fpT);
                break;
            case F::MEM:
                out += " " + regName(f.rd, s.fpD) + ", " + std::to_string(f.immediate) + "(" + regName(f.rs1) + ")";
                break;
            case F::B_S_T: out += " " + regName(f.rs1) + ", " + regName(f.rs2) + ", " + target(f.immediate); break;
            case F::B_S:   out += " " + regName(f.rs1) + ", " + target(f.immediate); break;
            case F::J:     out += " " + target(f.immediate); break;
            case F::RDSR:  out += " " + regName(f.rd) + ", " + specialRegName(static_cast<unsigned>(f.immediate)); break;
            case F::WRSR:  out += std::string(" ") + specialRegName(static_cast<unsigned>(f.immediate)) + ", " + regName(f.rs1); break;
        }
        return out;
    }

    // "0x00000010: 04C40008  ADD R1, R2, R3"
    static std::string line(ISA::InstrWord word, uint64_t pc, const std::map<uint64_t, std::string>* symbols = nullptr) {
        char b[48]; std::snprintf(b, sizeof b, "0x%05llX: %08X  ", (unsigned long long)pc, word);
        return b + format(word, pc, symbols);
    }

    // Hex/binary word strings (assignment: "parses binary or hexadecimal word strings").
    // Accepts "0x1A2B3C4D", "1A2B3C4D" or a 32-digit binary string (optionally "0b" prefixed).
    static ISA::InstrWord parseWord(std::string s) {
        std::string t;
        for (char c : s) if (!std::isspace(static_cast<unsigned char>(c)) && c != '_') t += c;
        if (t.size() > 2 && t[0] == '0' && (t[1] == 'b' || t[1] == 'B')) return static_cast<ISA::InstrWord>(std::stoull(t.substr(2), nullptr, 2));
        if (t.size() == 32 && t.find_first_not_of("01") == std::string::npos) return static_cast<ISA::InstrWord>(std::stoull(t, nullptr, 2));
        if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) t = t.substr(2);
        return static_cast<ISA::InstrWord>(std::stoull(t, nullptr, 16));
    }

private:
    static std::string hexWord(ISA::InstrWord w) {
        char b[32]; std::snprintf(b, sizeof b, ".word 0x%08X", w); return b;
    }
};

