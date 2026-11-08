#pragma once

// Stack preprocessor (assignment 2e): expands full-descending stack macros into plain
// RISC201 assembly before the assembler runs.
//
// Convention: SP points at the last item pushed; the stack grows toward lower addresses;
// one stack slot is one 64-bit word (8 bytes).
//
//   PUSH r1[, r2...]    SUB SP, SP, 8n ; STORE ri, off(SP)...     (r1 ends up deepest)
//   POP  r1[, r2...]    LOAD ri, off(SP)... ; ADD SP, SP, 8n      (mirror of PUSH: PUSH a,b ; POP a,b restores both)
//   FPUSH f1[, f2...]   as PUSH for FP registers (FSTORE)
//   FPOP  f1[, f2...]   as POP for FP registers (FLOAD)
//   PEEK rd[, n]        LOAD rd, 8n(SP)       read the n-th slot from the top (0 = top), SP unchanged
//   ALLOC n             SUB SP, SP, 8n        reserve n slots (local variables)
//   DROP n              ADD SP, SP, 8n        release n slots
//
// Each expanded line remembers the macro it came from, so the assembler and debugger can
// still show the programmer's original source line.

#include "asm_syntax.cpp"
#include <sstream>
#include <stdexcept>
#include <vector>

struct SourceLine {
    int         line = 0;        // 1-based line in the original source
    std::string text;            // text handed to the assembler
    std::string origin;          // macro this line was expanded from ("" if not from a macro)
};

struct PreprocError : std::runtime_error { using std::runtime_error::runtime_error; };

class StackPreprocessor {
public:
    static constexpr int WORD_BYTES = 8;

    static std::vector<SourceLine> expand(const std::string& source) {
        std::vector<SourceLine> out;
        std::vector<std::string> errors;
        std::istringstream in(source);
        std::string raw;
        int lineNo = 0;
        while (std::getline(in, raw)) {
            ++lineNo;
            if (!raw.empty() && raw.back() == '\r') raw.pop_back();
            std::string code = stripComment(raw);
            std::string label, rest = trim(code);
            const size_t colon = rest.find(':');
            if (colon != std::string::npos && isIdent(trim(rest.substr(0, colon)))) {
                label = trim(rest.substr(0, colon));
                rest  = trim(rest.substr(colon + 1));
            }
            std::string mn, args;
            splitMnemonic(rest, mn, args);
            const std::string M = upperStr(mn);
            if (!isMacro(M)) { out.push_back({lineNo, raw, ""}); continue; }
            try {
                std::vector<std::string> ins = expandMacro(M, splitArgs(args));
                const std::string origin = M + (args.empty() ? "" : " " + trim(args));
                for (size_t i = 0; i < ins.size(); ++i)
                    out.push_back({lineNo, (i == 0 && !label.empty() ? label + ": " : "    ") + ins[i], origin});
            } catch (const std::exception& e) {
                errors.push_back("line " + std::to_string(lineNo) + ": " + e.what());
            }
        }
        if (!errors.empty()) {
            std::string m = "preprocessor errors:";
            for (const std::string& e : errors) m += "\n  " + e;
            throw PreprocError(m);
        }
        return out;
    }

    // The expanded program as text (for the CLI `expand` command and for debugging).
    static std::string toText(const std::vector<SourceLine>& lines) {
        std::string s;
        for (const SourceLine& l : lines) {
            s += l.text;
            if (!l.origin.empty()) s += "    ; <- " + l.origin;
            s += "\n";
        }
        return s;
    }

    static std::string trim(const std::string& s) {
        size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
        return a == std::string::npos ? "" : s.substr(a, b - a + 1);
    }
    static std::string stripComment(const std::string& s) {
        size_t cut = std::string::npos;
        for (const char* c : {";", "#", "//"}) { size_t p = s.find(c); if (p < cut) cut = p; }
        return cut == std::string::npos ? s : s.substr(0, cut);
    }

private:
    static bool isIdent(const std::string& s) {
        if (s.empty() || !(std::isalpha(static_cast<unsigned char>(s[0])) || s[0] == '_' || s[0] == '.')) return false;
        for (char c : s) if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '.')) return false;
        return true;
    }
    static void splitMnemonic(const std::string& s, std::string& mn, std::string& args) {
        const size_t p = s.find_first_of(" \t");
        mn = p == std::string::npos ? s : s.substr(0, p);
        args = p == std::string::npos ? "" : trim(s.substr(p));
    }
    static std::vector<std::string> splitArgs(const std::string& a) {
        std::vector<std::string> r;
        if (trim(a).empty()) return r;
        std::stringstream ss(a); std::string t;
        while (std::getline(ss, t, ',')) r.push_back(trim(t));
        return r;
    }
    static bool isMacro(const std::string& m) {
        return m == "PUSH" || m == "POP" || m == "FPUSH" || m == "FPOP" || m == "PEEK" || m == "ALLOC" || m == "DROP";
    }
    static bool parseReg(const std::string& tok, bool fp, std::string& canon) {
        const std::string t = upperStr(tok);
        if (!fp) {
            if (t == "SP" || t == "LR" || t == "PC") { canon = t; return true; }
            if (t.size() >= 2 && t[0] == 'R' && t.find_first_not_of("0123456789", 1) == std::string::npos && std::stoi(t.substr(1)) < 16) { canon = t; return true; }
        } else if (t.size() >= 2 && t[0] == 'F' && t.find_first_not_of("0123456789", 1) == std::string::npos && std::stoi(t.substr(1)) < 16) {
            canon = t; return true;
        }
        return false;
    }
    static int slotCount(const std::string& tok, const char* what) {
        char* end = nullptr;
        const long n = std::strtol(tok.c_str(), &end, 0);
        if (tok.empty() || *end != '\0' || n < 0 || n > 4000) throw std::invalid_argument(std::string(what) + ": slot count must be a number 0..4000, got '" + tok + "'");
        return static_cast<int>(n);
    }
    static std::string off(int slots) { return std::to_string(slots * WORD_BYTES); }

    static std::vector<std::string> expandMacro(const std::string& M, const std::vector<std::string>& a) {
        std::vector<std::string> o;
        if (M == "ALLOC" || M == "DROP") {
            if (a.size() != 1) throw std::invalid_argument(M + " takes one operand (number of slots)");
            const int n = slotCount(a[0], M.c_str());
            o.push_back(std::string(M == "ALLOC" ? "SUB" : "ADD") + " SP, SP, " + off(n));
            return o;
        }
        if (M == "PEEK") {
            if (a.empty() || a.size() > 2) throw std::invalid_argument("PEEK rd[, slot]");
            std::string rd;
            if (!parseReg(a[0], false, rd) || rd == "PC") throw std::invalid_argument("PEEK: bad destination register '" + a[0] + "'");
            o.push_back("LOAD " + rd + ", " + off(a.size() == 2 ? slotCount(a[1], "PEEK") : 0) + "(SP)");
            return o;
        }
        const bool fp = (M == "FPUSH" || M == "FPOP");
        const bool push = (M == "PUSH" || M == "FPUSH");
        if (a.empty()) throw std::invalid_argument(M + " needs at least one register");
        std::vector<std::string> regs;
        for (const std::string& t : a) {
            std::string r;
            if (!parseReg(t, fp, r)) throw std::invalid_argument(M + ": bad register '" + t + "'");
            if (!fp && r == "SP") throw std::invalid_argument(M + " SP is not supported");
            if (!fp && !push && (r == "PC" || r == "R15")) throw std::invalid_argument("POP into PC is illegal (R15 is read-only)");
            if (!fp && (r == "R0") && !push) throw std::invalid_argument("POP R0 would discard the value; use DROP 1");
            regs.push_back(r);
        }
        const int n = static_cast<int>(regs.size());
        const char* st = fp ? "FSTORE" : "STORE";
        const char* ld = fp ? "FLOAD" : "LOAD";
        if (push) {
            o.push_back("SUB SP, SP, " + off(n));
            for (int i = 0; i < n; ++i) o.push_back(std::string(st) + " " + regs[i] + ", " + off(n - 1 - i) + "(SP)");
        } else {
            for (int i = n - 1; i >= 0; --i) o.push_back(std::string(ld) + " " + regs[i] + ", " + off(n - 1 - i) + "(SP)");
            o.push_back("ADD SP, SP, " + off(n));
        }
        return o;
    }
};
