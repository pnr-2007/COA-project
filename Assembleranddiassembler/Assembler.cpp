#pragma once

// Two-pass assembler: RISC201 assembly -> instruction words + initial data (assignment 2a).
//
//   Pass 1  walks every line, tracks the text/data location counters and records labels and .equ.
//   Pass 2  encodes each instruction, resolving forward references.
//
// Syntax
//   label:  OP operands          ; comment  (also '#' and '//')
//   registers  R0..R15 SP(R13) LR(R14) PC(R15)    FP registers F0..F15
//   immediates decimal, 0x.., 0b.., 'c', symbols, symbol+const / symbol-const
//   LOAD/STORE rd, imm(rs1)   (imm optional;  `LOAD R1, label` means label(R0))
//   branch target: a label (assembler computes the offset) or a plain number (offset in instructions)
//   .text / .data           select section (default .text)
//   .word v[, v...]         64-bit data words      .space n   n zero words
//   .equ NAME, value        constant               .stack n   stack size in bytes (multiple of 8)

#include "asm_syntax.cpp"
#include "stack_macros.cpp"
#include "exec.cpp"
#include <map>
#include <set>

struct AsmError : std::runtime_error { using std::runtime_error::runtime_error; };

struct Program {
    struct Line { int srcLine = 0; std::string text, origin; };
    std::vector<ISA::InstrWord> text;
    std::vector<ISA::Word>      data;                 // initial data memory from address 0
    std::vector<Line>           lines;                // one per instruction (source mapping)
    std::map<std::string, uint64_t> symbols;          // labels and .equ constants
    std::map<uint64_t, std::string> textLabels;       // address -> label (instruction space)
    std::map<uint64_t, std::string> dataLabels;       // address -> label (data space)
    uint64_t stackBytes = 0;                          // 0 = not specified
};

class Assembler {
public:
    static Program assemble(const std::string& source) {
        std::vector<SourceLine> lines;
        std::istringstream in(source);
        std::string l; int n = 0;
        while (std::getline(in, l)) { if (!l.empty() && l.back() == '\r') l.pop_back(); lines.push_back({++n, l, ""}); }
        return assemble(lines);
    }

    static Program assemble(const std::vector<SourceLine>& lines) {
        Assembler a;
        a.pass1(lines);
        a.pass2();
        if (!a.errors_.empty()) {
            std::string m = "assembler errors:";
            for (const std::string& e : a.errors_) m += "\n  " + e;
            throw AsmError(m);
        }
        return std::move(a.prog_);
    }

private:
    struct Item {                  // one instruction or data directive collected by pass 1
        enum Kind { INSTR, WORD, SPACE } kind = INSTR;
        const SourceLine* src = nullptr;
        std::string mnemonic;
        std::vector<std::string> ops;
        uint64_t addr = 0;
    };

    Program prog_;
    std::vector<Item> items_;
    std::vector<std::string> errors_;
    std::set<std::string> equs_;

    void err(const SourceLine& s, const std::string& m) { errors_.push_back("line " + std::to_string(s.line) + ": " + m); }

    // ---- lexing helpers ----
    static std::string trim(const std::string& s) { return StackPreprocessor::trim(s); }

    static std::vector<std::string> splitOperands(const std::string& s) {   // commas outside parentheses
        std::vector<std::string> r; std::string cur; int depth = 0;
        for (char c : s) {
            if (c == '(') ++depth; else if (c == ')') --depth;
            if (c == ',' && depth == 0) { r.push_back(trim(cur)); cur.clear(); } else cur += c;
        }
        if (!trim(cur).empty() || !r.empty()) r.push_back(trim(cur));
        return r;
    }

    // ---- expression evaluation: term (+|- term)* ----
    struct Val { int64_t v = 0; bool usesSymbol = false; };

    bool eval(const SourceLine& src, const std::string& expr, Val& out, bool needDefined) {
        const std::string e = trim(expr);
        if (e.empty()) { err(src, "missing value"); return false; }
        size_t i = 0; int sign = 1; Val acc; bool first = true;
        while (i < e.size()) {
            while (i < e.size() && std::isspace(static_cast<unsigned char>(e[i]))) ++i;
            if (first || e[i] == '+' || e[i] == '-') {
                if (!first || e[i] == '+' || e[i] == '-') { sign = e[i] == '-' ? -1 : 1; ++i; }
                while (i < e.size() && std::isspace(static_cast<unsigned char>(e[i]))) ++i;
            }
            first = false;
            size_t j = i;
            if (j < e.size() && e[j] == '\'') {
                if (j + 2 < e.size() && e[j + 2] == '\'') { acc.v += sign * static_cast<unsigned char>(e[j + 1]); i = j + 3; continue; }
                err(src, "bad character literal in '" + e + "'"); return false;
            }
            while (j < e.size() && e[j] != '+' && e[j] != '-' && !std::isspace(static_cast<unsigned char>(e[j]))) ++j;
            const std::string tok = e.substr(i, j - i);
            i = j;
            if (tok.empty()) { err(src, "bad expression '" + e + "'"); return false; }
            if (std::isdigit(static_cast<unsigned char>(tok[0]))) {
                char* end = nullptr; int base = 10; std::string t = tok;
                if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) { base = 16; t = t.substr(2); }
                else if (t.size() > 2 && t[0] == '0' && (t[1] == 'b' || t[1] == 'B')) { base = 2; t = t.substr(2); }
                const unsigned long long u = std::strtoull(t.c_str(), &end, base);
                if (*end != '\0') { err(src, "bad number '" + tok + "'"); return false; }
                acc.v += sign * static_cast<int64_t>(u);
            } else {
                auto it = prog_.symbols.find(tok);
                if (it == prog_.symbols.end()) {
                    if (needDefined) { err(src, "undefined symbol '" + tok + "'"); return false; }
                    acc.usesSymbol = true;
                } else { acc.v += sign * static_cast<int64_t>(it->second); acc.usesSymbol = true; }
            }
            sign = 1;
            while (i < e.size() && std::isspace(static_cast<unsigned char>(e[i]))) ++i;
            if (i < e.size() && e[i] != '+' && e[i] != '-') { err(src, "bad expression '" + e + "'"); return false; }
        }
        out = acc;
        return true;
    }

    // ---- registers ----
    bool parseReg(const SourceLine& s, const std::string& tok, bool fp, uint8_t& r) {
        const std::string t = upperStr(trim(tok));
        if (!fp) {
            if (t == "SP") { r = ISA::SP; return true; }
            if (t == "LR") { r = ISA::LR; return true; }
            if (t == "PC") { r = ISA::PC; return true; }
        }
        const char want = fp ? 'F' : 'R';
        if (t.size() >= 2 && t[0] == want && t.find_first_not_of("0123456789", 1) == std::string::npos) {
            const int n = std::stoi(t.substr(1));
            if (n < 16) { r = static_cast<uint8_t>(n); return true; }
        }
        err(s, std::string("expected ") + (fp ? "FP register F0..F15" : "register R0..R15/SP/LR/PC") + ", got '" + tok + "'");
        return false;
    }

    // ---- pass 1 ----
    void pass1(const std::vector<SourceLine>& lines) {
        bool inData = false;
        uint64_t textCount = 0, dataCount = 0;
        for (const SourceLine& src : lines) {
            std::string code = trim(StackPreprocessor::stripComment(src.text));
            for (;;) {                                                // leading labels
                const size_t colon = code.find(':');
                if (colon == std::string::npos) break;
                const std::string name = trim(code.substr(0, colon));
                if (name.empty() || name.find_first_of(" \t,()") != std::string::npos) break;
                const uint64_t addr = inData ? dataCount * 8 : textCount * 4;
                if (prog_.symbols.count(name)) err(src, "duplicate symbol '" + name + "'");
                else {
                    prog_.symbols[name] = addr;
                    (inData ? prog_.dataLabels : prog_.textLabels)[addr] = name;
                }
                code = trim(code.substr(colon + 1));
            }
            if (code.empty()) continue;
            const size_t sp = code.find_first_of(" \t");
            const std::string head = sp == std::string::npos ? code : code.substr(0, sp);
            const std::string rest = sp == std::string::npos ? "" : trim(code.substr(sp));
            const std::string H = upperStr(head);

            if (H == ".TEXT") { inData = false; continue; }
            if (H == ".DATA") { inData = true; continue; }
            if (H == ".EQU") {
                std::vector<std::string> o = splitOperands(rest);
                Val v;
                if (o.size() != 2 || !eval(src, o[1], v, true)) { if (o.size() != 2) err(src, ".equ NAME, value"); continue; }
                if (prog_.symbols.count(o[0])) err(src, "duplicate symbol '" + o[0] + "'");
                else prog_.symbols[o[0]] = static_cast<uint64_t>(v.v);
                continue;
            }
            if (H == ".STACK") {
                Val v;
                if (eval(src, rest, v, true)) {
                    if (v.v <= 0 || v.v % 8) err(src, ".stack size must be a positive multiple of 8 bytes");
                    else prog_.stackBytes = static_cast<uint64_t>(v.v);
                }
                continue;
            }
            if (H == ".WORD" || H == ".SPACE") {
                if (!inData) { err(src, head + " is only allowed in a .data section"); continue; }
                Item it; it.src = &src; it.addr = dataCount * 8;
                if (H == ".WORD") {
                    it.kind = Item::WORD; it.ops = splitOperands(rest);
                    if (it.ops.empty()) { err(src, ".word needs at least one value"); continue; }
                    dataCount += it.ops.size();
                } else {
                    Val v;
                    if (!eval(src, rest, v, true) || v.v < 0 || v.v > (1 << 20)) { err(src, ".space needs a count 0..1048576"); continue; }
                    it.kind = Item::SPACE; it.ops = {std::to_string(v.v)};
                    dataCount += static_cast<uint64_t>(v.v);
                }
                items_.push_back(it);
                continue;
            }
            if (H[0] == '.') { err(src, "unknown directive '" + head + "'"); continue; }
            if (inData) { err(src, "instruction in .data section"); continue; }
            Item it; it.kind = Item::INSTR; it.src = &src; it.mnemonic = H; it.ops = splitOperands(rest);
            it.addr = textCount * 4;
            ++textCount;
            items_.push_back(it);
        }
    }

    // ---- pass 2 ----
    void pass2() {
        for (Item& it : items_) {
            if (it.kind == Item::WORD) {
                for (const std::string& o : it.ops) { Val v; if (eval(*it.src, o, v, true)) prog_.data.push_back(static_cast<ISA::Word>(v.v)); else prog_.data.push_back(0); }
            } else if (it.kind == Item::SPACE) {
                prog_.data.insert(prog_.data.end(), static_cast<size_t>(std::stoull(it.ops[0])), 0);
            } else {
                ISA::InstrWord w = 0;
                if (!encode(it, w)) w = 0xFFFFFFFFu;
                prog_.text.push_back(w);
                prog_.lines.push_back({it.src->line, StackPreprocessor::trim(StackPreprocessor::stripComment(it.src->text)), it.src->origin});
            }
        }
    }

    bool fitsSigned(int64_t v, int bits) { return v >= -(int64_t(1) << (bits - 1)) && v < (int64_t(1) << (bits - 1)); }

    bool encode(const Item& it, ISA::InstrWord& word) {
        const SourceLine& s = *it.src;
        ISA::Opcode op;
        try { op = ISA::opcodeFromMnemonic(it.mnemonic); }
        catch (const std::invalid_argument&) { err(s, "unknown mnemonic '" + it.mnemonic + "'"); return false; }
        const AsmSyntax syn = syntaxOf(op);
        using F = AsmSyntax::Form;
        const std::vector<std::string>& o = it.ops;
        ISA::InstructionFields f; f.opcode = op;
        auto need = [&](size_t n) {
            if (o.size() != n) { err(s, it.mnemonic + " expects " + std::to_string(n) + " operand(s), got " + std::to_string(o.size())); return false; }
            return true;
        };
        auto imm = [&](const std::string& e, int bits, int32_t& out) {
            Val v;
            if (!eval(s, e, v, true)) return false;
            if (!fitsSigned(v.v, bits)) { err(s, "immediate " + std::to_string(v.v) + " does not fit in " + std::to_string(bits) + " signed bits"); return false; }
            out = static_cast<int32_t>(v.v); return true;
        };
        auto branchOffset = [&](const std::string& e, int bits, int32_t& out) {
            Val v;
            if (!eval(s, e, v, true)) return false;
            int64_t off = v.v;
            if (v.usesSymbol) {
                const int64_t d = v.v - static_cast<int64_t>(it.addr);
                if (d % 4) { err(s, "branch target is not 4-byte aligned"); return false; }
                off = d / 4;
            }
            if (!fitsSigned(off, bits)) { err(s, "branch target out of range"); return false; }
            out = static_cast<int32_t>(off); return true;
        };
        bool ok = true;
        switch (syn.form) {
            case F::NONE: ok = need(0); break;
            case F::D_S:  ok = need(2) && parseReg(s, o[0], syn.fpD, f.rd) && parseReg(s, o[1], syn.fpS, f.rs1); break;
            case F::D_S_T:
                ok = need(3) && parseReg(s, o[0], syn.fpD, f.rd) && parseReg(s, o[1], syn.fpS, f.rs1);
                if (!ok) break;
                if (syn.allowImm) {
                    const std::string t = upperStr(trim(o[2]));
                    const bool isReg = t == "SP" || t == "LR" || t == "PC" ||
                        (t.size() >= 2 && t[0] == 'R' && t.find_first_not_of("0123456789", 1) == std::string::npos);
                    if (isReg) ok = parseReg(s, o[2], false, f.rs2);
                    else { f.modifier = 1; ok = imm(o[2], 17, f.immediate); }
                } else ok = parseReg(s, o[2], syn.fpT, f.rs2);
                break;
            case F::MEM: {
                ok = need(2) && parseReg(s, o[0], syn.fpD, f.rd);
                if (!ok) break;
                const std::string a = trim(o[1]);
                const size_t lp = a.find('('), rp = a.rfind(')');
                if (lp != std::string::npos && rp == a.size() - 1 && rp > lp) {
                    const std::string disp = trim(a.substr(0, lp));
                    ok = parseReg(s, a.substr(lp + 1, rp - lp - 1), false, f.rs1) && (disp.empty() || imm(disp, 16, f.immediate));
                } else { f.rs1 = ISA::ZERO; ok = imm(a, 16, f.immediate); }
                break;
            }
            case F::B_S_T: ok = need(3) && parseReg(s, o[0], false, f.rs1) && parseReg(s, o[1], false, f.rs2) && branchOffset(o[2], 18, f.immediate); break;
            case F::B_S:   ok = need(2) && parseReg(s, o[0], false, f.rs1) && branchOffset(o[1], 18, f.immediate); break;
            case F::J:     ok = need(1) && branchOffset(o[0], 26, f.immediate); break;
            case F::RDSR: case F::WRSR: {
                const bool rd = syn.form == F::RDSR;
                ok = need(2);
                if (!ok) break;
                const std::string& selTok = rd ? o[1] : o[0];
                const std::string& regTok = rd ? o[0] : o[1];
                ok = parseReg(s, regTok, false, rd ? f.rd : f.rs1);
                if (!ok) break;
                const std::string S = upperStr(trim(selTok));
                int sel = -1;
                for (int i = 0; i < 8; ++i) if (S == specialRegName(i)) sel = i;
                if (sel < 0) { int32_t v; if (!imm(selTok, 16, v)) { ok = false; break; } sel = v; }
                if (!ExceptionHandler::validSelect(sel)) { err(s, "bad special-register select '" + selTok + "'"); ok = false; break; }
                f.immediate = sel;
                break;
            }
        }
        if (!ok) return false;
        // Register-writing instructions may not target R15 (PC is read-only).
        const Exec::RegUse u = Exec::regUse(f);
        if (u.writes && !u.destFp && u.dest == ISA::PC && op != ISA::Opcode::CALL) { err(s, "R15 (PC) cannot be written"); return false; }
        word = ISA::encode(f);
        return true;
    }
};


