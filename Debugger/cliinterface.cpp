#include "non-pipelined simulator.cpp"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

namespace {
using Fields = ISA::InstructionFields;
using Op = ISA::Opcode;

std::string trim(std::string s) {
    auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}
std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}
std::string cleanLine(std::string s) {
    for (const auto marker : {"#", ";", "//"}) {
        auto p = s.find(marker);
        if (p != std::string::npos) s.erase(p);
    }
    for (char& c : s) if (c == ',' || c == '[' || c == ']') c = ' ';
    return trim(s);
}
std::vector<std::string> tokens(const std::string& s) {
    std::istringstream in(s); std::vector<std::string> out; std::string t;
    while (in >> t) out.push_back(t);
    return out;
}
uint64_t number(const std::string& s) {
    size_t used = 0;
    long long v = std::stoll(s, &used, 0);
    if (used != s.size()) throw std::runtime_error("invalid number: " + s);
    return static_cast<uint64_t>(v);
}
uint8_t regNum(std::string s) {
    s = upper(s);
    if (!s.empty() && s[0] == 'R') s.erase(s.begin());
    auto v = number(s);
    if (v >= ISA::REGISTER_COUNT) throw std::runtime_error("register must be R0..R15");
    return static_cast<uint8_t>(v);
}

struct SourceLine { int lineNo; uint64_t pc; std::string text; };

ISA::InstrWord assembleOne(const std::vector<std::string>& t, uint64_t pc,
                           const std::unordered_map<std::string, uint64_t>& labels) {
    if (t.empty()) throw std::runtime_error("empty instruction");
    const std::string mnemonic = upper(t[0]);
    Fields f;
    f.opcode = ISA::opcodeFromMnemonic(mnemonic);
    auto target = [&](const std::string& arg) -> int32_t {
        auto it = labels.find(upper(arg));
        if (it != labels.end()) {
            int64_t delta = static_cast<int64_t>(it->second) - static_cast<int64_t>(pc);
            if (delta < INT32_MIN || delta > INT32_MAX) throw std::runtime_error("branch target out of range");
            return static_cast<int32_t>(delta);
        }
        return static_cast<int32_t>(std::stoll(arg, nullptr, 0));
    };
    switch (f.format()) {
        case ISA::InstructionFormat::R_TYPE:
            if (mnemonic == "HALT" || mnemonic == "NOP") {
                if (t.size() != 1) throw std::runtime_error(mnemonic + " takes no operands");
            } else if (ISA::isUnary(f.opcode)) {
                if (t.size() != 3) throw std::runtime_error(mnemonic + " syntax: " + mnemonic + " Rd Rs1");
                f.rd = regNum(t[1]); f.rs1 = regNum(t[2]);
            } else {
                if (t.size() != 4) throw std::runtime_error(mnemonic + " syntax: " + mnemonic + " Rd Rs1 Rs2");
                f.rd = regNum(t[1]); f.rs1 = regNum(t[2]); f.rs2 = regNum(t[3]);
            }
            break;
        case ISA::InstructionFormat::I_TYPE:
            if (t.size() != 4) throw std::runtime_error(mnemonic + " syntax: " + mnemonic + " Rd/base Rs1/base offset");
            f.rd = regNum(t[1]); f.rs1 = regNum(t[2]); f.immediate = target(t[3]);
            break;
        case ISA::InstructionFormat::B_TYPE:
            if (t.size() != 4) throw std::runtime_error(mnemonic + " syntax: " + mnemonic + " Rs1 Rs2 label_or_offset");
            f.rs1 = regNum(t[1]); f.rs2 = regNum(t[2]); f.immediate = target(t[3]);
            break;
        case ISA::InstructionFormat::J_TYPE:
            if (mnemonic == "RET") {
                if (t.size() != 1) throw std::runtime_error("RET takes no operands");
            } else {
                if (t.size() != 2) throw std::runtime_error(mnemonic + " syntax: " + mnemonic + " label_or_offset");
                f.immediate = target(t[1]);
            }
            break;
        case ISA::InstructionFormat::M_TYPE:
            throw std::runtime_error("matrix assembly is not enabled in this CLI build yet");
    }
    return ISA::encode(f);
}

std::vector<ISA::InstrWord> assembleFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    std::vector<SourceLine> lines;
    std::unordered_map<std::string, uint64_t> labels;
    std::string raw; int lineNo = 0; uint64_t pc = 0;
    while (std::getline(in, raw)) {
        ++lineNo; std::string s = cleanLine(raw); if (s.empty()) continue;
        auto colon = s.find(':');
        if (colon != std::string::npos) {
            std::string label = upper(trim(s.substr(0, colon)));
            if (label.empty()) throw std::runtime_error("line " + std::to_string(lineNo) + ": empty label");
            if (labels.count(label)) throw std::runtime_error("line " + std::to_string(lineNo) + ": duplicate label " + label);
            labels[label] = pc;
            s = trim(s.substr(colon + 1)); if (s.empty()) continue;
        }
        lines.push_back({lineNo, pc, s}); pc += 4;
    }
    std::vector<ISA::InstrWord> words; words.reserve(lines.size());
    for (const auto& line : lines) {
        try { words.push_back(assembleOne(tokens(line.text), line.pc, labels)); }
        catch (const std::exception& e) { throw std::runtime_error("line " + std::to_string(line.lineNo) + ": " + e.what()); }
    }
    return words;
}

std::string disassemble(ISA::InstrWord word) {
    Fields f = ISA::decode(word); std::ostringstream o;
    o << ISA::mnemonicOf(f.opcode);
    switch (f.format()) {
        case ISA::InstructionFormat::R_TYPE:
            if (f.opcode == Op::HALT || f.opcode == Op::NOP || f.opcode == Op::RET) break;
            o << " R" << unsigned(f.rd) << ", R" << unsigned(f.rs1);
            if (!ISA::isUnary(f.opcode)) o << ", R" << unsigned(f.rs2);
            break;
        case ISA::InstructionFormat::I_TYPE:
            o << " R" << unsigned(f.rd) << ", R" << unsigned(f.rs1) << ", " << f.immediate; break;
        case ISA::InstructionFormat::B_TYPE:
            o << " R" << unsigned(f.rs1) << ", R" << unsigned(f.rs2) << ", " << f.immediate; break;
        case ISA::InstructionFormat::J_TYPE: if (f.opcode != Op::RET) o << " " << f.immediate; break;
        case ISA::InstructionFormat::M_TYPE:
            o << " R" << unsigned(f.rd) << ", R" << unsigned(f.rs1) << ", R" << unsigned(f.rs2)
              << ", rows=" << unsigned(f.matrixRows) << ", cols=" << unsigned(f.matrixCols); break;
    }
    return o.str();
}

class Debugger {
    std::unique_ptr<Simulator> sim = std::make_unique<Simulator>();
    std::vector<ISA::InstrWord> program;
    std::set<uint64_t> breakpoints;
    bool loaded = false;

    void loadWords(const std::vector<ISA::InstrWord>& words) {
        sim = std::make_unique<Simulator>(); program = words; breakpoints.clear();
        for (size_t i = 0; i < words.size(); ++i) sim->loadInstruction(i * 4, words[i]);
        loaded = true;
        std::cout << "Loaded " << words.size() << " instructions. Entry PC = 0x0\n";
    }
    bool breakpointAtPC() { return breakpoints.count(sim->getRegister(ISA::PC)) != 0; }
    void showCurrent() {
        uint64_t pc = sim->getRegister(ISA::PC);
        try { auto word = sim->fetchInstruction(pc); std::cout << "PC=0x" << std::hex << pc << "  0x" << std::setw(8) << std::setfill('0') << word << std::dec << "  " << disassemble(word) << "\n"; }
        catch (const std::exception& e) { std::cout << "PC=0x" << std::hex << pc << std::dec << " (" << e.what() << ")\n"; }
    }
    void stepOnce() {
        if (!loaded) { std::cout << "Load a program first.\n"; return; }
        showCurrent();
        if (!sim->step()) { std::cout << "Execution stopped (halt/error).\n"; return; }
        std::cout << " -> PC=0x" << std::hex << sim->getRegister(ISA::PC) << std::dec << ", cycles=" << sim->getCycleCount() << "\n";
    }
    void runUntilStop(bool skipCurrentBreakpoint) {
        if (!loaded) { std::cout << "Load a program first.\n"; return; }
        bool skipOnce = skipCurrentBreakpoint && breakpointAtPC();
        size_t safety = 0;
        while (!sim->isHalted() && safety++ < 1000000) {
            if (breakpointAtPC() && !skipOnce) { std::cout << "Breakpoint hit at PC=0x" << std::hex << sim->getRegister(ISA::PC) << std::dec << "\n"; showCurrent(); return; }
            skipOnce = false;
            uint64_t oldpc = sim->getRegister(ISA::PC);
            if (!sim->step()) { std::cout << "Execution stopped (halt/error).\n"; break; }
            if (sim->getRegister(ISA::PC) == oldpc && !sim->isHalted()) { std::cout << "PC did not advance; stopping to avoid an infinite loop.\n"; break; }
        }
        std::cout << (sim->isHalted() ? "Program halted" : "Run stopped") << "; cycles=" << sim->getCycleCount() << "\n";
        showCurrent();
    }
    void demo() {
        Fields a; a.opcode=Op::ADD; a.rd=1; a.rs1=2; a.rs2=3;
        Fields b; b.opcode=Op::ADD; b.rd=4; b.rs1=1; b.rs2=3;
        Fields c; c.opcode=Op::SUB; c.rd=5; c.rs1=4; c.rs2=2;
        Fields h; h.opcode=Op::HALT;
        loadWords({ISA::encode(a), ISA::encode(b), ISA::encode(c), ISA::encode(h)});
        sim->setRegister(2, 10); sim->setRegister(3, 7);
        std::cout << "Demo initialized: R2=10, R3=7. Expected: R1=17, R4=24, R5=14.\n";
    }
public:
    void loop() {
        std::cout << "RISC201 Interactive CLI Debugger\nType 'help' for commands.\n";
        std::string line;
        while (std::cout << "dbg> " && std::getline(std::cin, line)) {
            line = trim(line); if (line.empty()) continue;
            auto t = tokens(line); auto cmd = upper(t[0]);
            try {
                if (cmd == "QUIT" || cmd == "EXIT") break;
                else if (cmd == "HELP") {
                    std::cout << "load <file.asm> | demo | step | run | continue | regs | mem <start> <count>\n"
                                 "break <address> | delete <address> | setreg <R0..R15> <value> | disasm | reset | help | quit\n"
                                 "Assembly subset: ADD/SUB/MUL/DIV/MOD/AND/OR/XOR/LSL/LSR/ASR/CMP Rd Rs1 Rs2;\n"
                                 "NOT/SIN/COS/LOG Rd Rs1; LOAD/STORE Rd Rs1 offset; JEQ/JGT/JZ/JNZ/JMP/CALL labels; RET/NOP/HALT.\n";
                } else if (cmd == "LOAD") {
                    if (t.size()!=2) throw std::runtime_error("usage: load <file.asm>");
                    loadWords(assembleFile(t[1]));
                } else if (cmd == "DEMO") demo();
                else if (cmd == "STEP") stepOnce();
                else if (cmd == "RUN" || cmd == "CONTINUE" || cmd == "C") runUntilStop(cmd=="CONTINUE" || cmd=="C");
                else if (cmd == "REGS" || cmd == "REGISTERS") { if (!loaded) throw std::runtime_error("load a program first"); sim->dumpRegisters(); }
                else if (cmd == "MEM") {
                    if (t.size()!=3) throw std::runtime_error("usage: mem <start> <count>");
                    sim->dumpMemory(number(t[1]), number(t[2]));
                } else if (cmd == "BREAK" || cmd == "B") {
                    if (t.size()!=2) throw std::runtime_error("usage: break <byte-address>");
                    uint64_t a=number(t[1]); if (a%4) throw std::runtime_error("breakpoint address must be 4-byte aligned");
                    breakpoints.insert(a); std::cout << "Breakpoint set at 0x" << std::hex << a << std::dec << "\n";
                } else if (cmd == "DELETE" || cmd == "DB") {
                    if (t.size()!=2) throw std::runtime_error("usage: delete <byte-address>");
                    breakpoints.erase(number(t[1])); std::cout << "Breakpoint removed if it existed.\n";
                } else if (cmd == "SETREG") {
                    if (t.size()!=3) throw std::runtime_error("usage: setreg Rn value");
                    uint8_t r=regNum(t[1]); if (r==ISA::ZERO) throw std::runtime_error("R0 is the constant zero register");
                    sim->setRegister(r, number(t[2])); std::cout << t[1] << " updated.\n";
                } else if (cmd == "DISASM" || cmd == "WHERE") showCurrent();
                else if (cmd == "RESET") { if (!loaded) throw std::runtime_error("no program to reset"); auto saved=program; loadWords(saved); }
                else std::cout << "Unknown command: " << t[0] << ". Type help.\n";
            } catch (const std::exception& e) { std::cout << "Error: " << e.what() << "\n"; }
        }
        std::cout << "Debugger closed.\n";
    }
};
}

int main() { Debugger debugger; debugger.loop(); return 0; }
