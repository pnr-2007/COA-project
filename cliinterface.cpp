#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

using namespace std;

// ==========================================
// ISA Definition & Decoder
// ==========================================
class ISA {
public:
    static const uint8_t OPCODE_BITS     = 6;
    static const uint8_t REGISTER_BITS   = 4;
    static const uint8_t REGISTER_COUNT  = 16;
    static const uint8_t MATRIX_DIM_BITS = 5;
    static const uint8_t MAX_MATRIX_ROWS = 20;
    static const uint8_t MAX_MATRIX_COLS = 20;

    typedef uint32_t InstrWord;
    typedef uint64_t Word;

    static const uint8_t ZERO = 0;
    static const uint8_t SP   = 13;
    static const uint8_t LR   = 14;
    static const uint8_t PC   = 15;

    enum class InstructionType   : uint8_t { ALU, MATRIX, MEMORY, BRANCH, SYSTEM };
    enum class InstructionFormat : uint8_t { R_TYPE, I_TYPE, B_TYPE, J_TYPE, M_TYPE };

    enum class Opcode : uint8_t {
        ADD = 0, SUB, MUL, DIV, MOD, AND, OR, XOR, NOT, LSL, LSR, ASR,
        SIN, COS, LOG, EXP, POW, CMP,
        MATMUL, MATADD, MATTRANSPOSE, MATINVERSE, MATADJOINT, MATDETERMINANT,
        LOAD, STORE,
        JEQ, JGT, JZ, JNZ, JMP, CALL, RET,
        NOP, HALT,
        OPCODE_COUNT
    };

    struct OpcodeMeta {
        InstructionFormat format;
        InstructionType   type;
        const char*       mnemonic;
        bool              isUnary;
    };

    static uint8_t opcodeValue(Opcode op) { return (uint8_t)op; }
    static bool isValidOpcode(uint8_t val) { return val < opcodeValue(Opcode::OPCODE_COUNT); }

    static OpcodeMeta getMeta(Opcode op) {
        if (!isValidOpcode(opcodeValue(op))) {
            throw invalid_argument("Invalid opcode");
        }
        static const OpcodeMeta table[] = {
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
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATMUL", false},
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATADD", false},
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATTRANSPOSE", false},
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATINVERSE", false},
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATADJOINT", false},
            {InstructionFormat::M_TYPE, InstructionType::MATRIX, "MATDETERMINANT", false},
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "LOAD", false},
            {InstructionFormat::I_TYPE, InstructionType::MEMORY, "STORE", false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JEQ", false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JGT", false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JZ",  false},
            {InstructionFormat::B_TYPE, InstructionType::BRANCH, "JNZ", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "JMP", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "CALL", false},
            {InstructionFormat::J_TYPE, InstructionType::BRANCH, "RET", false},
            {InstructionFormat::R_TYPE, InstructionType::SYSTEM, "NOP", false},
            {InstructionFormat::R_TYPE, InstructionType::SYSTEM, "HALT", false}
        };
        return table[opcodeValue(op)];
    }

    static InstructionFormat formatOf(Opcode op) { return getMeta(op).format; }
    static InstructionType   typeOf(Opcode op)   { return getMeta(op).type; }
    static const char*       mnemonicOf(Opcode op) { return getMeta(op).mnemonic; }
    static bool              isUnary(Opcode op)  { return getMeta(op).isUnary; }

    struct InstructionFields {
        Opcode   opcode     = Opcode::NOP;
        uint8_t  rd         = 0;
        uint8_t  rs1        = 0;
        uint8_t  rs2        = 0;
        int32_t  immediate  = 0;
        uint8_t  modifier   = 0;
        uint8_t  matrixRows = 0;
        uint8_t  matrixCols = 0;

        InstructionFormat format() const { return ISA::formatOf(opcode); }
        InstructionType   type()   const { return ISA::typeOf(opcode); }
    };

    static InstrWord encode(const InstructionFields& f) {
        if (!isValidOpcode(opcodeValue(f.opcode))) throw invalid_argument("invalid opcode");
        InstrWord word = (InstrWord)(f.opcode) << 26;

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
            case InstructionFormat::M_TYPE:
                word |= (f.rd & 0xF) << 22 | (f.rs1 & 0xF) << 18 | (f.rs2 & 0xF) << 14 |
                        (f.matrixRows & 0x1F) << 9 | (f.matrixCols & 0x1F) << 4;
                break;
        }
        return word;
    }

    static InstructionFields decode(InstrWord word) {
        uint8_t opVal = (uint8_t)((word >> 26) & 0x3F);
        if (!isValidOpcode(opVal)) throw invalid_argument("invalid opcode in word");

        InstructionFields f;
        f.opcode = (Opcode)opVal;

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
            case InstructionFormat::M_TYPE:
                f.rd         = (word >> 22) & 0xF;
                f.rs1        = (word >> 18) & 0xF;
                f.rs2        = (word >> 14) & 0xF;
                f.matrixRows = (word >> 9) & 0x1F;
                f.matrixCols = (word >> 4) & 0x1F;
                break;
        }
        return f;
    }

private:
    static int32_t signExtend(uint32_t value, uint8_t bits) {
        uint32_t signBit = 1u << (bits - 1);
        return (int32_t)((value ^ signBit) - signBit);
    }
};

// ==========================================
// Register File Class
// ==========================================
class RegisterFile {
private:
    array<ISA::Word, ISA::REGISTER_COUNT> regs;

public:
    RegisterFile() {
        reset();
    }

    void reset() {
        regs.fill(0);
        regs[ISA::SP] = 0x7FFF; // initial stack pointer
        regs[ISA::PC] = 0x0000;
    }

    ISA::Word read(uint8_t r) const {
        if (r >= ISA::REGISTER_COUNT) return 0;
        if (r == ISA::ZERO) return 0; // R0 is always 0
        return regs[r];
    }

    bool write(uint8_t r, ISA::Word val) {
        if (r >= ISA::REGISTER_COUNT) return false;
        if (r == ISA::ZERO) return true; // cannot write to R0
        regs[r] = val;
        return true;
    }

    static string getRegName(uint8_t r) {
        if (r == ISA::ZERO) return "R0(zr)";
        if (r == ISA::SP)   return "SP";
        if (r == ISA::LR)   return "LR";
        if (r == ISA::PC)   return "PC";
        return "R" + to_string(r);
    }

    static int parseRegName(string s) {
        for (auto &c : s) c = toupper(c);
        if (s == "SP" || s == "R13") return ISA::SP;
        if (s == "LR" || s == "R14") return ISA::LR;
        if (s == "PC" || s == "R15") return ISA::PC;
        if (s == "R0" || s == "ZERO") return ISA::ZERO;
        if (s.size() >= 2 && s[0] == 'R') {
            try {
                int num = stoi(s.substr(1));
                if (num >= 0 && num < ISA::REGISTER_COUNT) return num;
            } catch (...) {}
        }
        return -1;
    }
};

// ==========================================
// Disassembler Helper
// ==========================================
class Disassembler {
public:
    static string decodeToString(ISA::InstrWord word) {
        try {
            ISA::InstructionFields f = ISA::decode(word);
            string mnem = ISA::mnemonicOf(f.opcode);
            stringstream ss;
            ss << left << setw(8) << mnem;

            switch (f.format()) {
                case ISA::InstructionFormat::R_TYPE:
                    if (f.opcode == ISA::Opcode::NOP || f.opcode == ISA::Opcode::HALT) return mnem;
                    if (ISA::isUnary(f.opcode)) {
                        ss << RegisterFile::getRegName(f.rd) << ", " << RegisterFile::getRegName(f.rs1);
                    } else if (f.modifier) {
                        ss << RegisterFile::getRegName(f.rd) << ", " << RegisterFile::getRegName(f.rs1) << ", #" << f.immediate;
                    } else {
                        ss << RegisterFile::getRegName(f.rd) << ", " << RegisterFile::getRegName(f.rs1) << ", " << RegisterFile::getRegName(f.rs2);
                    }
                    break;
                case ISA::InstructionFormat::I_TYPE:
                    ss << RegisterFile::getRegName(f.rd) << ", " << f.immediate << "(" << RegisterFile::getRegName(f.rs1) << ")";
                    break;
                case ISA::InstructionFormat::B_TYPE:
                    ss << RegisterFile::getRegName(f.rs1) << ", " << RegisterFile::getRegName(f.rs2) << ", off=" << f.immediate;
                    break;
                case ISA::InstructionFormat::J_TYPE:
                    if (f.opcode == ISA::Opcode::RET) return "RET";
                    ss << "target=" << f.immediate;
                    break;
                case ISA::InstructionFormat::M_TYPE:
                    ss << RegisterFile::getRegName(f.rd) << ", " << RegisterFile::getRegName(f.rs1) << ", " << RegisterFile::getRegName(f.rs2)
                       << " [" << (int)f.matrixRows << "x" << (int)f.matrixCols << "]";
                    break;
            }
            return ss.str();
        } catch (...) {
            return "<UNKNOWN_INSTR>";
        }
    }
};

// ==========================================
// Simple CPU Model for Testing CLI
// ==========================================
struct MicroState {
    uint16_t uPC = 0;
    string controlModel = "Horizontal";
    uint32_t microWord = 0x5A3C;
    bool regWrite = true;
    bool memRead = false;
    bool memWrite = false;
    string aluOp = "ADD";
};

class SimCPU {
public:
    RegisterFile regs;
    vector<ISA::InstrWord> codeMem;
    map<ISA::Word, ISA::Word> dataMem;

    ISA::Word stackBase  = 0x7FFF;
    ISA::Word stackLimit = 0x7E00;

    bool is6Stage = false;
    vector<string> pipeStages;
    MicroState ustate;
    bool halted = false;

    SimCPU() {
        reset();
    }

    void reset() {
        regs.reset();
        halted = false;
        ustate.uPC = 0;
        updateStages();
    }

    void updateStages() {
        if (!is6Stage) {
            pipeStages = {"IF", "ID", "EX", "WB"};
        } else {
            pipeStages = {"IF", "ID", "EX", "MEM1", "MEM2", "WB"};
        }
    }

    bool step() {
        if (halted) return false;
        ISA::Word pc = regs.read(ISA::PC);
        if (pc >= codeMem.size()) {
            halted = true;
            return false;
        }

        ISA::InstrWord currentInstr = codeMem[pc];
        try {
            ISA::InstructionFields f = ISA::decode(currentInstr);
            if (f.opcode == ISA::Opcode::HALT) {
                halted = true;
                return false;
            }

            ustate.uPC = (ustate.uPC + 1) % 0x100;
            ustate.aluOp = ISA::mnemonicOf(f.opcode);

            // Basic test execution for ADD with immediate
            if (f.opcode == ISA::Opcode::ADD && f.modifier) {
                regs.write(f.rd, regs.read(f.rs1) + f.immediate);
            }

            regs.write(ISA::PC, pc + 1);
        } catch (...) {
            halted = true;
            return false;
        }
        return true;
    }

    bool isStackSafe() const {
        return regs.read(ISA::SP) >= stackLimit && regs.read(ISA::SP) <= stackBase;
    }
};

// ==========================================
// Interactive CLI Debugger (Navin's Module)
// ==========================================
class InteractiveCLI {
private:
    SimCPU cpu;
    unordered_set<ISA::Word> breakpoints;

public:
    void start() {
        loadDefaultProgram();
        cout << "====================================================\n";
        cout << "    RISC-201 CPU Simulator CLI (Navin's Module)     \n";
        cout << "====================================================\n";
        cout << "Type 'help' to see all commands. Type 'all' to print state.\n\n";

        string line;
        while (true) {
            cout << "risc201-cli> ";
            if (!getline(cin, line)) break;

            stringstream ss(line);
            string cmd;
            ss >> cmd;
            for (auto &c : cmd) c = tolower(c);

            if (cmd == "exit" || cmd == "quit") {
                cout << "Exiting CLI.\n";
                break;
            }
            else if (cmd == "regs") {
                printRegs();
            }
            else if (cmd == "set") {
                handleSet(ss);
            }
            else if (cmd == "get") {
                handleGet(ss);
            }
            else if (cmd == "step" || cmd == "s") {
                int count = 1;
                ss >> count;
                for (int i = 0; i < count; i++) {
                    if (!cpu.step()) {
                        cout << "[*] CPU halted or stopped.\n";
                        break;
                    }
                }
                printDisasm();
                printRegs();
            }
            else if (cmd == "run" || cmd == "r") {
                while (!cpu.halted) {
                    if (breakpoints.count(cpu.regs.read(ISA::PC))) {
                        cout << "[*] Hit breakpoint at address: 0x" << hex << cpu.regs.read(ISA::PC) << dec << "\n";
                        break;
                    }
                    if (!cpu.step()) break;
                }
                printRegs();
            }
            else if (cmd == "break" || cmd == "b") {
                string addrStr;
                if (ss >> addrStr) {
                    ISA::Word addr = stoull(addrStr, nullptr, 0);
                    breakpoints.insert(addr);
                    cout << "[+] Breakpoint added at 0x" << hex << addr << dec << "\n";
                } else {
                    cout << "Usage: break <address>\n";
                }
            }
            else if (cmd == "pipe") {
                printPipeline();
            }
            else if (cmd == "upc") {
                printMicrocode();
            }
            else if (cmd == "stack") {
                printStack();
            }
            else if (cmd == "disasm") {
                printDisasm();
            }
            else if (cmd == "mode") {
                int m = 4;
                if (ss >> m && (m == 4 || m == 6)) {
                    cpu.is6Stage = (m == 6);
                    cpu.updateStages();
                    cout << "[+] Switched to " << m << "-stage pipeline view.\n";
                    printPipeline();
                } else {
                    cout << "Usage: mode 4 or mode 6\n";
                }
            }
            else if (cmd == "reset") {
                cpu.reset();
                cout << "[+] CPU and registers reset.\n";
            }
            else if (cmd == "all") {
                printDisasm();
                printPipeline();
                printMicrocode();
                printRegs();
                printStack();
            }
            else if (cmd == "help") {
                printHelp();
            }
            else if (cmd.empty()) {
                continue;
            }
            else {
                cout << "Unknown command: '" << cmd << "'. Type 'help' for command list.\n";
            }
        }
    }

private:
    void handleSet(stringstream& ss) {
        string regName, valStr;
        if (ss >> regName >> valStr) {
            int r = RegisterFile::parseRegName(regName);
            if (r < 0) {
                cout << "Error: Unknown register '" << regName << "'\n";
                return;
            }
            if (r == ISA::ZERO) {
                cout << "Warning: R0 is hardwired to 0. Write ignored.\n";
                return;
            }
            try {
                ISA::Word val = stoull(valStr, nullptr, 0);
                cpu.regs.write(r, val);
                cout << "[+] " << RegisterFile::getRegName(r) << " set to 0x" << hex << val << dec << " (" << val << ")\n";
            } catch (...) {
                cout << "Error: Invalid number format.\n";
            }
        } else {
            cout << "Usage: set <reg> <value> (example: set R1 0x2A or set R2 42)\n";
        }
    }

    void handleGet(stringstream& ss) {
        string regName;
        if (ss >> regName) {
            int r = RegisterFile::parseRegName(regName);
            if (r < 0) {
                cout << "Error: Unknown register '" << regName << "'\n";
                return;
            }
            ISA::Word val = cpu.regs.read(r);
            cout << RegisterFile::getRegName(r) << " = 0x" << hex << val << dec << " (" << val << ")\n";
        } else {
            cout << "Usage: get <reg>\n";
        }
    }

    void printRegs() {
        cout << "\n--- Register File ---\n";
        for (int i = 0; i < ISA::REGISTER_COUNT; i += 2) {
            cout << left << setw(7) << RegisterFile::getRegName(i) << ": 0x" 
                 << right << hex << setfill('0') << setw(16) << cpu.regs.read(i) << dec << setfill(' ')
                 << "  |  "
                 << left << setw(7) << RegisterFile::getRegName(i + 1) << ": 0x" 
                 << right << hex << setfill('0') << setw(16) << cpu.regs.read(i + 1) << dec << setfill(' ')
                 << "\n";
        }
        cout << "---------------------\n";
    }

    void printDisasm() {
        cout << "\n--- Disassembly Window ---\n";
        ISA::Word currentPC = cpu.regs.read(ISA::PC);
        for (size_t i = 0; i < cpu.codeMem.size(); i++) {
            string arrow = (i == currentPC) ? "--> " : "    ";
            cout << arrow << "[0x" << hex << setw(4) << setfill('0') << i << dec << setfill(' ') << "] "
                 << left << setw(28) << Disassembler::decodeToString(cpu.codeMem[i])
                 << " (hex: 0x" << hex << cpu.codeMem[i] << dec << ")\n";
        }
        cout << "--------------------------\n";
    }

    void printPipeline() {
        cout << "\n--- Pipeline Stages (" << (cpu.is6Stage ? "6-Stage" : "4-Stage") << ") ---\n";
        for (size_t i = 0; i < cpu.pipeStages.size(); i++) {
            cout << "[" << cpu.pipeStages[i] << "] ";
        }
        cout << "\n----------------------------------------\n";
    }

    void printMicrocode() {
        cout << "\n--- Microcode Controller State ---\n";
        cout << "Control Mode: " << cpu.ustate.controlModel 
             << " | uPC: 0x" << hex << cpu.ustate.uPC 
             << " | MicroWord: 0x" << cpu.ustate.microWord << dec << "\n";
        cout << "Active Op: " << cpu.ustate.aluOp 
             << " | RegWrite: " << (cpu.ustate.regWrite ? "1" : "0")
             << " | MemRead: " << (cpu.ustate.memRead ? "1" : "0") 
             << " | MemWrite: " << (cpu.ustate.memWrite ? "1" : "0") << "\n";
        cout << "----------------------------------\n";
    }

    void printStack() {
        cout << "\n--- Active Stack Frame (Full-Descending) ---\n";
        if (!cpu.isStackSafe()) {
            cout << "WARNING: Stack pointer out of bounds!\n";
        }
        cout << "Stack Base: 0x" << hex << cpu.stackBase 
             << " | Limit: 0x" << cpu.stackLimit 
             << " | Current SP: 0x" << cpu.regs.read(ISA::SP) << dec << "\n";

        ISA::Word sp = cpu.regs.read(ISA::SP);
        for (ISA::Word addr = cpu.stackBase; addr >= max(cpu.stackLimit, sp - 2); addr--) {
            string tag = "";
            if (addr == cpu.stackBase) tag = " <-- Stack Base (BP)";
            else if (addr == sp)       tag = " <-- Stack Pointer (SP)";

            cout << "  0x" << hex << setw(4) << addr << ": 0x" 
                 << setw(16) << setfill('0') << cpu.dataMem[addr] << dec << setfill(' ') 
                 << tag << "\n";
            if (addr == 0) break;
        }
        cout << "--------------------------------------------\n";
    }

    void printHelp() {
        cout << "\nAvailable Commands:\n";
        cout << "  regs              : Display all 16 registers\n";
        cout << "  get <reg>         : Display single register (e.g., get R1, get SP)\n";
        cout << "  set <reg> <val>   : Modify register value (e.g., set R1 0x2A or set R2 100)\n";
        cout << "  step [n]          : Execute n instruction(s) or cycles\n";
        cout << "  run               : Run until breakpoint or HALT\n";
        cout << "  break <addr>      : Set a breakpoint at address (e.g., break 0x2)\n";
        cout << "  pipe              : Display pipeline stages\n";
        cout << "  mode <4|6>        : Switch between 4-stage and 6-stage pipeline view\n";
        cout << "  upc               : Display Micro-PC and control signals\n";
        cout << "  stack             : Render active stack memory frame\n";
        cout << "  disasm            : Show instructions in memory\n";
        cout << "  all               : Display entire CPU state at once\n";
        cout << "  reset             : Reset CPU and registers\n";
        cout << "  exit              : Exit simulator\n\n";
    }

    void loadDefaultProgram() {
        // Sample program
        ISA::InstructionFields f1;
        f1.opcode = ISA::Opcode::ADD;
        f1.rd = 1; f1.rs1 = 0; f1.modifier = 1; f1.immediate = 42; // ADD R1, R0, #42

        ISA::InstructionFields f2;
        f2.opcode = ISA::Opcode::MATMUL;
        f2.rd = 3; f2.rs1 = 1; f2.rs2 = 2; f2.matrixRows = 4; f2.matrixCols = 4;

        ISA::InstructionFields f3;
        f3.opcode = ISA::Opcode::HALT;

        cpu.codeMem.push_back(ISA::encode(f1));
        cpu.codeMem.push_back(ISA::encode(f2));
        cpu.codeMem.push_back(ISA::encode(f3));
    }
};

int main() {
    InteractiveCLI cli;
    cli.start();
    return 0;
}
