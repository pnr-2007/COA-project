#pragma once

// Forward declare ISA for use in method signatures
class ISA;

#include "isa.cpp"
#include "alu.cpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <cstring>

class Simulator {
public:
    static constexpr size_t DEFAULT_MEMORY_SIZE = 65536;  // 64K words

    Simulator(size_t memorySize = DEFAULT_MEMORY_SIZE)
        : memory(memorySize, 0), pc(0), halted(false), cycleCount(0) {
        registers.fill(0);
        registers[13] = memorySize - 1;  // SP = R13
        registers[15] = 0;                // PC = R15
    }

    // ---- Memory access ----

    void loadInstruction(uint64_t addr, uint32_t instr) {
        if (addr >= memory.size()) {
            throw std::runtime_error("Simulator: instruction address out of bounds");
        }
        // Store instruction in upper 32 bits of memory word at addr/2
        size_t idx = addr / 2;
        if (idx >= memory.size()) throw std::runtime_error("Simulator: memory overflow");
        
        if (addr % 2 == 0) {
            memory[idx] = (memory[idx] & 0xFFFFFFFF) | ((uint64_t)instr << 32);
        } else {
            memory[idx] = (memory[idx] & 0xFFFFFFFF00000000ULL) | instr;
        }
    }

    uint32_t fetchInstruction(uint64_t addr) {
        if (addr >= memory.size() * 2) {
            throw std::runtime_error("Simulator: instruction fetch out of bounds");
        }
        size_t idx = addr / 2;
        if (addr % 2 == 0) {
            return static_cast<ISA::InstrWord>(memory[idx] >> 32);
        } else {
            return static_cast<ISA::InstrWord>(memory[idx]);
        }
    }

    ISA::Word loadWord(uint64_t addr) {
        if (addr >= memory.size()) {
            throw std::runtime_error("Simulator: load address out of bounds");
        }
        return memory[addr];
    }

    void storeWord(uint64_t addr, ISA::Word value) {
        if (addr >= memory.size()) {
            throw std::runtime_error("Simulator: store address out of bounds");
        }
        memory[addr] = value;
    }

    // ---- Register access ----

    uint64_t getRegister(uint8_t reg) {
        if (reg >= 16) {
            throw std::runtime_error("Simulator: invalid register");
        }
        if (reg == 15) return pc;  // PC = R15
        return registers[reg];
    }

    void setRegister(uint8_t reg, uint64_t value) {
        if (reg >= 16) {
            throw std::runtime_error("Simulator: invalid register");
        }
        if (reg == 15) {  // PC = R15
            pc = value;
        } else {
            registers[reg] = value;
        }
    }

    // ---- Execution ----

    bool step() {
        if (halted) return false;

        cycleCount++;

        try {
            // Fetch
            ISA::InstrWord instrWord = fetchInstruction(pc);
            
            // Decode
            ISA::InstructionFields instr = ISA::decode(instrWord);
            
            // Execute
            executeInstruction(instr);
            
            // Update PC (unless instruction did it)
            if (instr.opcode != ISA::Opcode::JMP && 
                instr.opcode != ISA::Opcode::CALL && 
                instr.opcode != ISA::Opcode::RET &&
                instr.opcode != ISA::Opcode::JEQ &&
                instr.opcode != ISA::Opcode::JGT &&
                instr.opcode != ISA::Opcode::JZ &&
                instr.opcode != ISA::Opcode::JNZ) {
                pc += 4;  // Instruction is 32 bits
            }

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Simulator error at PC=" << std::hex << pc << ": " << e.what() << std::endl;
            halted = true;
            return false;
        }
    }

    int run() {
        while (!halted && cycleCount < 1000000) {
            if (!step()) break;
        }
        return halted ? 0 : -1;
    }

    void halt() { halted = true; }
    bool isHalted() const { return halted; }
    uint64_t getCycleCount() const { return cycleCount; }

    // ---- Debugging ----

    void dumpRegisters(std::ostream& os = std::cout) {
        os << "=== Register File ===" << std::endl;
        for (int i = 0; i < ISA::REGISTER_COUNT; ++i) {
            os << "R" << std::dec << i << " = 0x" << std::hex << std::setfill('0') 
               << std::setw(16) << registers[i] << std::endl;
        }
        os << "PC = 0x" << std::hex << pc << std::endl;
        os << "Cycles: " << std::dec << cycleCount << std::endl;
    }

    void dumpMemory(uint64_t start, uint64_t count, std::ostream& os = std::cout) {
        os << "=== Memory [0x" << std::hex << start << ", 0x" << (start + count - 1) << "] ===" << std::endl;
        for (uint64_t i = start; i < start + count && i < memory.size(); ++i) {
            os << "M[0x" << std::hex << std::setfill('0') << std::setw(8) << i 
               << "] = 0x" << std::setw(16) << memory[i] << std::endl;
        }
    }

private:
    std::array<uint64_t, 16> registers;
    std::vector<uint64_t> memory;
    uint64_t pc;
    bool halted;
    uint64_t cycleCount;

    void executeInstruction(const ISA::InstructionFields& instr) {
        using Op = ISA::Opcode;
        
        ISA::Word rs1_val = getRegister(instr.rs1);
        ISA::Word rs2_val = getRegister(instr.rs2);
        ISA::Word rd_val = 0;
        bool has_result = true;

        switch (instr.opcode) {
            // ---- Integer Arithmetic ----
            case Op::ADD: {
                ALU::Result res = ALU::add(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::SUB: {
                ALU::Result res = ALU::sub(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::MUL: {
                ALU::MultiplyResult res = ALU::multiply(rs1_val, rs2_val);
                rd_val = res.low;
                break;
            }
            case Op::DIV: {
                if (rs2_val == 0) {
                    throw std::runtime_error("divide by zero");
                }
                ALU::DivideResult res = ALU::divide(rs1_val, rs2_val);
                rd_val = res.quotient;
                break;
            }
            case Op::MOD: {
                if (rs2_val == 0) {
                    throw std::runtime_error("modulus by zero");
                }
                ALU::Result res = ALU::modulus(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }

            // ---- Bitwise Operations ----
            case Op::AND: {
                ALU::Result res = ALU::bitwiseAnd(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::OR: {
                ALU::Result res = ALU::bitwiseOr(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::XOR: {
                ALU::Result res = ALU::bitwiseXor(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::NOT: {
                ALU::Result res = ALU::bitwiseNot(rs1_val);
                rd_val = res.value;
                break;
            }

            // ---- Shifts ----
            case Op::LSL: {
                ALU::Result res = ALU::lsl(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::LSR: {
                ALU::Result res = ALU::lsr(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }
            case Op::ASR: {
                ALU::Result res = ALU::asr(rs1_val, rs2_val);
                rd_val = res.value;
                break;
            }

            // ---- Transcendental ----
            case Op::SIN: {
                ALU::Result res = ALU::evaluateSineSigned(rs1_val);
                rd_val = res.value;
                break;
            }
            case Op::COS: {
                ALU::Result res = ALU::evaluateCosineSigned(rs1_val);
                rd_val = res.value;
                break;
            }
            case Op::LOG: {
                ALU::Result res = ALU::evaluateLog2(rs1_val);
                rd_val = res.value;
                break;
            }

            // ---- Compare (store comparison result for now) ----
            case Op::CMP: {
                rd_val = (rs1_val < rs2_val) ? -1 : (rs1_val > rs2_val) ? 1 : 0;
                break;
            }

            // ---- Unimplemented for now ----
            case Op::EXP:
            case Op::POW:
                throw std::runtime_error("EXP/POW not implemented");

            // ---- Matrix Operations ----
            case Op::MATMUL: {
                executeMatmul(instr);
                has_result = false;  // Result stored directly to memory
                break;
            }
            case Op::MATTRANSPOSE: {
                executeMatTranspose(instr);
                has_result = false;
                break;
            }
            case Op::MATINVERSE: {
                executeMatInverse(instr);
                has_result = false;
                break;
            }
            case Op::MATADJOINT: {
                executeMatAdjoint(instr);
                has_result = false;
                break;
            }
            case Op::MATDETERMINANT: {
                executeMatDeterminant(instr);
                has_result = false;
                break;
            }
            case Op::MATADD:
                throw std::runtime_error("MATADD not implemented");

            // ---- Memory ----
            case Op::LOAD: {
                uint64_t addr = rs1_val + instr.immediate;
                rd_val = loadWord(addr);
                break;
            }
            case Op::STORE: {
                uint64_t addr = rs1_val + instr.immediate;
                uint64_t data = getRegister(instr.rd);   // rd is data source for STORE
                storeWord(addr, data);
                has_result = false;
                break;
            }

            // ---- Branches ----
            case Op::JEQ: {
                if (rs1_val == rs2_val) {
                    pc += instr.immediate;
                } else {
                    pc += 4;
                }
                has_result = false;
                break;
            }
            case Op::JGT: {
                if (static_cast<int64_t>(rs1_val) > static_cast<int64_t>(rs2_val)) {
                    pc += instr.immediate;
                } else {
                    pc += 4;
                }
                has_result = false;
                break;
            }
            case Op::JZ: {
                if (rs1_val == 0) {
                    pc += instr.immediate;
                } else {
                    pc += 4;
                }
                has_result = false;
                break;
            }
            case Op::JNZ: {
                if (rs1_val != 0) {
                    pc += instr.immediate;
                } else {
                    pc += 4;
                }
                has_result = false;
                break;
            }
            case Op::JMP: {
                pc += instr.immediate;
                has_result = false;
                break;
            }
            case Op::CALL: {
                setRegister(14, pc + 4);  // Return address in LR (R14)
                pc += instr.immediate;
                has_result = false;
                break;
            }
            case Op::RET: {
                pc = getRegister(14);  // Return from LR (R14)
                has_result = false;
                break;
            }

            // ---- System ----
            case Op::NOP: {
                pc += 4;
                has_result = false;
                break;
            }
            case Op::HALT: {
                halted = true;
                has_result = false;
                break;
            }

            default:
                throw std::runtime_error("Unknown opcode");
        }

        if (has_result && instr.rd != 0) {  // R0 is zero register
            setRegister(instr.rd, rd_val);
        }
    }

    void executeMatmul(const ISA::InstructionFields& instr) {
        uint64_t addr_a = getRegister(instr.rs1);
        uint64_t addr_b = getRegister(instr.rs2);
        uint64_t addr_c = getRegister(instr.rd);

        size_t rows = instr.matrixRows;
        size_t cols = instr.matrixCols;
        if (rows == 0 || cols == 0) {
            throw std::runtime_error("MATMUL: invalid dimensions");
        }

        // Assume square matrices stored row-major
        size_t n = rows;

        // Load A and B from memory
        ALU::Matrix A(n, n), B(n, n);
        for (size_t i = 0; i < n * n; ++i) {
            A.data[i] = loadWord(addr_a + i);
            B.data[i] = loadWord(addr_b + i);
        }

        // Multiply
        auto res = ALU::matMultiplyNaive(A, B);  // Use naive, not Strassen, for 20x20

        // Store result
        for (size_t i = 0; i < n * n; ++i) {
            storeWord(addr_c + i, res.value.data[i]);
        }
    }

    void executeMatTranspose(const ISA::InstructionFields& instr) {
        uint64_t addr_a = getRegister(instr.rs1);
        uint64_t addr_c = getRegister(instr.rd);

        size_t rows = instr.matrixRows;
        size_t cols = instr.matrixCols;

        ALU::Matrix A(rows, cols);
        for (size_t i = 0; i < rows * cols; ++i) {
            A.data[i] = loadWord(addr_a + i);
        }

        auto res = ALU::matTranspose(A);

        for (size_t i = 0; i < rows * cols; ++i) {
            storeWord(addr_c + i, res.value.data[i]);
        }
    }

    void executeMatInverse(const ISA::InstructionFields& instr) {
        uint64_t addr_a = getRegister(instr.rs1);
        uint64_t addr_c = getRegister(instr.rd);

        size_t n = instr.matrixRows;

        ALU::Matrix A(n, n);
        for (size_t i = 0; i < n * n; ++i) {
            A.data[i] = loadWord(addr_a + i);
        }

        auto res = ALU::matInverse(A);
        if (res.singular) {
            throw std::runtime_error("MATINVERSE: singular matrix");
        }

        for (size_t i = 0; i < n * n; ++i) {
            storeWord(addr_c + i, res.value.data[i]);
        }
    }

    void executeMatAdjoint(const ISA::InstructionFields& instr) {
        uint64_t addr_a = getRegister(instr.rs1);
        uint64_t addr_c = getRegister(instr.rd);

        size_t n = instr.matrixRows;

        ALU::Matrix A(n, n);
        for (size_t i = 0; i < n * n; ++i) {
            A.data[i] = loadWord(addr_a + i);
        }

        auto res = ALU::matAdjoint(A);

        for (size_t i = 0; i < n * n; ++i) {
            storeWord(addr_c + i, res.value.data[i]);
        }
    }

    void executeMatDeterminant(const ISA::InstructionFields& instr) {
        uint64_t addr_a = getRegister(instr.rs1);

        size_t n = instr.matrixRows;

        ALU::Matrix A(n, n);
        for (size_t i = 0; i < n * n; ++i) {
            A.data[i] = loadWord(addr_a + i);
        }

        auto res = ALU::matDeterminant(A);
        if (res.singular) {
            throw std::runtime_error("MATDETERMINANT: singular matrix");
        }

        // Store determinant in destination register
        if (instr.rd != 0) {  // R0 is zero register
            setRegister(instr.rd, res.scalar);
        }
    }
};
