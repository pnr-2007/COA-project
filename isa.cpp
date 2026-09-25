#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cstdint>

using namespace std;

class CPUInstructionFormat {

private:

    struct InstructionFields {

        uint32_t opcode = 0;
        uint32_t rd = 0;
        uint32_t rs1 = 0;
        uint32_t rs2 = 0;
        uint32_t immediate = 0;

        uint8_t modifier = 0;

        uint32_t matrixSize = 0;
    };

    vector<int64_t> registers = vector<int64_t>(16);

    static constexpr int ZERO = 0;
    static constexpr int SP = 13;
    static constexpr int LR = 14;
    static constexpr int PC = 15;

    vector<InstructionFields> instructionSet =
        vector<InstructionFields>(5);

public:

    static constexpr int TOTAL_INSTRUCTIONS = 40;

    enum class InstructionType {
        ALU,
        IMMEDIATE,
        BRANCH,
        JUMP,
        MEMORY,
        MATRIX
    };

    enum class ALUInstruction {
        ADD,
        SUB,
        MUL,
        DIV,
        MOD,
        AND,
        OR,
        XOR,
        NOT,
        LSL,
        LSR,
        ASR,
        ASL,
        SIN,
        COS,
        LOG,
        EXP,
        POW
    };

    enum class MATRIXInstruction {
        MATMUL,
        MATADD,
        MATSUB,
        MATTRANSPOSE,
        MATINVERSE,
        MATADJOINT,
        MATDETERMINANT
    };

    enum class MEMORYInstruction {
        LOAD,
        STORE,
        ATOMIC_MEMORY_OPS,
        SEQUENTIAL_MEMORY_OPS,
        STRING_MEMORY_OPS
    };

    enum class BRANCHInstruction {

        JEQ,
        JGT,
        JZ,
        JNZ,

        LOOPZ,
        LOOPNZ,

        JMP,

        CALL,
        RET
    };

    vector<InstructionFields> getInstructionSet() const {
        return instructionSet;
    }
};
