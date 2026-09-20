#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>

#include <cstdint>


using namespace std;

class CPUinstructionformat{
   private:
    struct InstructionFields {
        unit_32_t=opcode;
        unit_32_t=rd;
        unit_32_t=rs1;
        unit_32_t=rs2;
        unit_32_t=immediate;
        unit_8_t=modifier;
        unit_32_t=MatrixSize;
    };

    
     unit_64_t=vector<int> registers(16);

       static constexpr int ZERO = 0;
       static constexpr int SP = 13;
       static constexpr int LR = 14;
       static constexpr int PC = 15;

      vector<InstructionFields> instructionSet(5);
    /* instructionSet[0]=alutype instruction
    instructionSet[1]=immediatetype instruction
    instructionSet[2]=branch instruction
    instructionSet[3]=memorytype instruction
    instructionSet[4]=matrixoperation instruction
    */

    static constexpr int TOTAL_INSTRUCTIONS=40;
    enum class InstructionType {
        ALU,
        IMMEDIATE,
        BRANCH,
        JUMP,
        MEMORY
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
        POW,
        };

   
     enum  class MATRIXInstruction {
        MATMUL;
        MATADD;
        MATTRANSPOSE;
        MATINVERSE;
        MATADJOINT;
        MATDETERMINANT;
     };

     enum class MEMORYInstruction {
        LOAD;
        STORE;
        ATOMIC_MEMORY_OPS;
        SEQUENTIAL_MEMORY_OPS;
        STRING_MEMORY_OPS;
     };


     enum class BRANCHInstruction{
        /*Conditional branch*/
        JEQ;
        JGT;
        JZ;
        JNZ;

        /*Loop conditions*/
        LOOPZ;
        LOOPNZ;

        /*Unconditional branch*/
        JMP;

        /*procedural instructions*/
        CALL:
        RET;
     }

     vector<InstructionFields> getInstructionSet() const {
         return instructionSet;
     }







     
};