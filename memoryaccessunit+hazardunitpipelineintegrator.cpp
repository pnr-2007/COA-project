#pragma once

#include "l1_cache.hpp"
#include "l2_cache.hpp"
#include "main_memory.hpp"
#include "memory_access_unit.hpp"
#include "hazard_unit.hpp"
#include <vector>
#include <cstdint>
#include <iostream>

// ----------------------------------------------------------------------------
// PIPELINE INTER-STAGE REGISTERS
// ----------------------------------------------------------------------------
struct IF_ID_Reg {
    uint64_t pc = 0;
    uint32_t instr = 0; // 32-bit RISC-V / custom instruction
    uint8_t  rs1 = 0;
    uint8_t  rs2 = 0;
    bool     valid = false;
};

struct ID_EX_Reg {
    uint64_t pc = 0;
    uint64_t rs1Val = 0;
    uint64_t rs2Val = 0;
    int64_t  imm = 0;
    uint8_t  rs1 = 0;
    uint8_t  rs2 = 0;
    uint8_t  rd = 0;
    
    // Control Signals
    bool aluSrcImm = false;
    uint8_t aluOp = 0;
    bool memRead = false;
    bool memWrite = false;
    bool regWrite = false;
    bool isBranch = false;
    uint8_t accessSizeBytes = 8;
    bool isUnsigned = false;
    bool valid = false;
};

// EX_MEM_Register and MEM_WB_Register are defined in memory_access_unit.hpp

// ----------------------------------------------------------------------------
// FULL CPU PIPELINE TOP-LEVEL INTEGRATION
// ----------------------------------------------------------------------------
class CPUPipeline {
public:
    CPUPipeline(size_t ramSizeBytes = 1024 * 1024)
        : ram_(ramSizeBytes, 20),
          l2_(128 * 1024, 64, 8, 5, ram_),                           // 128KB L2 Cache
          l1i_(32 * 1024, 64, 4, 1, l2_),                             // 32KB L1 Instruction Cache
          l1d_(32 * 1024, 64, 4, 1, l2_),                             // 32KB L1 Data Cache
          memUnit_(l1d_),
          registers_(32, 0) {}

    // Executes a single clock cycle across all stages
    void step() {
        cycleCount_++;

        // Handle active multi-cycle cache miss stalls
        if (stallCyclesRemaining_ > 0) {
            stallCyclesRemaining_--;
            return;
        }

        // ====================================================================
        // STAGE 5: WRITE-BACK (WB)
        // ====================================================================
        if (mem_wb_reg_.regWrite && mem_wb_reg_.rd != 0) {
            uint64_t writeVal = mem_wb_reg_.memToReg ? mem_wb_reg_.memReadData : mem_wb_reg_.aluResult;
            registers_[mem_wb_reg_.rd] = writeVal;
        }

        // ====================================================================
        // STAGE 4: MEMORY ACCESS (MEM)
        // ====================================================================
        MemoryAccessUnit::StageResult memResult = memUnit_.executeCycle(ex_mem_reg_);
        
        if (memResult.isStalled) {
            // Trigger multi-cycle freeze for L2/RAM fills
            stallCyclesRemaining_ = memResult.stallCyclesNeeded - 1;
        }

        // ====================================================================
        // EVALUATE HAZARD UNIT SIGNALS
        // ====================================================================
        HazardSignals hazard = hazardUnit_.evaluate(
            if_id_reg_.rs1, if_id_reg_.rs2,
            id_ex_reg_.rs1, id_ex_reg_.rs2, id_ex_reg_.rd, id_ex_reg_.memRead,
            ex_mem_reg_.rd, ex_mem_reg_.regWrite,
            mem_wb_reg_.rd, mem_wb_reg_.regWrite,
            branchTaken_,
            memResult.stallCyclesNeeded
        );

        // ====================================================================
        // STAGE 3: EXECUTE (EX)
        // ====================================================================
        EX_MEM_Register next_ex_mem;
        branchTaken_ = false;

        if (id_ex_reg_.valid) {
            // 1. Resolve Forwarding Mux A (rs1)
            uint64_t operandA = id_ex_reg_.rs1Val;
            if (hazard.forwardA == FORWARD_EX_MEM) {
                operandA = ex_mem_reg_.aluResult;
            } else if (hazard.forwardA == FORWARD_MEM_WB) {
                operandA = mem_wb_reg_.memToReg ? mem_wb_reg_.memReadData : mem_wb_reg_.aluResult;
            }

            // 2. Resolve Forwarding Mux B (rs2)
            uint64_t operandB = id_ex_reg_.rs2Val;
            if (hazard.forwardB == FORWARD_EX_MEM) {
                operandB = ex_mem_reg_.aluResult;
            } else if (hazard.forwardB == FORWARD_MEM_WB) {
                operandB = mem_wb_reg_.memToReg ? mem_wb_reg_.memReadData : mem_wb_reg_.aluResult;
            }

            uint64_t aluOpB = id_ex_reg_.aluSrcImm ? static_cast<uint64_t>(id_ex_reg_.imm) : operandB;

            // 3. Simple ALU Execution
            uint64_t aluResult = 0;
            switch (id_ex_reg_.aluOp) {
                case 0: aluResult = operandA + aluOpB; break; // ADD / Address calc
                case 1: aluResult = operandA - aluOpB; break; // SUB
                case 2: aluResult = operandA & aluOpB; break; // AND
                case 3: aluResult = operandA | aluOpB; break; // OR
                default: aluResult = operandA + aluOpB; break;
            }

            // 4. Branch Evaluation
            if (id_ex_reg_.isBranch && (operandA == operandB)) {
                branchTaken_ = true;
                branchTargetPC_ = id_ex_reg_.pc + id_ex_reg_.imm;
            }

            // Pack EX/MEM output
            next_ex_mem.aluResult = aluResult;
            next_ex_mem.storeData = operandB;
            next_ex_mem.rd = id_ex_reg_.rd;
            next_ex_mem.memRead = id_ex_reg_.memRead;
            next_ex_mem.memWrite = id_ex_reg_.memWrite;
            next_ex_mem.regWrite = id_ex_reg_.regWrite;
            next_ex_mem.accessSizeBytes = id_ex_reg_.accessSizeBytes;
            next_ex_mem.isUnsigned = id_ex_reg_.isUnsigned;
        }

        // ====================================================================
        // STAGE 2: INSTRUCTION DECODE (ID)
        // ====================================================================
        ID_EX_Reg next_id_ex;
        if (if_id_reg_.valid) {
            next_id_ex.pc = if_id_reg_.pc;
            next_id_ex.rs1 = if_id_reg_.rs1;
            next_id_ex.rs2 = if_id_reg_.rs2;
            next_id_ex.rs1Val = registers_[if_id_reg_.rs1];
            next_id_ex.rs2Val = registers_[if_id_reg_.rs2];
            
            // Decoded mock fields from instruction bitstream
            uint32_t instr = if_id_reg_.instr;
            next_id_ex.rd = (instr >> 7) & 0x1F;
            next_id_ex.regWrite = true; 
            next_id_ex.valid = true;
        }

        // ====================================================================
        // STAGE 1: INSTRUCTION FETCH (IF)
        // ====================================================================
        IF_ID_Reg next_if_id;
        if (hazard.pcWrite) {
            uint64_t currentFetchPC = branchTaken_ ? branchTargetPC_ : pc_;
            L1Cache::Response fetchResp = l1i_.access(currentFetchPC, false, 0);

            if (fetchResp.stallCycles > 1) {
                stallCyclesRemaining_ = fetchResp.stallCycles - 1;
            }

            next_if_id.pc = currentFetchPC;
            next_if_id.instr = static_cast<uint32_t>(fetchResp.data); // Load instruction word
            next_if_id.rs1 = (next_if_id.instr >> 15) & 0x1F;
            next_if_id.rs2 = (next_if_id.instr >> 20) & 0x1F;
            next_if_id.valid = true;

            pc_ = currentFetchPC + 4; // Advance PC to next instruction boundary
        }

        // ====================================================================
        // PIPELINE REGISTER TRANSITIONS & FLUSH CONTROL
        // ====================================================================
        mem_wb_reg_ = memResult.outReg;

        if (hazard.ex_mem_flush) {
            ex_mem_reg_ = EX_MEM_Register(); // Inject NOP bubble
        } else {
            ex_mem_reg_ = next_ex_mem;
        }

        if (hazard.id_ex_flush || branchTaken_) {
            id_ex_reg_ = ID_EX_Reg();         // Inject NOP bubble
        } else if (!hazard.isStalled) {
            id_ex_reg_ = next_id_ex;
        }

        if (branchTaken_) {
            if_id_reg_ = IF_ID_Reg();         // Flush fetch register on branch mispredict
        } else if (hazard.if_id_write) {
            if_id_reg_ = next_if_id;
        }
    }

    // Direct Memory Loader for loading program binaries into RAM
    void loadProgram(uint64_t startAddr, const std::vector<uint64_t>& data) {
        for (size_t i = 0; i < data.size(); ++i) {
            ram_.writeWord(startAddr + (i * 8), data[i]);
        }
    }

    uint64_t cycleCount() const { return cycleCount_; }

private:
    uint64_t pc_ = 0;
    uint64_t branchTargetPC_ = 0;
    bool branchTaken_ = false;
    uint32_t stallCyclesRemaining_ = 0;
    uint64_t cycleCount_ = 0;

    // Memory Hierarchy Components
    MainMemory ram_;
    L2Cache    l2_;
    L1Cache    l1i_;
    L1Cache    l1d_;

    // Pipeline Sub-Units
    MemoryAccessUnit memUnit_;
    HazardUnit       hazardUnit_;

    // Architectural Register File (R0-R31)
    std::vector<uint64_t> registers_;

    // Pipeline Registers
    IF_ID_Reg       if_id_reg_;
    ID_EX_Reg       id_ex_reg_;
    EX_MEM_Register ex_mem_reg_;
    MEM_WB_Register mem_wb_reg_;
};
