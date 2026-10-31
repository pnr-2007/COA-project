#pragma once

#include "l1_cache.hpp"
#include <cstdint>

// Pipeline Control & Data Structure passing into MEM stage
struct EX_MEM_Register {
    uint64_t aluResult = 0;   // Physical address for Memory ops / Value for ALU ops
    uint64_t storeData = 0;   // Register data to write to memory
    uint8_t  rd = 0;          // Destination register index
    bool     memRead = false; // Memory Read Control Signal
    bool     memWrite = false;// Memory Write Control Signal
    bool     regWrite = false;// Register File Write Control Signal
    uint8_t  accessSizeBytes = 8; // 1 (Byte), 2 (Half), 4 (Word), 8 (Double)
    bool     isUnsigned = false;  // Zero extension vs Sign extension
};

// Output pipeline register passing into WB stage
struct MEM_WB_Register {
    uint64_t memReadData = 0;
    uint64_t aluResult = 0;
    uint8_t  rd = 0;
    bool     regWrite = false;
    bool     memToReg = false;
};

class MemoryAccessUnit {
public:
    explicit MemoryAccessUnit(L1Cache& l1d) : l1dCache_(l1d) {}

    struct StageResult {
        MEM_WB_Register outReg;
        uint32_t stallCyclesNeeded = 0;
        bool isStalled = false;
    };

    StageResult executeCycle(const EX_MEM_Register& inReg) {
        StageResult result;
        result.outReg.rd = inReg.rd;
        result.outReg.regWrite = inReg.regWrite;
        result.outReg.aluResult = inReg.aluResult;
        result.outReg.memToReg = inReg.memRead;

        // Pass-through for Non-Memory Instructions (ALU Ops)
        if (!inReg.memRead && !inReg.memWrite) {
            result.isStalled = false;
            result.stallCyclesNeeded = 0;
            return result;
        }

        uint64_t physAddr = inReg.aluResult;

        // --------------------------------------------------------------------
        // LOAD OPERATION (memRead)
        // --------------------------------------------------------------------
        if (inReg.memRead) {
            L1Cache::Response resp = l1dCache_.access(physAddr, false, 0);
            
            result.stallCyclesNeeded = resp.stallCycles;
            result.isStalled = (resp.stallCycles > 1);

            // Extract sub-word data and apply sign/zero extension
            uint64_t rawWord = resp.data;
            uint32_t byteOffset = physAddr & 0x7; // Offset within 64-bit word
            uint64_t shifted = rawWord >> (byteOffset * 8);

            switch (inReg.accessSizeBytes) {
                case 1: // Byte
                    if (inReg.isUnsigned)
                        result.outReg.memReadData = shifted & 0xFFULL;
                    else
                        result.outReg.memReadData = static_cast<int64_t>(static_cast<int8_t>(shifted & 0xFFULL));
                    break;
                case 2: // Half-word (16-bit)
                    if (inReg.isUnsigned)
                        result.outReg.memReadData = shifted & 0xFFFFULL;
                    else
                        result.outReg.memReadData = static_cast<int64_t>(static_cast<int16_t>(shifted & 0xFFFFULL));
                    break;
                case 4: // Word (32-bit)
                    if (inReg.isUnsigned)
                        result.outReg.memReadData = shifted & 0xFFFFFFFFULL;
                    else
                        result.outReg.memReadData = static_cast<int64_t>(static_cast<int32_t>(shifted & 0xFFFFFFFFULL));
                    break;
                case 8: // Double-word (64-bit)
                default:
                    result.outReg.memReadData = shifted;
                    break;
            }
        }
        // --------------------------------------------------------------------
        // STORE OPERATION (memWrite)
        // --------------------------------------------------------------------
        else if (inReg.memWrite) {
            // Read target word first for sub-word byte masking
            L1Cache::Response readResp = l1dCache_.access(physAddr, false, 0);
            uint64_t currentWord = readResp.data;
            uint32_t byteOffset = physAddr & 0x7;
            uint64_t bitShift = byteOffset * 8;

            uint64_t writeMask = 0;
            switch (inReg.accessSizeBytes) {
                case 1: writeMask = 0xFFULL << bitShift; break;
                case 2: writeMask = 0xFFFFULL << bitShift; break;
                case 4: writeMask = 0xFFFFFFFFULL << bitShift; break;
                case 8: default: writeMask = ~0ULL; break;
            }

            uint64_t updatedWord = (currentWord & ~writeMask) | ((inReg.storeData << bitShift) & writeMask);

            // Write updated word back to L1 Cache
            L1Cache::Response writeResp = l1dCache_.access(physAddr, true, updatedWord);
            
            result.stallCyclesNeeded = readResp.stallCycles + writeResp.stallCycles;
            result.isStalled = (result.stallCyclesNeeded > 1);
        }

        return result;
    }

private:
    L1Cache& l1dCache_;
};
