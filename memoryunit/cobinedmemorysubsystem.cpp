#pragma once

#include "main_memory.hpp"
#include "l2_cache.hpp"
#include "l1_cache.hpp"

class HierarchicalMemorySystem {
public:
    HierarchicalMemorySystem(size_t mainRamBytes = 1 << 19)
        : ram(mainRamBytes, 20),                               // 20 cycles for RAM
          l2(64 * 1024, 64, 8, 5, ram),                       // 64KB L2, 8-way, 5 cycles
          l1(16 * 1024, 16 * 1024, 64, 4, 1, l2) {}          // 16KB L1-I & L1-D, 4-way, 1 cycle

    MainMemory ram;
    L2Cache l2;
    L1Subsystem l1;

    // Helper for pipeline instruction fetch (IF stage)
    L1Cache::Response fetchInstruction(uint64_t pc) {
        return l1.iCache.access(pc, false, 0);
    }

    // Helper for pipeline memory stage (MEM stage)
    L1Cache::Response accessData(uint64_t addr, bool isWrite, uint64_t writeData) {
        return l1.dCache.access(addr, isWrite, writeData);
    }
};
