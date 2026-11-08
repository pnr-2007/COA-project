#pragma once

#include "exec.cpp"
#include "memory_subsystem.cpp"

class MemoryAccessUnit {
public:
    using Word = ISA::Word;

    struct Result {
        bool fault = false;
        Word data = 0;
        uint32_t cycles = 1;
    };

    struct Fetch {
        ISA::InstrWord word = 0;
        uint32_t cycles = 1;
    };

    MemoryAccessUnit(HierarchicalMemorySystem& mem, size_t dataBytes) : mem_(mem), dataBytes_(dataBytes) {}

    Result access(bool isWrite, Word addr, Word storeData) {
        Result r;
        if ((addr & 7) != 0 || addr >= dataBytes_) { r.fault = true; return r; }
        const HierarchicalMemorySystem::DataResult d = mem_.accessData(addr, isWrite, storeData);
        r.data = d.data;
        r.cycles = d.cycles;
        return r;
    }

    Fetch fetch(Word pc) {
        const HierarchicalMemorySystem::FetchResult f = mem_.fetchInstruction(pc);
        return {f.word, f.cycles};
    }

private:
    HierarchicalMemorySystem& mem_;
    size_t dataBytes_;
};

