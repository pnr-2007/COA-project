#pragma once

#include "memory_unit.cpp"
#include "forwarding_unit6.cpp"

class MemoryStages6 {
public:
    using Word = ISA::Word;

    struct Mem1Result {
        bool fault = false;
        Word addr = 0;
        uint32_t cycles = 1;
    };

    struct Mem2Result {
        bool fault = false;
        Word data = 0;
        uint32_t cycles = 1;
    };

    MemoryStages6(MemoryAccessUnit& mau, size_t dataBytes) : mau_(mau), dataBytes_(dataBytes) {}

    Mem1Result address(Word base, int32_t imm) const {
        Mem1Result r;
        r.addr = base + static_cast<Word>(static_cast<int64_t>(imm));
        r.fault = (r.addr & 7) != 0 || r.addr >= dataBytes_;
        return r;
    }

    Mem2Result access(bool isWrite, Word addr, Word storeData) {
        Mem2Result r;
        const MemoryAccessUnit::Result m = mau_.access(isWrite, addr, storeData);
        r.fault = m.fault;
        r.data = m.data;
        r.cycles = m.cycles;
        return r;
    }

private:
    MemoryAccessUnit& mau_;
    size_t dataBytes_;
};
