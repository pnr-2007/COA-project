#pragma once

#include "main_memory.cpp"
#include "l2_cache.cpp"
#include "l1_cache.cpp"
#include <vector>

class HierarchicalMemorySystem {
public:
    struct Config {
        size_t   l1Bytes = 16 * 1024;
        size_t   l2Bytes = 64 * 1024;
        size_t   lineBytes = 64;
        size_t   l1Ways = 4;
        size_t   l2Ways = 8;
        uint32_t l1Latency = 1;
        uint32_t l2Latency = 5;
        uint32_t ramLatency = 20;
        bool     timing = true;
    };

private:
    Config cfg_;
    size_t dataBytes_, instrBytes_;

public:
    struct FetchResult { uint32_t word; uint32_t cycles; };
    struct DataResult  { uint64_t data; uint32_t cycles; };

    HierarchicalMemorySystem(size_t dataBytes, size_t instrBytes, const Config& cfg)
        : cfg_(cfg), dataBytes_(dataBytes), instrBytes_(instrBytes),
          ram(dataBytes + instrBytes, cfg.ramLatency),
          l2(cfg.l2Bytes, cfg.lineBytes, cfg.l2Ways, cfg.l2Latency, ram),
          l1(cfg.l1Bytes, cfg.l1Bytes, cfg.lineBytes, cfg.l1Ways, cfg.l1Latency, l2) {
        for (size_t w = dataBytes / 8; w < ram.storage().size(); ++w) ram.storage()[w] = ~uint64_t(0);
    }

    MainMemory  ram;
    L2Cache     l2;
    L1Subsystem l1;

    FetchResult fetchInstruction(uint64_t pc) {
        const L1Cache::Response r = l1.iCache.access(dataBytes_ + pc, false, 0);
        const uint64_t word = r.data >> ((pc & 4) ? 32 : 0);
        return {static_cast<uint32_t>(word), cycles(r.stallCycles)};
    }

    DataResult accessData(uint64_t addr, bool isWrite, uint64_t writeData) {
        const L1Cache::Response r = l1.dCache.access(addr, isWrite, writeData);
        return {r.data, cycles(r.stallCycles)};
    }

    void loadInstructions(uint64_t baseBytes, const std::vector<uint32_t>& words) {
        std::vector<uint64_t>& s = ram.storage();
        for (size_t i = 0; i < words.size(); ++i) {
            const uint64_t k = (baseBytes >> 2) + i;
            uint64_t& w = s[dataBytes_ / 8 + (k >> 1)];
            const unsigned sh = (k & 1) ? 32 : 0;
            w = (w & ~(uint64_t(0xFFFFFFFFu) << sh)) | (uint64_t(words[i]) << sh);
        }
        l1.iCache.invalidateRange(dataBytes_, instrBytes_);
        l2.invalidateRange(dataBytes_, instrBytes_);
    }

    std::vector<uint64_t> dataSnapshot() {
        l1.dCache.flushAll();
        l2.flushAll();
        return std::vector<uint64_t>(ram.storage().begin(), ram.storage().begin() + dataBytes_ / 8);
    }

    void pokeData(uint64_t addr, uint64_t value) {
        l1.dCache.invalidateRange(addr, 8);
        l2.invalidateRange(addr, 8);
        ram.writeWord(addr, value);
    }

    size_t dataBytes() const { return dataBytes_; }
    const Config& config() const { return cfg_; }

private:
    uint32_t cycles(uint32_t stall) const { return cfg_.timing ? stall : 1; }

};
