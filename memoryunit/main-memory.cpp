#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

class MainMemory {
public:
    explicit MainMemory(size_t bytes, uint32_t accessCycles = 20)
        : words_(bytes / 8, 0), latency_(accessCycles) {
        if (bytes % 8 != 0) throw std::invalid_argument("MainMemory: size must be a multiple of 8");
    }

    uint64_t readWord(uint64_t physAddr) const { return words_[physAddr >> 3]; }
    void writeWord(uint64_t physAddr, uint64_t data) { words_[physAddr >> 3] = data; }

    uint32_t latency() const { return latency_; }
    size_t sizeBytes() const { return words_.size() * 8; }
    std::vector<uint64_t>& storage() { return words_; }
    const std::vector<uint64_t>& storage() const { return words_; }

private:
    std::vector<uint64_t> words_;
    uint32_t latency_;
};
