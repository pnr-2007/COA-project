#pragma once

#include "main_memory.hpp"
#include <vector>
#include <cmath>
#include <cstdint>

class L2Cache {
public:
    struct Response {
        uint64_t data = 0;
        bool hit = false;
        uint32_t stallCycles = 0;
    };

    struct TagEntry {
        bool valid = false;
        bool dirty = false;
        uint64_t tag = 0;
    };

    struct DataBlock {
        std::vector<uint64_t> words;
        explicit DataBlock(size_t wordsPerLine = 8) : words(wordsPerLine, 0) {}
    };

    L2Cache(size_t sizeBytes, size_t lineSizeBytes, size_t ways, uint32_t l2HitLatency, MainMemory& ram)
        : ways_(ways), l2HitLatency_(l2HitLatency), lineSize_(lineSizeBytes), ram_(ram) {
        
        numSets_ = (sizeBytes / lineSizeBytes) / ways;
        tagArray_.resize(numSets_, std::vector<TagEntry>(ways));
        dataArray_.resize(numSets_, std::vector<DataBlock>(ways, DataBlock(lineSizeBytes / 8)));
        
        // 7 tree bits per set for 8-way L2 cache
        plruTrees_.resize(numSets_, std::vector<uint8_t>(ways - 1, 0));
    }

    Response access(uint64_t physAddr, bool isWrite, uint64_t writeData) {
        Response resp;
        const uint64_t offsetBits = static_cast<uint64_t>(std::log2(lineSize_));
        const uint64_t indexBits = static_cast<uint64_t>(std::log2(numSets_));
        
        const uint64_t index = (physAddr >> offsetBits) & (numSets_ - 1);
        const uint64_t tag = physAddr >> (offsetBits + indexBits);
        const uint64_t wordOffset = (physAddr & (lineSize_ - 1)) >> 3;

        // Tag Check
        int hitWay = -1;
        for (size_t way = 0; way < ways_; ++way) {
            if (tagArray_[index][way].valid && tagArray_[index][way].tag == tag) {
                hitWay = static_cast<int>(way);
                break;
            }
        }

        if (hitWay != -1) {
            resp.hit = true;
            resp.stallCycles = l2HitLatency_;

            if (isWrite) {
                dataArray_[index][hitWay].words[wordOffset] = writeData;
                tagArray_[index][hitWay].dirty = true;
            } else {
                resp.data = dataArray_[index][hitWay].words[wordOffset];
            }

            updatePLRUTree(index, static_cast<size_t>(hitWay));
            return resp;
        }

        // L2 Miss -> Use Tree-PLRU to pick victim line
        resp.hit = false;
        resp.stallCycles = l2HitLatency_ + ram_.latency();

        size_t victimWay = getVictimWayPLRU(index);
        TagEntry& victimTag = tagArray_[index][victimWay];
        DataBlock& victimData = dataArray_[index][victimWay];

        if (victimTag.valid && victimTag.dirty) {
            uint64_t victimAddr = (victimTag.tag << (offsetBits + indexBits)) | (index << offsetBits);
            for (size_t i = 0; i < victimData.words.size(); ++i) {
                ram_.writeWord(victimAddr + (i * 8), victimData.words[i]);
            }
        }

        uint64_t lineBaseAddr = physAddr & ~(lineSize_ - 1);
        for (size_t i = 0; i < victimData.words.size(); ++i) {
            victimData.words[i] = ram_.readWord(lineBaseAddr + (i * 8));
        }

        victimTag.valid = true;
        victimTag.dirty = isWrite;
        victimTag.tag = tag;
        updatePLRUTree(index, victimWay);

        if (isWrite) {
            victimData.words[wordOffset] = writeData;
        } else {
            resp.data = victimData.words[wordOffset];
        }

        return resp;
    }

private:
    size_t getVictimWayPLRU(size_t setIndex) const {
        for (size_t way = 0; way < ways_; ++way) {
            if (!tagArray_[setIndex][way].valid) return way;
        }

        size_t node = 0;
        size_t levels = static_cast<size_t>(std::log2(ways_));
        for (size_t lvl = 0; lvl < levels; ++lvl) {
            uint8_t direction = plruTrees_[setIndex][node];
            node = 2 * node + 1 + direction;
        }
        return node - (ways_ - 1);
    }

    void updatePLRUTree(size_t setIndex, size_t accessedWay) {
        size_t node = (ways_ - 1) + accessedWay;
        while (node > 0) {
            size_t parent = (node - 1) / 2;
            bool isRightChild = (node % 2 == 0);
            plruTrees_[setIndex][parent] = isRightChild ? 0 : 1;
            node = parent;
        }
    }

    size_t numSets_;
    size_t ways_;
    uint32_t l2HitLatency_;
    size_t lineSize_;
    MainMemory& ram_;

    std::vector<std::vector<TagEntry>>  tagArray_;
    std::vector<std::vector<DataBlock>> dataArray_;
    std::vector<std::vector<uint8_t>>   plruTrees_;
};
