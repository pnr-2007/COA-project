#pragma once

#include "l2_cache.hpp"
#include <vector>
#include <cmath>
#include <cstdint>

class L1Cache {
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

    L1Cache(size_t sizeBytes, size_t lineSizeBytes, size_t ways, uint32_t l1HitLatency, L2Cache& l2)
        : ways_(ways), l1HitLatency_(l1HitLatency), lineSize_(lineSizeBytes), l2_(l2) {
        
        numSets_ = (sizeBytes / lineSizeBytes) / ways;
        
        // Parallel Storage Arrays
        tagArray_.resize(numSets_, std::vector<TagEntry>(ways));
        dataArray_.resize(numSets_, std::vector<DataBlock>(ways, DataBlock(lineSizeBytes / 8)));

        // Tree-PLRU state: (N - 1) bit decision tree nodes per set
        plruTrees_.resize(numSets_, std::vector<uint8_t>(ways - 1, 0));
    }

    Response access(uint64_t physAddr, bool isWrite, uint64_t writeData) {
        Response resp;
        const uint64_t offsetBits = static_cast<uint64_t>(std::log2(lineSize_));
        const uint64_t indexBits = static_cast<uint64_t>(std::log2(numSets_));
        
        const uint64_t index = (physAddr >> offsetBits) & (numSets_ - 1);
        const uint64_t tag = physAddr >> (offsetBits + indexBits);
        const uint64_t wordOffset = (physAddr & (lineSize_ - 1)) >> 3;

        // --------------------------------------------------------------------
        // 1. PARALLEL TAG LOOKUP
        // --------------------------------------------------------------------
        int hitWay = -1;
        for (size_t way = 0; way < ways_; ++way) {
            if (tagArray_[index][way].valid && tagArray_[index][way].tag == tag) {
                hitWay = static_cast<int>(way);
                break;
            }
        }

        // --------------------------------------------------------------------
        // 2. CACHE HIT
        // --------------------------------------------------------------------
        if (hitWay != -1) {
            resp.hit = true;
            resp.stallCycles = l1HitLatency_;

            if (isWrite) {
                dataArray_[index][hitWay].words[wordOffset] = writeData;
                tagArray_[index][hitWay].dirty = true;
            } else {
                resp.data = dataArray_[index][hitWay].words[wordOffset];
            }

            // Update Tree-PLRU state: Point tree nodes away from the accessed way
            updatePLRUTree(index, static_cast<size_t>(hitWay));
            return resp;
        }

        // --------------------------------------------------------------------
        // 3. CACHE MISS -> TREE-PLRU VICTIM SELECTION
        // --------------------------------------------------------------------
        resp.hit = false;
        
        // Select victim way using Tree-PLRU traversal (prefer unallocated invalid lines first)
        size_t victimWay = getVictimWayPLRU(index);

        TagEntry& victimTag = tagArray_[index][victimWay];
        DataBlock& victimData = dataArray_[index][victimWay];

        // Evict dirty line if present
        if (victimTag.valid && victimTag.dirty) {
            uint64_t victimAddr = (victimTag.tag << (offsetBits + indexBits)) | (index << offsetBits);
            for (size_t i = 0; i < victimData.words.size(); ++i) {
                l2_.access(victimAddr + (i * 8), true, victimData.words[i]);
            }
        }

        // Refill from L2 Cache
        uint64_t lineBaseAddr = physAddr & ~(lineSize_ - 1);
        uint32_t totalStallCycles = l1HitLatency_;

        for (size_t i = 0; i < victimData.words.size(); ++i) {
            L2Cache::Response l2Resp = l2_.access(lineBaseAddr + (i * 8), false, 0);
            victimData.words[i] = l2Resp.data;
            if (i == 0) totalStallCycles += l2Resp.stallCycles;
        }

        // Commit tag entry & update Tree-PLRU
        victimTag.valid = true;
        victimTag.dirty = isWrite;
        victimTag.tag = tag;
        updatePLRUTree(index, victimWay);

        if (isWrite) {
            victimData.words[wordOffset] = writeData;
        } else {
            resp.data = victimData.words[wordOffset];
        }

        resp.stallCycles = totalStallCycles;
        return resp;
    }

private:
    // Traverses tree nodes down from root to locate the Least Recently Used victim way
    size_t getVictimWayPLRU(size_t setIndex) const {
        // First check if any invalid empty slots exist in the set
        for (size_t way = 0; way < ways_; ++way) {
            if (!tagArray_[setIndex][way].valid) {
                return way;
            }
        }

        // Tree Traversal: 0 points Left subtree, 1 points Right subtree
        size_t node = 0;
        size_t levels = static_cast<size_t>(std::log2(ways_));
        for (size_t lvl = 0; lvl < levels; ++lvl) {
            uint8_t direction = plruTrees_[setIndex][node];
            node = 2 * node + 1 + direction; // Move left (if 0) or right (if 1)
        }
        return node - (ways_ - 1);
    }

    // Updates tree bits along path from root to accessed way to point AWAY from it
    void updatePLRUTree(size_t setIndex, size_t accessedWay) {
        size_t node = (ways_ - 1) + accessedWay;
        while (node > 0) {
            size_t parent = (node - 1) / 2;
            bool isRightChild = (node % 2 == 0);
            // Flip parent bit to point OPPOSITE to the accessed child branch
            plruTrees_[setIndex][parent] = isRightChild ? 0 : 1;
            node = parent;
        }
    }

    size_t numSets_;
    size_t ways_;
    uint32_t l1HitLatency_;
    size_t lineSize_;
    L2Cache& l2_;

    std::vector<std::vector<TagEntry>>  tagArray_;
    std::vector<std::vector<DataBlock>> dataArray_;
    std::vector<std::vector<uint8_t>>   plruTrees_; // Tree-PLRU bit storage
};
