#pragma once

#include "l2_cache.cpp"
#include <cstdint>
#include <stdexcept>
#include <vector>

class L1Cache {
public:
    struct Stats {
        uint64_t hits = 0;
        uint64_t misses = 0;
        uint64_t writebacks = 0;
        double hitRate() const { return (hits + misses) ? double(hits) / double(hits + misses) : 0.0; }
    };

    L1Cache(size_t sizeBytes, size_t lineBytes, size_t ways, uint32_t hitLatency, L2Cache& l2) : ways_(ways), lineBytes_(lineBytes), hitLatency_(hitLatency), l2_(l2) {
        if (!pow2(lineBytes) || lineBytes < 8 || !pow2(ways) || sizeBytes % (lineBytes * ways) != 0)
            throw std::invalid_argument("Cache: line size and ways must be powers of two and divide the size");
        numSets_ = sizeBytes / (lineBytes * ways);
        if (!pow2(numSets_)) throw std::invalid_argument("Cache: number of sets must be a power of two");
        offsetBits_ = log2u(lineBytes);
        indexBits_  = log2u(numSets_);
        sets_.assign(numSets_, std::vector<Line>(ways, Line(lineBytes / 8)));
        plru_.assign(numSets_, std::vector<uint8_t>(ways > 1 ? ways - 1 : 0, 0));
    }

    const Stats& stats() const { return stats_; }

    struct Response {
        uint64_t data = 0;
        bool hit = false;
        uint32_t stallCycles = 0;
    };

    size_t lineBytes() const { return lineBytes_; }

    void invalidateRange(uint64_t base, uint64_t bytes) {
        forEachLine(base, bytes, [&](size_t set, size_t way, uint64_t) { sets_[set][way].valid = false; sets_[set][way].dirty = false; });
    }


    Response access(uint64_t addr, bool isWrite, uint64_t writeData) {
        Response resp;
        const uint64_t set = setIndex(addr), tag = tagOf(addr), off = wordOffset(addr);
        int way = find(set, tag);
        if (way >= 0) {
            ++stats_.hits;
            resp.hit = true;
            resp.stallCycles = hitLatency_;
        } else {
            ++stats_.misses;
            way = static_cast<int>(victim(set));
            Line& v = sets_[set][way];
            if (v.valid && v.dirty) {
                l2_.accessLine(lineAddress(set, v.tag), true, v.words);
                ++stats_.writebacks;
            }
            const L2Cache::Response fill = l2_.accessLine(lineBase(addr), false, v.words);
            v.valid = true;
            v.dirty = false;
            v.tag = tag;
            resp.stallCycles = hitLatency_ + fill.stallCycles;
        }
        Line& l = sets_[set][way];
        if (isWrite) { l.words[off] = writeData; l.dirty = true; }
        else         { resp.data = l.words[off]; }
        touch(set, static_cast<size_t>(way));
        return resp;
    }

    uint32_t flushAll() {
        return flushAllTo([&](uint64_t a, std::vector<uint64_t>& w) { l2_.accessLine(a, true, w); });
    }

private:
    struct Line {
        bool valid = false, dirty = false;
        uint64_t tag = 0;
        std::vector<uint64_t> words;
        explicit Line(size_t n = 8) : words(n, 0) {}
    };

    static bool pow2(size_t v) { return v && (v & (v - 1)) == 0; }
    static unsigned log2u(size_t v) { unsigned n = 0; while (v > 1) { v >>= 1; ++n; } return n; }

    uint64_t setIndex(uint64_t a) const { return (a >> offsetBits_) & (numSets_ - 1); }
    uint64_t tagOf(uint64_t a) const { return a >> (offsetBits_ + indexBits_); }
    uint64_t wordOffset(uint64_t a) const { return (a & (lineBytes_ - 1)) >> 3; }
    uint64_t lineBase(uint64_t a) const { return a & ~uint64_t(lineBytes_ - 1); }
    uint64_t lineAddress(uint64_t set, uint64_t tag) const { return (tag << (offsetBits_ + indexBits_)) | (set << offsetBits_); }

    int find(uint64_t set, uint64_t tag) const {
        for (size_t w = 0; w < ways_; ++w)
            if (sets_[set][w].valid && sets_[set][w].tag == tag) return static_cast<int>(w);
        return -1;
    }

    size_t victim(uint64_t set) const {
        for (size_t w = 0; w < ways_; ++w)
            if (!sets_[set][w].valid) return w;
        size_t node = 0;
        for (unsigned l = 0; l < log2u(ways_); ++l) node = 2 * node + 1 + plru_[set][node];
        return node - (ways_ - 1);
    }

    void touch(uint64_t set, size_t way) {
        size_t node = (ways_ - 1) + way;
        while (node > 0) {
            const size_t parent = (node - 1) / 2;
            plru_[set][parent] = (node % 2 == 0) ? 0 : 1;
            node = parent;
        }
    }

    template <class F>
    void forEachLine(uint64_t base, uint64_t bytes, F&& f) {
        if (bytes == 0) return;
        const uint64_t last = lineBase(base + bytes - 1);
        for (uint64_t a = lineBase(base); a <= last; a += lineBytes_) {
            const uint64_t set = setIndex(a);
            const int w = find(set, tagOf(a));
            if (w >= 0) f(static_cast<size_t>(set), static_cast<size_t>(w), a);
        }
    }

    template <class Sink>
    uint32_t flushAllTo(Sink&& sink) {
        uint32_t n = 0;
        for (size_t s = 0; s < numSets_; ++s)
            for (size_t w = 0; w < ways_; ++w) {
                Line& l = sets_[s][w];
                if (l.valid && l.dirty) { sink(lineAddress(s, l.tag), l.words); l.dirty = false; ++n; ++stats_.writebacks; }
            }
        return n;
    }

    size_t numSets_ = 0, ways_, lineBytes_;
    unsigned offsetBits_ = 0, indexBits_ = 0;
    std::vector<std::vector<Line>> sets_;
    std::vector<std::vector<uint8_t>> plru_;
    Stats stats_;
    uint32_t hitLatency_;
    L2Cache& l2_;
};

struct L1Subsystem {
    L1Cache iCache;
    L1Cache dCache;
    L1Subsystem(size_t iBytes, size_t dBytes, size_t lineBytes, size_t ways, uint32_t hitLatency, L2Cache& l2)
        : iCache(iBytes, lineBytes, ways, hitLatency, l2), dCache(dBytes, lineBytes, ways, hitLatency, l2) {}
};;
