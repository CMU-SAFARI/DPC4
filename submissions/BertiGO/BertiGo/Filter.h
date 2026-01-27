#ifndef REGION_BITMAP_H
#define REGION_BITMAP_H

#include <cstring>
#include <iostream>
#include "champsim.h"
#include "dpc_api.h"

namespace bertigo
{

// ============================================================================
// Region-Bitmap Redundant Prefetch Filter
// ============================================================================
//
// Hardware Budget (KB)
//   - Current config (F1×1360, T29): ~15.61 KB (Fully Associative)
// Breakdown for F1×1360_T29:
//   - 1 set × 1360 ways = 1360 page entries (Fully Associative)
//   - Per entry: 29-bit tag + 64-bit bitmap = 93 bits
//   - PLRU: 1360 bits (1 bit per way)
//   - Total: 1360 × 93 + 1360 = 127,840 bits = 15,980 bytes ≈ 15.61 KB
//
// Virtual Address Breakdown (48-bit VA, 4KB pages):
//   [47:26] tag (20 bits stored)
//   [25:20] set index (6 bits, implicit in position)
//   [11:6]  block index within page (indexes into bitmap)
// ============================================================================

class Filter
{
private:
    static constexpr int LOG2_BLOCK = 6;
    static constexpr int LOG2_PAGE  = 12;
    static constexpr uint64_t BLOCK_MASK = 0x3F;

    static constexpr uint32_t MULTICORE_SIGNAL = 0xcafe;  // LLC multicore hint
    bool multicore = false;

    static constexpr int NUM_SETS = 1;
    static constexpr int NUM_WAYS = 1360;
    static constexpr int LOG2_SETS = 0;
    static constexpr int TAG_BITS = 29;

    static constexpr uint64_t SET_MASK = NUM_SETS - 1;
    static constexpr uint64_t TAG_MASK = (1ULL << TAG_BITS) - 1;

    struct Entry {
        uint64_t tag;
        uint64_t bitmap;  // One bit per cache line in page (64 bits)
    };

    Entry table[NUM_SETS][NUM_WAYS];
    uint16_t plru[NUM_SETS]; // 10-bit PLRU per set

    uint64_t stat_filter_hits;
    uint64_t stat_filter_misses;
    uint64_t stat_evictions;

    // Address decomposition
    uint64_t getVPN(uint64_t addr) const { return addr >> LOG2_PAGE; }
    uint32_t getSet(uint64_t vpn) const { return vpn & SET_MASK; }
    uint64_t getTag(uint64_t vpn) const { return (vpn >> LOG2_SETS) & TAG_MASK; }
    uint64_t getBlockOffset(uint64_t addr) const { return (addr >> LOG2_BLOCK) & BLOCK_MASK; }

    int getVictim(uint32_t set) {
        for (int way = 0; way < NUM_WAYS; way++) {
            if (!(plru[set] & (1 << way)))
                return way; // found the first not recently used
        }
        plru[set] = 0; // should not happen
        return 0;
    }

    void touchPLRU(uint32_t set, int way) {
        plru[set] |= (1 << way); // mark as recently used
        if (plru[set] == ((1 << NUM_WAYS) - 1)) // All ones
            plru[set] = (1 << way); // reset, keep this way as most recently used
    }

    int lookup(uint32_t set, uint64_t tag) {
        for (int way = 0; way < NUM_WAYS; way++) {
            if (table[set][way].bitmap != 0 && table[set][way].tag == tag)
                return way;
        }
        return -1;
    }

public:
    Filter() : stat_filter_hits(0), stat_filter_misses(0), stat_evictions(0), multicore(false) {
        memset(table, 0, sizeof(table));
        memset(plru, 0, sizeof(plru));

        std::cout << "Region-Bitmap Prefetch Filter" << std::endl;
        std::cout << "  Sets: " << NUM_SETS << ", Ways: " << NUM_WAYS << " (" << (NUM_SETS * NUM_WAYS) << " entries)" << std::endl;
        std::cout << "  Tag bits: " << TAG_BITS << ", Bitmap: 64 bits/entry, PLRU: " << NUM_WAYS << " bits/set" << std::endl;
        std::cout << "  Storage: ~" << ((NUM_SETS * NUM_WAYS * 84 + NUM_SETS * NUM_WAYS) / 8) << " bytes" << std::endl;
    }

    // Check if address is already tracked. Returns true = skip prefetch.
    bool shouldFilter(uint64_t addr) {
      if (multicore) {
        return false;
      } // Bypass filter in multicore mode
        uint64_t vpn = getVPN(addr);
        int way = lookup(getSet(vpn), getTag(vpn));
        
        if (way < 0) {
            stat_filter_misses++;
            return false;
        }

        uint64_t offset = getBlockOffset(addr);
        bool tracked = (table[getSet(vpn)][way].bitmap >> offset) & 1ULL;
        
        tracked ? stat_filter_hits++ : stat_filter_misses++;
        return tracked;
    }

    // Mark address as accessed. Call on demand access or issued prefetch.
    void markAccessed(uint64_t addr) {
        uint64_t vpn = getVPN(addr);
        uint32_t set = getSet(vpn);
        uint64_t tag = getTag(vpn);
        uint64_t offset = getBlockOffset(addr);

        int way = lookup(set, tag);

        if (way < 0) {
            // Find invalid entry or evict via PLRU
            for (int w = 0; w < NUM_WAYS; w++) {
                if (table[set][w].bitmap == 0) {
                    way = w;
                    break;
                }
            }
            if (way < 0) {
                way = getVictim(set);
                stat_evictions++;
            }
            table[set][way].tag = tag;
            table[set][way].bitmap = 0;
        }

        table[set][way].bitmap |= (1ULL << offset);
        touchPLRU(set, way);
    }

    // Clear bit on cache eviction. Entry auto-invalidates when bitmap hits zero.
    void onEviction(uint64_t addr) {
        uint64_t vpn = getVPN(addr);
        int way = lookup(getSet(vpn), getTag(vpn));
        
        if (way >= 0) {
            uint64_t offset = getBlockOffset(addr);
            table[getSet(vpn)][way].bitmap &= ~(1ULL << offset);
        }
    }

    void printStats() const {
        std::cout << "REGION_BITMAP"
                  << " filter_hits=" << stat_filter_hits
                  << " filter_misses=" << stat_filter_misses
                  << " evictions=" << stat_evictions
                  << " multicore=" << (multicore ? "ON" : "OFF")
                  << std::endl;
    }

    void onFill(uint32_t metadata_in) {
        if (metadata_in == MULTICORE_SIGNAL)
            multicore = true;
    }
};

} // namespace bertigo

#endif