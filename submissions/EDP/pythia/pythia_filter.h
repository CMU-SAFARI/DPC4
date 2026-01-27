#ifndef DYNAMIC_SHADOW_PREFETCH_H
#define DYNAMIC_SHADOW_PREFETCH_H

#include <cstring>
#include <iostream>

#include "champsim.h"

class BloomPythiaFilter
{
public:
  // Bloom filter config: 2KB = 16384 bits
  // static constexpr int BLOOM_BYTES = 4096;
  // Filter 64KB
  static constexpr int BLOOM_BYTES = 8192;
  static constexpr int BLOOM_BITS = BLOOM_BYTES * 8;
  static constexpr uint64_t BLOOM_MASK = BLOOM_BITS - 1;
  static constexpr uint64_t L2_WINDOW = 768 * 2;
  // Bloom filter for redundancy filtering
  uint8_t bloom[BLOOM_BYTES] = {0};

  uint64_t miss_count_stamp;
  uint64_t window_start;

  // ==================== Bloom Filter ====================
  // Two hash functions for better distribution
  inline uint64_t hash1(uint64_t b_addr) { return b_addr & BLOOM_MASK; }

  inline uint64_t hash2(uint64_t b_addr)
  {
    // Mix bits: rotate and xor
    return ((b_addr >> 7) ^ (b_addr << 3) ^ (b_addr >> 13)) & BLOOM_MASK;
  }

  inline void bloomSet(uint64_t b_addr)
  {
    uint64_t h1 = hash1(b_addr);
    uint64_t h2 = hash2(b_addr);
    bloom[h1 >> 3] |= (1 << (h1 & 7));
    bloom[h2 >> 3] |= (1 << (h2 & 7));
  }

  inline void bloomClear(uint64_t b_addr)
  {
    uint64_t h1 = hash1(b_addr);
    uint64_t h2 = hash2(b_addr);
    bloom[h1 >> 3] &= ~(1 << (h1 & 7));
    bloom[h2 >> 3] &= ~(1 << (h2 & 7));
  }

  inline bool bloomTest(uint64_t b_addr)
  {
    uint64_t h1 = hash1(b_addr);
    uint64_t h2 = hash2(b_addr);
    return (bloom[h1 >> 3] & (1 << (h1 & 7))) && (bloom[h2 >> 3] & (1 << (h2 & 7)));
  }

  inline void bloomClear() { memset(bloom, 0, sizeof(bloom)); }

  /**
   * Check if prefetch should be filtered.
   * Returns true if block is likely in cache or in-flight
   */
  bool shouldPf(uint64_t addr)
  {
    // uint64_t b_addr = addr >> LOG2_BLOCK_SIZE;
    return !bloomTest(addr);
  }
};
#endif
