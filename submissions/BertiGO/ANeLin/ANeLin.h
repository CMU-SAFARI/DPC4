#ifndef PREFETCHER_ANELIN_H
#define PREFETCHER_ANELIN_H

#include <cstdint>
#include <iostream>

#include "address.h"
#include "modules.h"

struct ANeLin : public champsim::modules::prefetcher {
  using prefetcher::prefetcher;
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);

  void prefetcher_initialize();
  // void prefetcher_branch_operate(champsim::address ip, uint8_t branch_type, champsim::address branch_target) {}
  // void prefetcher_cycle_operate() {}
  void prefetcher_final_stats();
};

#endif
