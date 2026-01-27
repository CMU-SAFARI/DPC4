/*
 * The Entangling Data Prefetcher (EDP)
 *
 * Submission #49
 *
 * 4 Data Prefetching Championship
 */
#ifndef EDP_NO_H
#define EDP_NO_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <optional>
#include <set>

#include "cacheStruct.hh"
#include "champsim.h"
#include "modules.h"
class edp : public champsim::modules::prefetcher {
public:
  using prefetcher::prefetcher;

  void prefetcher_initialize();
  // void prefetcher_branch_operate(champsim::address ip, uint8_t branch_type,
  // champsim::address branch_target) {}
  uint32_t prefetcher_cache_operate(champsim::address addr,
                                    champsim::address ip, uint8_t cache_hit,
                                    bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way,
                                 uint8_t prefetch,
                                 champsim::address evicted_addr,
                                 uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
};

#endif
