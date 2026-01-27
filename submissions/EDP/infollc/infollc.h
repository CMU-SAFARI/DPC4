/*
 * The Entangling Data Prefetcher (EDP)
 *
 * Submission #49
 *
 * 4 Data Prefetching Championship
 *
 * A mechanism to throttle L1D prefetcher
 */
#ifndef PREFETCHER_INFOLLC_H
#define PREFETCHER_INFOLLC_H

#include <cstdint>
#include <unordered_set>

#include "champsim.h"
#include "modules.h"

class infollc : public champsim::modules::prefetcher {
public:
  using prefetcher::prefetcher;
  bool limit_all = false;
  bool unlimit_all = true;

  std::array<bool, 16> cpu_limited = {false};
  std::unordered_set<uint64_t> mshr;
  std::array<int, 16> demand_pf_accesses = {0};
  std::array<int, 16> total_accesses = {0};

  std::array<int, 16> cores_fill = {0};
  int num_cores = 0;

  // void prefetcher_initialize() {}
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
  // void prefetcher_cycle_operate() {}
  // void prefetcher_final_stats() {}
};

#endif
