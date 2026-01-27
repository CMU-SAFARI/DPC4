// Based on VBerti impl from https://github.com/ChampSim/ChampSim/pull/486

#ifndef EMENDER_L1D_H_
#define EMENDER_L1D_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <queue>
#include <stdlib.h>
#include <time.h>
#include <tuple>
#include <vector>

#include "cache.h"

// KNOBS
#define ENABLE_CUCKOO_FILTER (1)
#define ENABLE_DYNAMIC_CONF (1)
#define ENABLE_L3_THROTTLE (1)
#define ENABLE_PENDING_TARGET_BUFFER (1)

namespace emender_l1d_ns
{
class vberti;
}; // namespace emender_l1d_ns

class emender_l1d : public champsim::modules::prefetcher
{
private:
  emender_l1d_ns::vberti* inner;

public:
  using prefetcher::prefetcher;

  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
};
#endif
