#ifndef EMENDER_L3_H
#define EMENDER_L3_H

#include <cstdint>

#include "../pythia/pythia.h"
#include "champsim.h"
#include "modules.h"

class emender_l3 : public champsim::modules::prefetcher
{
private:
  uint64_t counter;
  int useless[4];

public:
  using prefetcher::prefetcher;

  void prefetcher_initialize();
  // void prefetcher_branch_operate(champsim::address ip, uint8_t branch_type, champsim::address branch_target) {}
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
};

#endif
