/*
 * The Entangling Data Prefetcher (EDP)
 *
 * Submission #49
 *
 * 4 Data Prefetching Championship
 *
 * A mechanism to throttle L1D prefetcher
 */
#include "infollc.h"

#include <iostream>

#include "cache.h"

uint32_t
infollc::prefetcher_cache_operate(champsim::address addr, champsim::address ip,
                                  uint8_t cache_hit, bool useful_prefetch,
                                  access_type type, uint32_t metadata_in) {
  uint64_t block = addr.to<uint64_t>();
  if (cache_hit)
    return metadata_in;

  if (std::size(mshr) < 256 && mshr.count(block) == 0)
    mshr.insert(block);
  // assert(addr == ip); // Invariant for instruction prefetchers
  return metadata_in;
}

uint32_t infollc::prefetcher_cache_fill(champsim::address addr, long set,
                                        long way, uint8_t prefetch,
                                        champsim::address evicted_addr,
                                        uint32_t metadata_in) {

  uint64_t block = addr.to<uint64_t>();
  uint64_t blockEvict = evicted_addr.to<uint64_t>();
  auto cpu = intern_->cpu;

  uint32_t val_to_return = 0;
  if (limit_all)
    val_to_return = 0xcafe;
  else if (unlimit_all)
    val_to_return = 0xbbbb;

  cores_fill[cpu]++;
  if (cores_fill[cpu] >= 1024) {
    num_cores = 0;
    // All cores are using the llc?
    for (int i = 0; i < 16; i++) {
      if (cores_fill[i] > 64) {
        num_cores++;
      }
      cores_fill[cpu] = 0;
    }
  }

  if (mshr.count(block)) {
    demand_pf_accesses[cpu]++;
    mshr.erase(block);
  }
  total_accesses[cpu]++;
  if (total_accesses[cpu] >= 32) {
    if (demand_pf_accesses[cpu] < (total_accesses[cpu] / 3)) {
      if (num_cores > 1) {
        unlimit_all = false;
        val_to_return = 0xbeba;
        if (cpu_limited[cpu]) {
          limit_all = true;
        }
        cpu_limited[cpu] = true;
      }
    } else if (demand_pf_accesses[cpu] == total_accesses[cpu] && limit_all &&
               cpu_limited[cpu]) {
      val_to_return = 0xbbbb;
      limit_all = false;
      unlimit_all = true;
    } else if (demand_pf_accesses[cpu] == total_accesses[cpu] &&
               cpu_limited[cpu]) {
      cpu_limited[cpu] = false;
      // val_to_return = 0xbbbb;
    }
    total_accesses[cpu] = 0;
    demand_pf_accesses[cpu] = 0;
  }

  return val_to_return;
}
