#include "emender_l3.h"

void emender_l3::prefetcher_initialize()
{
  counter = 0;
  for (int i = 0; i < 4; i++) {
    useless[i] = -1;
  }
}

uint32_t emender_l3::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                              uint32_t metadata_in)
{
  int cpu_id = (metadata_in >> 28) & 3;
  uint32_t metadata_out = 0;
  // l2 reports
  if ((metadata_in >> 30) == 2) {
    int rate = metadata_in & ((1 << 28) - 1);
    useless[cpu_id] = rate;
  }
  counter++;
  if (counter % 1024 == 0) {
    printf("Useless count %d %d %d %d\n", useless[0], useless[1], useless[2], useless[3]);
  }
  return metadata_out;
}

uint32_t emender_l3::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  int cpu_id = (metadata_in >> 28) & 3;
  uint32_t metadata_out = 0;
  if ((metadata_in >> 30) == 1 || (metadata_in >> 30) == 2) {
    // from l1 or l2

    // is it the max one?
    bool is_max = true;
    bool has_smaller = false;
    for (int i = 0; i < 4; i++) {
      if (useless[i] != -1 && useless[i] < useless[cpu_id]) {
        has_smaller = true;
      }

      if (useless[i] != -1 && useless[cpu_id] < useless[i]) {
        is_max = false;
      }
    }
    // throttle for the max useless
    metadata_out = (3 << 30) | (is_max && has_smaller);
  }
  return metadata_out;
}

void emender_l3::prefetcher_cycle_operate() {}

void emender_l3::prefetcher_final_stats() {}