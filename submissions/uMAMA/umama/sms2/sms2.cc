//=======================================================================================//
// File             : sms/sms.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 19/AUG/2025
// Description      : Implements Spatial Memory Streaming prefetcher, ISCA'06
//=======================================================================================//

#include "sms2.h"

void sms2::prefetcher_initialize()
{
  /* init PHT */
  std::deque<PHTEntry2*> d;
  pht.resize(sms2::PHT_SETS, d);
}

uint32_t sms2::prefetcher_cache_operate(champsim::address address, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                       uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t page = addr >> sms2::REGION_SIZE_LOG;
  uint32_t offset = (uint32_t)((addr >> LOG2_BLOCK_SIZE) & ((1ull << (sms2::REGION_SIZE_LOG - LOG2_BLOCK_SIZE)) - 1));
  std::vector<uint64_t> pref_addr;

  // cout << "pc " << hex << setw(16) << pc
  // 	<< " address " << hex << setw(16) << address
  // 	<< " page " << hex << setw(16) << page
  // 	<< " offset " << dec << setw(2) << offset
  // 	<< endl;

  auto at_index = search_acc_table(page);
  //   stats.at.lookup++;
  if (at_index != acc_table.end()) {
    /* accumulation table hit */
    // stats.at.hit++;
    (*at_index)->pattern[offset] = 1;
    update_age_acc_table(at_index);
  } else {
    /* search filter table */
    auto ft_index = search_filter_table(page);
    // stats.ft.lookup++;
    if (ft_index != filter_table.end()) {
      /* filter table hit */
      //   stats.ft.hit++;
      insert_acc_table((*ft_index), offset);
      evict_filter_table(ft_index);
    } else {
      /* filter table miss. Beginning of new generation. Issue prefetch */
      insert_filter_table(ip.to<uint64_t>(), page, offset);
      generate_prefetch(ip.to<uint64_t>(), addr, page, offset, pref_addr);
      buffer_prefetch(pref_addr);
    }
  }
  return 0;
}

void sms2::prefetcher_cycle_operate() { issue_prefetch(); }

uint32_t sms2::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  return 0;
}
