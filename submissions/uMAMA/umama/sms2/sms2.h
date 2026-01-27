//=======================================================================================//
// File             : sms/sms.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 19/AUG/2025
// Description      : Implements Spatial Memory Streaming prefetcher, ISCA'06
//=======================================================================================//

#ifndef __SMS2_H__
#define __SMS2_H__

#include <deque>
#include <vector>

#include "champsim.h"
#include "modules.h"
#include "sms2_helper.h"

#include "../bloom.h"
struct sms2 : public champsim::modules::prefetcher {
private:
  // config
  constexpr static uint32_t AT_SIZE = 48;
  constexpr static uint32_t FT_SIZE = 96;
  constexpr static uint32_t PHT_SIZE = 3072;
  constexpr static uint32_t PHT_ASSOC = 16;
  constexpr static uint32_t PHT_SETS = PHT_SIZE / PHT_ASSOC;
  constexpr static uint32_t REGION_SIZE = 2048;
  constexpr static uint32_t REGION_SIZE_LOG = 11;
  constexpr static uint32_t PREF_BUFFER_SIZE = 384;

  // internal data structures
  std::deque<FTEntry2*> filter_table;
  std::deque<ATEntry2*> acc_table;
  std::vector<std::deque<PHTEntry2*>> pht;
  uint32_t pht_sets;
  std::deque<uint64_t> pref_buffer;

  BloomFilter<L2_BLOOM_N, L2_BLOOM_M> *bloom;

  // private functions
  std::deque<FTEntry2*>::iterator search_filter_table(uint64_t page);
  std::deque<FTEntry2*>::iterator search_victim_filter_table();
  void evict_filter_table(std::deque<FTEntry2*>::iterator victim);
  void insert_filter_table(uint64_t pc, uint64_t page, uint32_t offset);

  std::deque<ATEntry2*>::iterator search_acc_table(uint64_t page);
  std::deque<ATEntry2*>::iterator search_victim_acc_table();
  void evict_acc_table(std::deque<ATEntry2*>::iterator victim);
  void update_age_acc_table(std::deque<ATEntry2*>::iterator current);
  void insert_acc_table(FTEntry2* ftentry, uint32_t offset);

  std::deque<PHTEntry2*>::iterator search_pht(uint64_t signature, uint32_t& set);
  std::deque<PHTEntry2*>::iterator search_victim_pht(int32_t set);
  void evcit_pht(int32_t set, std::deque<PHTEntry2*>::iterator victim);
  void update_age_pht(int32_t set, std::deque<PHTEntry2*>::iterator current);
  void insert_pht_table(ATEntry2* atentry);

  uint64_t create_signature(uint64_t pc, uint32_t offset);
  std::size_t generate_prefetch(uint64_t pc, uint64_t address, uint64_t page, uint32_t offset, std::vector<uint64_t>& pref_addr);
  void buffer_prefetch(std::vector<uint64_t> pref_addr);
  void issue_prefetch();

public:
  using champsim::modules::prefetcher::prefetcher;
  
  uint32_t PREF_DEGREE = 4;

  void set_bloom(BloomFilter<L2_BLOOM_N, L2_BLOOM_M> *bloom_) {
    bloom = bloom_;
  }

  // champsim interface prototypes
  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
};

#endif /* __SMS_H__ */
