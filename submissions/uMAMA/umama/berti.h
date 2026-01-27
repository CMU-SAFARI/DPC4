//=======================================================================================//
// File             : berti/berti_helper.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 12/OCT/2025
// Description      : Implements Berti
//=======================================================================================//

#ifndef __BERTI_H__
#define __BERTI_H__

#include "berti_params.h"
#include "champsim.h"
#include "modules.h"

typedef struct __l1d_current_page_entry {
  uint64_t page_addr;                                    // 52 bits
  uint64_t ip;                                           // 10 bits
  uint64_t u_vector;                                     // 64 bits
  uint64_t first_offset;                                 // 6 bits
  int berti[L1D_CURRENT_PAGES_TABLE_NUM_BERTI];          // 70 bits
  unsigned berti_ctr[L1D_CURRENT_PAGES_TABLE_NUM_BERTI]; // 60 bits
  uint64_t last_burst;                                   // 6 bits
  uint64_t lru;                                          // 6 bits
} l1d_current_page_entry;

//------------------------------------------//
// PREVIOUS REQUESTS TABLE
//------------------------------------------//

typedef struct __l1d_prev_request_entry {
  uint64_t page_addr_pointer; // 6 bits
  uint64_t offset;            // 6 bits
  uint64_t time;              // 16 bits
} l1d_prev_request_entry;

//------------------------------------------//
// PREVIOUS PREFETCHES TABLE
//------------------------------------------//

// We do not have access to the MSHR, so we aproximate it using this structure.
typedef struct __l1d_prev_prefetch_entry {
  uint64_t page_addr_pointer; // 6 bits
  uint64_t offset;            // 6 bits
  uint64_t time_lat;          // 16 bits // time if not completed, latency if completed
  bool completed;             // 1 bit
} l1d_prev_prefetch_entry;

//------------------------------------------//
// RECORD PAGES TABLE
//------------------------------------------//

typedef struct __l1d_record_page_entry {
  uint64_t page_addr;    // 4 bytes
  uint64_t u_vector;     // 8 bytes
  uint64_t first_offset; // 6 bits
  int berti;             // 7 bits
  uint64_t lru;          // 10 bits
} l1d_record_page_entry;

//------------------------------------------//
// Berti prefetcher
//------------------------------------------//
struct berti : public champsim::modules::prefetcher {
private:
  l1d_current_page_entry l1d_current_pages_table[L1D_CURRENT_PAGES_TABLE_ENTRIES];
  l1d_prev_request_entry l1d_prev_requests_table[L1D_PREV_REQUESTS_TABLE_ENTRIES];
  uint64_t l1d_prev_requests_table_head;
  l1d_prev_prefetch_entry l1d_prev_prefetches_table[L1D_PREV_PREFETCHES_TABLE_ENTRIES];
  uint64_t l1d_prev_prefetches_table_head;
  l1d_record_page_entry l1d_record_pages_table[L1D_RECORD_PAGES_TABLE_ENTRIES];
  uint64_t l1d_ip_table[L1D_IP_TABLE_ENTRIES];

  uint64_t l1d_get_latency(uint64_t cycle, uint64_t cycle_prev);

  int l1d_calculate_stride(uint64_t prev_offset, uint64_t current_offset);

  void l1d_init_current_pages_table();
  uint64_t l1d_get_current_pages_entry(uint64_t page_addr);
  void l1d_update_lru_current_pages_table(uint64_t index);
  uint64_t l1d_get_lru_current_pages_entry();
  int l1d_get_berti_current_pages_table(uint64_t index, uint64_t& ctr);
  void l1d_add_current_pages_table(uint64_t index, uint64_t page_addr, uint64_t ip, uint64_t offset);
  uint64_t l1d_update_demand_current_pages_table(uint64_t index, uint64_t offset);
  void l1d_add_berti_current_pages_table(uint64_t index, int berti);
  bool l1d_requested_offset_current_pages_table(uint64_t index, uint64_t offset);
  void l1d_remove_current_table_entry(uint64_t index);

  void l1d_init_prev_requests_table();
  uint64_t l1d_find_prev_request_entry(uint64_t pointer, uint64_t offset);
  void l1d_add_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle);
  void l1d_reset_pointer_prev_requests(uint64_t pointer);
  uint64_t l1d_get_latency_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle);
  void l1d_get_berti_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle, int* berti);

  void l1d_init_prev_prefetches_table();
  uint64_t l1d_find_prev_prefetch_entry(uint64_t pointer, uint64_t offset);
  void l1d_add_prev_prefetches_table(uint64_t pointer, uint64_t offset, uint64_t cycle);
  void l1d_reset_pointer_prev_prefetches(uint64_t pointer);
  void l1d_reset_entry_prev_prefetches_table(uint64_t pointer, uint64_t offset);
  uint64_t l1d_get_and_set_latency_prev_prefetches_table(uint64_t pointer, uint64_t offset, uint64_t cycle);
  uint64_t l1d_get_latency_prev_prefetches_table(uint64_t pointer, uint64_t offset);

  void l1d_init_record_pages_table();
  uint64_t l1d_get_lru_record_pages_entry();
  void l1d_update_lru_record_pages_table(uint64_t index);
  void l1d_add_record_pages_table(uint64_t index, uint64_t page_addr, uint64_t vector, uint64_t first_offset, int berti);
  uint64_t l1d_get_entry_record_pages_table(uint64_t page_addr, uint64_t first_offset);
  uint64_t l1d_get_entry_record_pages_table(uint64_t page_addr);
  void l1d_copy_entries_record_pages_table(uint64_t index_from, uint64_t index_to);

  void l1d_init_ip_table();

  void l1d_record_current_page(uint64_t index_current);

public:
  using champsim::modules::prefetcher::prefetcher;

  // champsim interface prototypes
  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
};

#endif /* __BERTI_H__ */