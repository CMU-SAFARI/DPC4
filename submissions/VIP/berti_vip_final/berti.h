//=======================================================================================//
// File             : berti_vip_final/berti.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 12/OCT/2025
// Description      : Implements Berti VIP Final prefetcher
//                    Combines original Berti prefetcher (L1 cache) with
//                    VIP (L2 stride prefetcher) for L1 misses
//=======================================================================================//

#ifndef __BERTI_VIP_FINAL_H__
#define __BERTI_VIP_FINAL_H__

#include "berti_params.h"
#include "champsim.h"
#include "modules.h"
#include <optional>
#include <cstring>

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
// Berti VIP Final prefetcher
// This prefetcher combines:
// 1. Berti prefetcher: Original Berti page-based prefetching for L1 cache
// 2. VIP (L2 stride prefetcher): IP-stride prefetcher for L1 misses (L2 accesses)
//------------------------------------------//
struct berti_vip_final : public champsim::modules::prefetcher {
private:
  // ============================================
  // BERTI PREFETCHER COMPONENTS
  // ============================================
  // These tables and functions are from the original Berti prefetcher
  // They handle L1 cache prefetching based on page access patterns
  
  l1d_current_page_entry l1d_current_pages_table[L1D_CURRENT_PAGES_TABLE_ENTRIES];
  l1d_prev_request_entry l1d_prev_requests_table[L1D_PREV_REQUESTS_TABLE_ENTRIES];
  uint64_t l1d_prev_requests_table_head;
  l1d_prev_prefetch_entry l1d_prev_prefetches_table[L1D_PREV_PREFETCHES_TABLE_ENTRIES];
  uint64_t l1d_prev_prefetches_table_head;
  l1d_record_page_entry l1d_record_pages_table[L1D_RECORD_PAGES_TABLE_ENTRIES];
  uint64_t l1d_ip_table[L1D_IP_TABLE_ENTRIES];

  // ============================================
  // VIP (L2 STRIDE PREFETCHER) COMPONENTS
  // ============================================
  // These components are ADDED for VIP functionality
  // They handle IP-stride prefetching for L1 misses (L2 accesses)
  // Uses direct array management with explicit index/tag calculation
  struct l2_stride_tracker_entry {
    uint64_t ip_tag = 0;                                    // IP tag (upper bits after index) - 6 bits needed
    champsim::block_number last_cl_addr{};                  // the last address accessed by this IP - (address::bits - LOG2_BLOCK_SIZE) 42 bits needed 
    champsim::block_number::difference_type last_stride{};  // the stride between the last two addresses accessed by this IP - signed, 8bits needed
    int confidence = 0;                                      // number of consecutive stride matches - 4 bits needed
    uint64_t ip_valid = 0;                                  // Valid bit for conflict detection - 1 bit needed
    // Total per entry (minimum): 6 + 42 + 8 + 4 + 1 = 61 bits
    // Total for 64 entries (minimum): 61 * 64 = 3,904 bits (488 bytes)
  };

  struct l2_stride_lookahead_entry {
    champsim::address address{};
    champsim::address::difference_type stride{};
    int degree = 0; // degree remaining
  };

  constexpr static std::size_t L2_STRIDE_TRACKER_SETS = 64;
  constexpr static std::size_t L2_STRIDE_TRACKER_WAYS = 1;
  constexpr static int L2_STRIDE_IP_INDEX_BITS = 6;        // 6 bits for 64 sets
  constexpr static int L2_STRIDE_IP_TAG_BITS = 6;          // Tag bits for IP matching
  constexpr static int L2_STRIDE_PREFETCH_DEGREE_MAX = 64;
  constexpr static int L2_STRIDE_PREFETCH_DEGREE_MED = L2_STRIDE_PREFETCH_DEGREE_MAX / 2;
  constexpr static int L2_STRIDE_PREFETCH_DEGREE_LOW =
    (L2_STRIDE_PREFETCH_DEGREE_MAX / 4) > 0 ? (L2_STRIDE_PREFETCH_DEGREE_MAX / 4) : 1;
  constexpr static int L2_STRIDE_PREFETCH_DEGREE_MIN =
    (L2_STRIDE_PREFETCH_DEGREE_MAX / 8) > 0 ? (L2_STRIDE_PREFETCH_DEGREE_MAX / 8) : 1;
  constexpr static int L2_STRIDE_CONFIDENCE_MAX = 8;

  std::optional<l2_stride_lookahead_entry> l2_stride_active_lookahead;
  l2_stride_tracker_entry l2_stride_table[L2_STRIDE_TRACKER_SETS];

  // ============================================
  // BERTI PREFETCHER HELPER FUNCTIONS
  // ============================================
  // All functions below are from the original Berti prefetcher
  
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
  
  // ============================================
  // VIP (L2 STRIDE PREFETCHER) HELPER FUNCTIONS
  // ============================================
  // Function for VIP functionality only
  
  int l2_stride_select_degree(int confidence) const;

public:
  explicit berti_vip_final(CACHE* cache) : champsim::modules::prefetcher(cache),
    l1d_prev_requests_table_head(0),
    l1d_prev_prefetches_table_head(0)
  {
    // Initialize Berti prefetcher tables
    std::memset(l1d_current_pages_table, 0, sizeof(l1d_current_pages_table));
    std::memset(l1d_prev_requests_table, 0, sizeof(l1d_prev_requests_table));
    std::memset(l1d_prev_prefetches_table, 0, sizeof(l1d_prev_prefetches_table));
    std::memset(l1d_record_pages_table, 0, sizeof(l1d_record_pages_table));
    std::memset(l1d_ip_table, 0, sizeof(l1d_ip_table));
    
    // Initialize VIP (L2 stride prefetcher) table
    // Use value-initialization (not memset) for non-trivial types
    for (std::size_t i = 0; i < L2_STRIDE_TRACKER_SETS; i++) {
      l2_stride_table[i] = l2_stride_tracker_entry{};
    }
  }

  // champsim interface prototypes
  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
};

#endif /* __BERTI_VIP_FINAL_H__ */

