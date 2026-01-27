//=======================================================================================//
// File             : berti_vip_final/berti.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 12/OCT/2025
// Description      : Implements Berti VIP Final prefetcher
//                    Combines original Berti prefetcher (L1 cache) with
//                    VIP (L2 stride prefetcher) for L1 misses
//=======================================================================================//

#include "berti.h"

#include <iostream>

#include "cache.h"
#include "dpc_api.h"

// ============================================
// VIP (L2 STRIDE PREFETCHER) FUNCTIONS (Bandwidth-degree variant)
// ============================================

int berti_vip_final::l2_stride_select_degree(int confidence) const
{
  // Select prefetch degree based on confidence level
  // Higher confidence -> higher degree (more aggressive prefetching)
  //
  // Bandwidth-aware tuning using get_dram_bw():
  // - Piecewise throttling based on bw (0-15):
  //   * bw  0-10: keep degree
  //   * bw 11-12: degree /= 2
  //   * bw    13: degree /= 4
  //   * bw    14: degree /= 8
  //   * bw    15: degree = 0 (disable VIP prefetching)

  int degree = 0;

  if (confidence >= L2_STRIDE_CONFIDENCE_MAX)
    degree = L2_STRIDE_PREFETCH_DEGREE_MAX;
  else if (confidence >= 6)
    degree = L2_STRIDE_PREFETCH_DEGREE_MAX;
  else if (confidence >= 4)
    degree = L2_STRIDE_PREFETCH_DEGREE_MED;
  else if (confidence >= 2)
    degree = L2_STRIDE_PREFETCH_DEGREE_LOW;
  else if (confidence >= 1)
    degree = L2_STRIDE_PREFETCH_DEGREE_MIN;
  else
    degree = 0;

  // DRAM bandwidth feedback: throttle/disable based on bandwidth level.
  uint8_t bw = get_dram_bw();
  if (degree > 0) {
    if (bw >= 15) {
      // Very high bandwidth pressure: do not issue VIP prefetches.
      degree = 0;
    } else if (bw >= 14) {
      degree = degree / 8;
      if (degree < 1)
        degree = 1;
    } else if (bw >= 13) {
      degree = degree / 4;
      if (degree < 1)
        degree = 1;
    } else if (bw >= 11) {
      degree = degree / 2;
      if (degree < 1)
        degree = 1;
    }
  }

  return degree;
}

// ============================================
// BERTI PREFETCHER FUNCTIONS
// ============================================

void berti_vip_final::prefetcher_initialize()
{
  std::cout << " L1D Berti VIP prefetcher (Berti + L2 stride)" << std::endl;

  // Initialize Berti prefetcher tables
  l1d_init_current_pages_table();
  l1d_init_prev_requests_table();
  l1d_init_prev_prefetches_table();
  l1d_init_record_pages_table();
  l1d_init_ip_table();
  
  // VIP (L2 stride prefetcher) table is initialized in constructor
}

uint32_t berti_vip_final::prefetcher_cache_operate(champsim::address address, champsim::address ip_addr, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                         uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t ip = ip_addr.to<uint64_t>();

  // get current CPU cycle
  auto current_core_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;

  uint64_t line_addr = addr >> LOG2_BLOCK_SIZE;
  uint64_t page_addr = line_addr >> L1D_PAGE_BLOCKS_BITS;
  uint64_t offset = line_addr & L1D_PAGE_OFFSET_MASK;

  std::size_t pq_size = intern_->get_pq_size()[0];
  std::size_t pq_occupancy = intern_->get_pq_occupancy()[0];

  // ============================================
  // BERTI PREFETCHER LOGIC (Original Berti)
  // ============================================
  // This section handles L1 cache prefetching based on page access patterns
  // It is identical to the original Berti prefetcher implementation
  
  // Update current pages table
  // Find the entry
  uint64_t index = l1d_get_current_pages_entry(page_addr);

  // If not accessed recently
  if (index == L1D_CURRENT_PAGES_TABLE_ENTRIES || !l1d_requested_offset_current_pages_table(index, offset)) {
    // cout << "OFFSETS: " << hex << ip << " " << page_addr << " " << dec << offset << endl;

    if (index < L1D_CURRENT_PAGES_TABLE_ENTRIES) { // Found
      // cout << " FOUND" << endl;

      // If offset found, already requested, so return;
      if (l1d_requested_offset_current_pages_table(index, offset))
        return 0;

      uint64_t first_ip = l1d_update_demand_current_pages_table(index, offset);
      assert(l1d_ip_table[first_ip & L1D_IP_TABLE_INDEX_MASK] != L1D_IP_TABLE_NULL_POINTER);

      // Update berti
      if (cache_hit) {
        uint64_t pref_latency = l1d_get_latency_prev_prefetches_table(index, offset);
        if (pref_latency != 0) {
          // Find berti distance from pref_latency cycles before
          int b[L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS];
          l1d_get_berti_prev_requests_table(index, offset, current_core_cycle - pref_latency, b);
          for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS; i++) {
            if (b[i] == 0)
              break;
            assert(abs(b[i]) < L1D_PAGE_BLOCKS);
            l1d_add_berti_current_pages_table(index, b[i]);
          }

          // Eliminate a prev prefetch since it has been used
          l1d_reset_entry_prev_prefetches_table(index, offset);
        }
      }

      if (first_ip != ip) {
        // Assign same pointer to group IPs
        l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = l1d_ip_table[first_ip & L1D_IP_TABLE_INDEX_MASK];
      }
    } else { // Not found: Add entry

      // Find victim and clear pointers to it
      uint64_t victim_index = l1d_get_lru_current_pages_entry(); // already updates lru
      assert(victim_index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
      l1d_reset_pointer_prev_requests(victim_index);   // Not valid anymore
      l1d_reset_pointer_prev_prefetches(victim_index); // Not valid anymore

      // Copy victim to record table
      l1d_record_current_page(victim_index);

      // Add new current page
      index = victim_index;
      l1d_add_current_pages_table(index, page_addr, ip & L1D_IP_TABLE_INDEX_MASK, offset);

      // Set pointer in IP table
      uint64_t index_record = l1d_get_entry_record_pages_table(page_addr, offset);
      // The ip pointer is null
      if (l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] == L1D_IP_TABLE_NULL_POINTER) {
        if (index_record == L1D_RECORD_PAGES_TABLE_ENTRIES) { // Page not recorded
          // Get free record page pointer.
          uint64_t new_pointer = l1d_get_lru_record_pages_entry();
          l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = new_pointer;
        } else {
          l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = index_record;
        }
      } else if (l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] != index_record) {
        // If the current IP is valid, but points to another address
        // we replicate it in another record entry (lru)
        // such that the recorded page is not deleted when the current entry summarizes
        uint64_t new_pointer = l1d_get_lru_record_pages_entry();
        l1d_copy_entries_record_pages_table(l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK], new_pointer);
        l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = new_pointer;
      }
    }

    l1d_add_prev_requests_table(index, offset, current_core_cycle);

    // PREDICT
    uint64_t u_vector = 0;
    uint64_t first_offset = l1d_current_pages_table[index].first_offset;
    int b = 0;
    bool recorded = false;

    uint64_t ip_pointer = l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK];
    uint64_t pgo_pointer = l1d_get_entry_record_pages_table(page_addr, first_offset);
    uint64_t pg_pointer = l1d_get_entry_record_pages_table(page_addr);
    uint64_t berti_confidence = 0;
    int current_berti = l1d_get_berti_current_pages_table(index, berti_confidence);
    uint64_t match_confidence = 0;

    // If match with current page+first_offset, use record
    if (pgo_pointer != L1D_RECORD_PAGES_TABLE_ENTRIES
        && (l1d_record_pages_table[pgo_pointer].u_vector | l1d_current_pages_table[index].u_vector) == l1d_record_pages_table[pgo_pointer].u_vector) {
      u_vector = l1d_record_pages_table[pgo_pointer].u_vector;
      b = l1d_record_pages_table[pgo_pointer].berti;
      match_confidence = 1; // High confidence
      recorded = true;
    } else
      // If match with current ip+first_offset, use record
      if (l1d_record_pages_table[ip_pointer].first_offset == first_offset
          && (l1d_record_pages_table[ip_pointer].u_vector | l1d_current_pages_table[index].u_vector) == l1d_record_pages_table[ip_pointer].u_vector) {
        u_vector = l1d_record_pages_table[ip_pointer].u_vector;
        b = l1d_record_pages_table[ip_pointer].berti;
        match_confidence = 1; // High confidence
        recorded = true;
      } else
        // If no exact match, trust current if it has already a berti
        if (current_berti != 0 && berti_confidence >= L1D_BERTI_CTR_MED_HIGH_CONFIDENCE) { // Medium-High confidence
          u_vector = l1d_current_pages_table[index].u_vector;
          b = current_berti;
        } else
          // If match with current page, use record
          if (pg_pointer != L1D_RECORD_PAGES_TABLE_ENTRIES) { // Medium confidence
            u_vector = l1d_record_pages_table[pg_pointer].u_vector;
            b = l1d_record_pages_table[pg_pointer].berti;
            recorded = true;
          } else
            // If match with current ip, use record
            if (l1d_record_pages_table[ip_pointer].u_vector) { // Medium confidence
              u_vector = l1d_record_pages_table[ip_pointer].u_vector;
              b = l1d_record_pages_table[ip_pointer].berti;
              recorded = true;
            }

    // Burst for the first access of a page or if pending bursts
    if (first_offset == offset || l1d_current_pages_table[index].last_burst != 0) {
      uint64_t first_burst;
      if (l1d_current_pages_table[index].last_burst != 0) {
        first_burst = l1d_current_pages_table[index].last_burst;
        l1d_current_pages_table[index].last_burst = 0;
      } else if (b >= 0) {
        first_burst = offset + 1;
      } else {
        first_burst = offset - 1;
      }
      if (recorded && match_confidence) {
        int bursts = 0;
        if (b > 0) {
          for (uint64_t i = first_burst; i < offset + b; i++) {
            if ((int)i >= L1D_PAGE_BLOCKS)
              break; // Stay in the page
            // Only if previously requested and not demanded
            uint64_t pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | i;
            uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            uint64_t pf_page_addr = pf_line_addr >> L1D_PAGE_BLOCKS_BITS;
            uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << i) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                if (intern_->virtual_prefetch
                    || pf_page_addr == page_addr) { // either Berti is prefetching in virtual addr space, or it needs to obey page boundary
                  bool prefetched = intern_->prefetch_line(champsim::address{pf_addr}, true, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
                }
              } else { // record last burst
                l1d_current_pages_table[index].last_burst = i;
                break;
              }
            }
          }
        } else if (b < 0) {
          for (int i = (int)first_burst; i > ((int)offset) + b; i--) {
            if (i < 0)
              break; // Stay in the page
            // Only if previously requested and not demanded
            uint64_t pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | i;
            uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            uint64_t pf_page_addr = pf_line_addr >> L1D_PAGE_BLOCKS_BITS;
            uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << i) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                if (intern_->virtual_prefetch || pf_page_addr == page_addr) {
                  bool prefetched = intern_->prefetch_line(champsim::address{pf_addr}, true, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
                }
              } else { // record last burst
                l1d_current_pages_table[index].last_burst = i;
                break;
              }
            }
          }
        } else { // berti == 0 (zig zag of all)
          for (int i = (int)first_burst, j = (int)((first_offset << 1) - i); i < L1D_PAGE_BLOCKS || j >= 0; i++, j = (int)((first_offset << 1) - i)) {
            // Only if previously requested and not demanded
            // Dir ++
            uint64_t pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | i;
            uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            uint64_t pf_page_addr = pf_line_addr >> L1D_PAGE_BLOCKS_BITS;
            uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << i) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                if (intern_->virtual_prefetch || pf_page_addr == page_addr) {
                  bool prefetched = intern_->prefetch_line(champsim::address{pf_addr}, true, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
                }
              } else { // record last burst
                l1d_current_pages_table[index].last_burst = i;
                break;
              }
            }
            // Dir --
            pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | j;
            pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            pf_page_addr = pf_line_addr >> L1D_PAGE_BLOCKS_BITS;
            pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << j) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                if (intern_->virtual_prefetch || pf_page_addr == page_addr) {
                  bool prefetched = intern_->prefetch_line(champsim::address{pf_addr}, true, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
                }
              } else {
                // record only positive burst
              }
            }
          }
        }
      } else { // not recorded
      }
    }

    if (b != 0) {
      uint64_t pf_line_addr = line_addr + b;
      uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
      uint64_t pf_page_addr = pf_line_addr >> L1D_PAGE_BLOCKS_BITS;
      uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
      if (!l1d_requested_offset_current_pages_table(index, pf_offset)          // Only prefetch if not demanded
          && (!match_confidence || (((uint64_t)1 << pf_offset) & u_vector))) { // And prev. accessed
        if (intern_->virtual_prefetch || pf_page_addr == page_addr) {
          bool prefetched = intern_->prefetch_line(champsim::address{pf_addr}, true, 0);
          if (prefetched) {
            l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
          }
        }
      }
    }
  }

  // ============================================
  // VIP (L2 STRIDE PREFETCHER) LOGIC
  // ============================================
  // This section is ADDED for VIP functionality
  // It handles IP-stride prefetching for L1 misses (L2 accesses)
  // Uses direct array management with explicit index/tag calculation
  
  if (cache_hit == 0) { // L1 cache miss - this is an L2 access
    champsim::block_number cl_addr{address};
    uint64_t l2_ip = ip_addr.to<uint64_t>();
    
    // Calculate index and tag using bit slicing
    // Index: lower L2_STRIDE_IP_INDEX_BITS bits of IP
    // Tag: next L2_STRIDE_IP_TAG_BITS bits of IP (after index bits)
    int l2_index = l2_ip & ((1 << L2_STRIDE_IP_INDEX_BITS) - 1);
    uint64_t ip_tag = (l2_ip >> L2_STRIDE_IP_INDEX_BITS) & ((1 << L2_STRIDE_IP_TAG_BITS) - 1);
    
    // Check for tag mismatch (new/conflict IP) - conflict detection using valid bit
    if (l2_stride_table[l2_index].ip_tag != ip_tag) {
      if (l2_stride_table[l2_index].ip_valid == 0) {
        // If valid bit is zero, update with latest IP info
        l2_stride_table[l2_index].ip_tag = ip_tag;
        l2_stride_table[l2_index].last_cl_addr = cl_addr;
        l2_stride_table[l2_index].last_stride = 0;
        l2_stride_table[l2_index].confidence = 0;
        l2_stride_table[l2_index].ip_valid = 1;
      } else {
        // Otherwise, reset valid bit and leave the previous IP as it is
        l2_stride_table[l2_index].ip_valid = 0;
      }
      // No prefetching on new IP conflict
      return 0;
    } else {
      // Same IP encountered, set valid bit
      l2_stride_table[l2_index].ip_valid = 1;
    }
    
    // Calculate stride between current and last address accessed by this IP
    champsim::block_number::difference_type stride = 0;
    if (l2_stride_table[l2_index].ip_valid == 1 && l2_stride_table[l2_index].last_cl_addr != champsim::block_number{}) {
      stride = champsim::offset(l2_stride_table[l2_index].last_cl_addr, cl_addr);
    }
    
    int new_confidence = 0;
    
    // Don't do anything if same address is seen twice in a row
    if (stride == 0) {
      // Update last address but don't change confidence
      l2_stride_table[l2_index].last_cl_addr = cl_addr;
      return 0;
    }
    
    // Initialize prefetch state when stride is consistent
    // If stride matches previous stride, increase confidence and start prefetching
    if (stride != 0 && stride == l2_stride_table[l2_index].last_stride) {
      new_confidence = l2_stride_table[l2_index].confidence + 1;
      if (new_confidence > L2_STRIDE_CONFIDENCE_MAX)
        new_confidence = L2_STRIDE_CONFIDENCE_MAX;
      champsim::address next_addr{champsim::block_number{cl_addr} + stride};
      // Start prefetching regardless of page boundary (same page or cross-page)
      // The actual prefetching happens in prefetcher_cycle_operate()
      auto degree = l2_stride_select_degree(new_confidence);
      if (degree > 0)
        l2_stride_active_lookahead = {next_addr, stride, degree};
    } else if (stride != 0) {
      // New stride detected, reset confidence to 1
      new_confidence = 1;
    }
    
    // Update tracking entry with new address, stride, and confidence
    l2_stride_table[l2_index].last_cl_addr = cl_addr;
    l2_stride_table[l2_index].last_stride = stride;
    l2_stride_table[l2_index].confidence = new_confidence;
  }

  return 0;
}

uint32_t berti_vip_final::prefetcher_cache_fill(champsim::address address, long set, long way, uint8_t prefetch, champsim::address evicted_address, uint32_t metadata_in)
{
  // ============================================
  // BERTI PREFETCHER LOGIC (Original Berti)
  // ============================================
  // This section handles cache fill events for Berti prefetcher
  // It is identical to the original Berti prefetcher implementation
  
  uint64_t addr = address.to<uint64_t>();
  uint64_t evicted_addr = evicted_address.to<uint64_t>();
  auto current_core_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;

  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE);
  uint64_t page_addr = line_addr >> L1D_PAGE_BLOCKS_BITS;
  uint64_t offset = line_addr & L1D_PAGE_OFFSET_MASK;

  uint64_t pointer_prev = l1d_get_current_pages_entry(page_addr);

  if (pointer_prev < L1D_CURRENT_PAGES_TABLE_ENTRIES) { // Not found, not entry in prev requests
    uint64_t pref_latency = l1d_get_and_set_latency_prev_prefetches_table(pointer_prev, offset, current_core_cycle);
    uint64_t demand_latency = l1d_get_latency_prev_requests_table(pointer_prev, offset, current_core_cycle);

    // First look in prefetcher, since if there is a hit, it is the time the miss started
    // If no prefetch, then its latency is the demand one
    if (pref_latency == 0) {
      pref_latency = demand_latency;
    }

    if (demand_latency != 0) { // Not found, berti will not be found neither

      // Find berti (distance from pref_latency + demand_latency cycles before
      int b[L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS];
      l1d_get_berti_prev_requests_table(pointer_prev, offset, current_core_cycle - (pref_latency + demand_latency), b);
      for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS; i++) {
        if (b[i] == 0)
          break;
        assert(abs(b[i]) < L1D_PAGE_BLOCKS);
        l1d_add_berti_current_pages_table(pointer_prev, b[i]);
      }
    }
  }

  uint64_t victim_index = l1d_get_current_pages_entry(evicted_addr >> LOG2_PAGE_SIZE);
  if (victim_index < L1D_CURRENT_PAGES_TABLE_ENTRIES) {
    // Copy victim to record table
    l1d_record_current_page(victim_index);
    l1d_remove_current_table_entry(victim_index);
  }

  return 0;
}

void berti_vip_final::prefetcher_cycle_operate()
{
  // ============================================
  // VIP (L2 STRIDE PREFETCHER) LOGIC
  // ============================================
  // This function is ADDED for VIP functionality
  // It handles lookahead prefetching for L1 misses (L2 accesses)
  // Performs one prefetch per cycle to avoid overwhelming the prefetch queue
  
  // If a lookahead is active, continue prefetching along the stride pattern
  if (l2_stride_active_lookahead.has_value()) {
    auto [old_pf_address, stride, degree] = l2_stride_active_lookahead.value();
    assert(degree > 0);

    champsim::address pf_address{champsim::block_number{old_pf_address} + stride};

    champsim::page_number old_page{old_pf_address};
    champsim::page_number pf_page{pf_address};

    // Continue prefetching only within the new page (or if virtual prefetching is enabled)
    // This allows cross-page prefetching when virtual_prefetch is enabled
    if (intern_->virtual_prefetch || old_page == pf_page) {
      const bool success = prefetch_line(pf_address, false, 0);
      if (success) {
        // Prefetch succeeded, update lookahead state and decrement degree
        l2_stride_active_lookahead = {pf_address, stride, degree - 1};
      }
      // If we fail, try again next cycle (keep the same state)

      // If degree reaches 0, we've prefetched enough, reset lookahead
      if (l2_stride_active_lookahead->degree == 0) {
        l2_stride_active_lookahead.reset();
      }
    } else {
      // Page boundary crossed and virtual prefetching is disabled, stop prefetching
      l2_stride_active_lookahead.reset();
    }
  }
  
  // Note: Berti prefetcher does not use prefetcher_cycle_operate()
  // This function is only for VIP (L2 stride prefetcher) functionality
}

