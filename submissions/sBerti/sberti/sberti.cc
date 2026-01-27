//=======================================================================================//
// File             : sberti.cc
// Description      : Implementation of sBerti (Smart Stride + Berti)
//=======================================================================================//

#include "sberti.h"
#include "cache.h"
#include <cassert>
#include <iostream>
#include <cstdlib>

// =====================================================================
// SHARED / HELPER FUNCTIONS
// =====================================================================

void sberti::addRecentPrefetch(uint64_t addr) {
    recent_prefetches[recent_head] = addr;
    recent_head = (recent_head + 1) % RECENT_WINDOW_SIZE;
}

bool sberti::hasRecentlyPrefetched(uint64_t addr) {
    for (int i = 0; i < RECENT_WINDOW_SIZE; i++) {
        if (recent_prefetches[i] == addr) return true;
    }
    return false;
}

// Global issue wrapper to ensure both prefetchers acknowledge the issue
bool sberti::issue_prefetch(champsim::address addr, uint64_t ip, uint32_t metadata_in) {
    uint64_t addr_val = addr.to<uint64_t>();
    
    // Shared Deduplication: Prevents redundant issues from both Berti and Stride
    if (hasRecentlyPrefetched(addr_val)) return false;

    // Issue to L1D
    bool prefetched = intern_->prefetch_line(addr, true, metadata_in);
    
    if (prefetched) {
        addRecentPrefetch(addr_val);
    }
    return prefetched;
}

// =====================================================================
// BERTI HELPER IMPLEMENTATIONS
// =====================================================================

uint64_t sberti::l1d_get_latency(uint64_t cycle, uint64_t cycle_prev)
{
  return cycle - cycle_prev;
}

int sberti::l1d_calculate_stride(uint64_t prev_offset, uint64_t current_offset)
{
  int stride;
  if (current_offset > prev_offset) {
    stride = (int)(current_offset - prev_offset);
  } else {
    stride = (int)(prev_offset - current_offset);
    stride *= -1;
  }
  return stride;
}

// --- CURRENT PAGES TABLE ---

void sberti::l1d_init_current_pages_table()
{
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_ENTRIES; i++) {
    l1d_current_pages_table[i].page_addr = 0;
    l1d_current_pages_table[i].ip = 0;
    l1d_current_pages_table[i].u_vector = 0; 
    l1d_current_pages_table[i].last_burst = 0;
    l1d_current_pages_table[i].lru = i;
  }
}

uint64_t sberti::l1d_get_current_pages_entry(uint64_t page_addr)
{
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_ENTRIES; i++) {
    if (l1d_current_pages_table[i].page_addr == page_addr)
      return i;
  }
  return L1D_CURRENT_PAGES_TABLE_ENTRIES;
}

void sberti::l1d_update_lru_current_pages_table(uint64_t index)
{
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_ENTRIES; i++) {
    if (l1d_current_pages_table[i].lru < l1d_current_pages_table[index].lru) { 
      l1d_current_pages_table[i].lru++;
    }
  }
  l1d_current_pages_table[index].lru = 0;
}

uint64_t sberti::l1d_get_lru_current_pages_entry()
{
  uint64_t lru = L1D_CURRENT_PAGES_TABLE_ENTRIES;
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_ENTRIES; i++) {
    l1d_current_pages_table[i].lru++;
    if (l1d_current_pages_table[i].lru == L1D_CURRENT_PAGES_TABLE_ENTRIES) {
      l1d_current_pages_table[i].lru = 0;
      lru = i;
    }
  }
  assert(lru != L1D_CURRENT_PAGES_TABLE_ENTRIES);
  return lru;
}

int sberti::l1d_get_berti_current_pages_table(uint64_t index, uint64_t& ctr)
{
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  uint64_t max_score = 0;
  uint64_t b = 0;
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI; i++) {
    uint64_t score;
    score = l1d_current_pages_table[index].berti_ctr[i];
    if (score > max_score) {
      b = l1d_current_pages_table[index].berti[i];
      max_score = score;
      ctr = l1d_current_pages_table[index].berti_ctr[i];
    }
  }
  return (int)b;
}

void sberti::l1d_add_current_pages_table(uint64_t index, uint64_t page_addr, uint64_t ip, uint64_t offset)
{
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  l1d_current_pages_table[index].page_addr = page_addr;
  l1d_current_pages_table[index].ip = ip;
  l1d_current_pages_table[index].u_vector = (uint64_t)1 << offset;
  l1d_current_pages_table[index].first_offset = offset;
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI; i++) {
    l1d_current_pages_table[index].berti_ctr[i] = 0;
  }
  l1d_current_pages_table[index].last_burst = 0;
}

uint64_t sberti::l1d_update_demand_current_pages_table(uint64_t index, uint64_t offset)
{
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  l1d_current_pages_table[index].u_vector |= (uint64_t)1 << offset;
  l1d_update_lru_current_pages_table(index);
  return l1d_current_pages_table[index].ip;
}

void sberti::l1d_add_berti_current_pages_table(uint64_t index, int b)
{
  assert(b != 0);
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI; i++) {
    if (l1d_current_pages_table[index].berti_ctr[i] == 0) {
      l1d_current_pages_table[index].berti[i] = b;
      l1d_current_pages_table[index].berti_ctr[i] = 1;
      break;
    } else if (l1d_current_pages_table[index].berti[i] == b) {
      l1d_current_pages_table[index].berti_ctr[i]++;
      break;
    }
  }
  l1d_update_lru_current_pages_table(index);
}

bool sberti::l1d_requested_offset_current_pages_table(uint64_t index, uint64_t offset)
{
  assert(index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
  return l1d_current_pages_table[index].u_vector & ((uint64_t)1 << offset);
}

void sberti::l1d_remove_current_table_entry(uint64_t index)
{
  l1d_current_pages_table[index].page_addr = 0;
  l1d_current_pages_table[index].u_vector = 0;
  l1d_current_pages_table[index].berti[0] = 0;
}

// --- PREVIOUS REQUESTS TABLE ---

void sberti::l1d_init_prev_requests_table()
{
  l1d_prev_requests_table_head = 0;
  for (int i = 0; i < L1D_PREV_REQUESTS_TABLE_ENTRIES; i++) {
    l1d_prev_requests_table[i].page_addr_pointer = L1D_PREV_REQUESTS_TABLE_NULL_POINTER;
  }
}

uint64_t sberti::l1d_find_prev_request_entry(uint64_t pointer, uint64_t offset)
{
  for (int i = 0; i < L1D_PREV_REQUESTS_TABLE_ENTRIES; i++) {
    if (l1d_prev_requests_table[i].page_addr_pointer == pointer && l1d_prev_requests_table[i].offset == offset)
      return i;
  }
  return L1D_PREV_REQUESTS_TABLE_ENTRIES;
}

void sberti::l1d_add_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle)
{
  if (l1d_find_prev_request_entry(pointer, offset) != L1D_PREV_REQUESTS_TABLE_ENTRIES)
    return;

  l1d_prev_requests_table[l1d_prev_requests_table_head].page_addr_pointer = pointer;
  l1d_prev_requests_table[l1d_prev_requests_table_head].offset = offset;
  l1d_prev_requests_table[l1d_prev_requests_table_head].time = cycle & L1D_TIME_MASK;
  l1d_prev_requests_table_head = (l1d_prev_requests_table_head + 1) & L1D_PREV_REQUESTS_TABLE_MASK;
}

void sberti::l1d_reset_pointer_prev_requests(uint64_t pointer)
{
  for (int i = 0; i < L1D_PREV_REQUESTS_TABLE_ENTRIES; i++) {
    if (l1d_prev_requests_table[i].page_addr_pointer == pointer) {
      l1d_prev_requests_table[i].page_addr_pointer = L1D_PREV_REQUESTS_TABLE_NULL_POINTER;
    }
  }
}

uint64_t sberti::l1d_get_latency_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle)
{
  uint64_t index = l1d_find_prev_request_entry(pointer, offset);
  if (index == L1D_PREV_REQUESTS_TABLE_ENTRIES)
    return 0;
  return l1d_get_latency(cycle, l1d_prev_requests_table[index].time);
}

void sberti::l1d_get_berti_prev_requests_table(uint64_t pointer, uint64_t offset, uint64_t cycle, int* b)
{
  int my_pos = 0;
  uint64_t extra_time = 0;
  uint64_t last_time = l1d_prev_requests_table[(l1d_prev_requests_table_head + L1D_PREV_REQUESTS_TABLE_MASK) & L1D_PREV_REQUESTS_TABLE_MASK].time;
  for (uint64_t i = (l1d_prev_requests_table_head + L1D_PREV_REQUESTS_TABLE_MASK) & L1D_PREV_REQUESTS_TABLE_MASK; i != l1d_prev_requests_table_head;
       i = (i + L1D_PREV_REQUESTS_TABLE_MASK) & L1D_PREV_REQUESTS_TABLE_MASK) {
    if (last_time < l1d_prev_requests_table[i].time) {
      extra_time = L1D_TIME_OVERFLOW;
    }
    last_time = l1d_prev_requests_table[i].time;
    if (l1d_prev_requests_table[i].page_addr_pointer == pointer) {
      if (l1d_prev_requests_table[i].time <= (cycle & L1D_TIME_MASK) + extra_time) {
        b[my_pos] = l1d_calculate_stride(l1d_prev_requests_table[i].offset, offset);
        my_pos++;
        if (my_pos == L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS)
          return;
      }
    }
  }
  b[my_pos] = 0;
}

// --- PREVIOUS PREFETCHES TABLE ---

void sberti::l1d_init_prev_prefetches_table()
{
  l1d_prev_prefetches_table_head = 0;
  for (int i = 0; i < L1D_PREV_PREFETCHES_TABLE_ENTRIES; i++) {
    l1d_prev_prefetches_table[i].page_addr_pointer = L1D_PREV_PREFETCHES_TABLE_NULL_POINTER;
  }
}

uint64_t sberti::l1d_find_prev_prefetch_entry(uint64_t pointer, uint64_t offset)
{
  for (int i = 0; i < L1D_PREV_PREFETCHES_TABLE_ENTRIES; i++) {
    if (l1d_prev_prefetches_table[i].page_addr_pointer == pointer && l1d_prev_prefetches_table[i].offset == offset)
      return i;
  }
  return L1D_PREV_PREFETCHES_TABLE_ENTRIES;
}

void sberti::l1d_add_prev_prefetches_table(uint64_t pointer, uint64_t offset, uint64_t cycle)
{
  if (l1d_find_prev_prefetch_entry(pointer, offset) != L1D_PREV_PREFETCHES_TABLE_ENTRIES)
    return;

  l1d_prev_prefetches_table[l1d_prev_prefetches_table_head].page_addr_pointer = pointer;
  l1d_prev_prefetches_table[l1d_prev_prefetches_table_head].offset = offset;
  l1d_prev_prefetches_table[l1d_prev_prefetches_table_head].time_lat = cycle & L1D_TIME_MASK;
  l1d_prev_prefetches_table[l1d_prev_prefetches_table_head].completed = false;
  l1d_prev_prefetches_table_head = (l1d_prev_prefetches_table_head + 1) & L1D_PREV_PREFETCHES_TABLE_MASK;
}

void sberti::l1d_reset_pointer_prev_prefetches(uint64_t pointer)
{
  for (int i = 0; i < L1D_PREV_PREFETCHES_TABLE_ENTRIES; i++) {
    if (l1d_prev_prefetches_table[i].page_addr_pointer == pointer) {
      l1d_prev_prefetches_table[i].page_addr_pointer = L1D_PREV_PREFETCHES_TABLE_NULL_POINTER;
    }
  }
}

void sberti::l1d_reset_entry_prev_prefetches_table(uint64_t pointer, uint64_t offset)
{
  uint64_t index = l1d_find_prev_prefetch_entry(pointer, offset);
  if (index != L1D_PREV_PREFETCHES_TABLE_ENTRIES) {
    l1d_prev_prefetches_table[index].page_addr_pointer = L1D_PREV_PREFETCHES_TABLE_NULL_POINTER;
  }
}

uint64_t sberti::l1d_get_and_set_latency_prev_prefetches_table(uint64_t pointer, uint64_t offset, uint64_t cycle)
{
  uint64_t index = l1d_find_prev_prefetch_entry(pointer, offset);
  if (index == L1D_PREV_PREFETCHES_TABLE_ENTRIES)
    return 0;
  if (!l1d_prev_prefetches_table[index].completed) {
    l1d_prev_prefetches_table[index].time_lat = l1d_get_latency(cycle, l1d_prev_prefetches_table[index].time_lat);
    l1d_prev_prefetches_table[index].completed = true;
  }
  return l1d_prev_prefetches_table[index].time_lat;
}

uint64_t sberti::l1d_get_latency_prev_prefetches_table(uint64_t pointer, uint64_t offset)
{
  uint64_t index = l1d_find_prev_prefetch_entry(pointer, offset);
  if (index == L1D_PREV_PREFETCHES_TABLE_ENTRIES)
    return 0;
  if (!l1d_prev_prefetches_table[index].completed)
    return 0;
  return l1d_prev_prefetches_table[index].time_lat;
}

// --- RECORD PAGES TABLE ---

void sberti::l1d_init_record_pages_table()
{
  for (int i = 0; i < L1D_RECORD_PAGES_TABLE_ENTRIES; i++) {
    l1d_record_pages_table[i].page_addr = 0;
    l1d_record_pages_table[i].u_vector = 0;
    l1d_record_pages_table[i].lru = i;
  }
}

uint64_t sberti::l1d_get_lru_record_pages_entry()
{
  uint64_t lru = L1D_RECORD_PAGES_TABLE_ENTRIES;
  for (int i = 0; i < L1D_RECORD_PAGES_TABLE_ENTRIES; i++) {
    l1d_record_pages_table[i].lru++;
    if (l1d_record_pages_table[i].lru == L1D_RECORD_PAGES_TABLE_ENTRIES) {
      l1d_record_pages_table[i].lru = 0;
      lru = i;
    }
  }
  assert(lru != L1D_RECORD_PAGES_TABLE_ENTRIES);
  return lru;
}

void sberti::l1d_update_lru_record_pages_table(uint64_t index)
{
  assert(index < L1D_RECORD_PAGES_TABLE_ENTRIES);
  for (int i = 0; i < L1D_RECORD_PAGES_TABLE_ENTRIES; i++) {
    if (l1d_record_pages_table[i].lru < l1d_record_pages_table[index].lru) {
      l1d_record_pages_table[i].lru++;
    }
  }
  l1d_record_pages_table[index].lru = 0;
}

void sberti::l1d_add_record_pages_table(uint64_t index, uint64_t page_addr, uint64_t vector, uint64_t first_offset, int b)
{
  assert(index < L1D_RECORD_PAGES_TABLE_ENTRIES);
  l1d_record_pages_table[index].page_addr = page_addr & L1D_TRUNCATED_PAGE_ADDR_MASK;
  l1d_record_pages_table[index].u_vector = vector;
  l1d_record_pages_table[index].first_offset = first_offset;
  l1d_record_pages_table[index].berti = b;
  l1d_update_lru_record_pages_table(index);
}

uint64_t sberti::l1d_get_entry_record_pages_table(uint64_t page_addr, uint64_t first_offset)
{
  uint64_t trunc_page_addr = page_addr & L1D_TRUNCATED_PAGE_ADDR_MASK;
  for (int i = 0; i < L1D_RECORD_PAGES_TABLE_ENTRIES; i++) {
    if (l1d_record_pages_table[i].page_addr == trunc_page_addr && l1d_record_pages_table[i].first_offset == first_offset) {
      return i;
    }
  }
  return L1D_RECORD_PAGES_TABLE_ENTRIES;
}

uint64_t sberti::l1d_get_entry_record_pages_table(uint64_t page_addr)
{
  uint64_t trunc_page_addr = page_addr & L1D_TRUNCATED_PAGE_ADDR_MASK;
  for (int i = 0; i < L1D_RECORD_PAGES_TABLE_ENTRIES; i++) {
    if (l1d_record_pages_table[i].page_addr == trunc_page_addr) {
      return i;
    }
  }
  return L1D_RECORD_PAGES_TABLE_ENTRIES;
}

void sberti::l1d_copy_entries_record_pages_table(uint64_t index_from, uint64_t index_to)
{
  assert(index_from < L1D_RECORD_PAGES_TABLE_ENTRIES);
  assert(index_to < L1D_RECORD_PAGES_TABLE_ENTRIES);
  l1d_record_pages_table[index_to].page_addr = l1d_record_pages_table[index_from].page_addr;
  l1d_record_pages_table[index_to].u_vector = l1d_record_pages_table[index_from].u_vector;
  l1d_record_pages_table[index_to].first_offset = l1d_record_pages_table[index_from].first_offset;
  l1d_record_pages_table[index_to].berti = l1d_record_pages_table[index_from].berti;
  l1d_update_lru_record_pages_table(index_to);
}

// --- IP TABLE ---

void sberti::l1d_init_ip_table()
{
  for (int i = 0; i < L1D_IP_TABLE_ENTRIES; i++) {
    l1d_ip_table[i] = L1D_IP_TABLE_NULL_POINTER;
  }
}

void sberti::l1d_record_current_page(uint64_t index_current)
{
  if (l1d_current_pages_table[index_current].u_vector) { 
    uint64_t record_index = l1d_ip_table[l1d_current_pages_table[index_current].ip & L1D_IP_TABLE_INDEX_MASK];
    assert(record_index < L1D_RECORD_PAGES_TABLE_ENTRIES);
    uint64_t confidence;
    l1d_add_record_pages_table(record_index, l1d_current_pages_table[index_current].page_addr, l1d_current_pages_table[index_current].u_vector,
                               l1d_current_pages_table[index_current].first_offset, l1d_get_berti_current_pages_table(index_current, confidence));
  }
}

// =====================================================================
// SMART STRIDE IMPLEMENTATION
// =====================================================================

void sberti::stride_initialize() {
    for(int i=0; i<STRIDE_SETS; i++) {
        for(int j=0; j<STRIDE_WAYS; j++) {
            stride_table[i][j].valid = false;
        }
    }
}

uint64_t sberti::strideHashPc(uint64_t pc) {
    uint64_t pc_high = (pc >> 10) ^ (pc >> 20);
    return (pc_high << 6) | (pc & 0x3F); 
}

void sberti::stride_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint64_t current_cycle, uint32_t metadata_in) {
    uint64_t hash = strideHashPc(ip);
    int set_idx = hash % STRIDE_SETS;
    uint64_t tag = hash / STRIDE_SETS;

    int way_idx = -1;
    for (int i = 0; i < STRIDE_WAYS; i++) {
        if (stride_table[set_idx][i].valid && stride_table[set_idx][i].tag == tag) {
            way_idx = i;
            break;
        }
    }

    if (way_idx != -1) {
        // HIT
        SmartStrideEntry& entry = stride_table[set_idx][way_idx];
        entry.lru_cycle = current_cycle;

        int64_t new_stride = (int64_t)addr - (int64_t)entry.last_addr;
        bool stride_match = false;

        if (std::abs(new_stride) < BLOCK_SIZE && new_stride != 0) return; 

        if (new_stride == entry.stride) stride_match = true;
        else if (entry.stride > BLOCK_SIZE && std::abs(new_stride) % std::abs(entry.stride) == 0) stride_match = true;

        if (stride_match) {
            // Confidence Update: Increase confidence on stride match
            if (entry.conf < STRIDE_MAX_CONF) entry.conf++;
            
            // Adaptive Feedback: Adjust depth/confidence based on timeliness (Early/Late)
            bool is_miss = (cache_hit == 0);
            bool is_late = is_miss && hasRecentlyPrefetched(addr);
            bool is_timely = (cache_hit == 1) && useful_prefetch;

            if (is_timely) {
                if (entry.late_conf > 0) entry.late_conf--;
            } else if (is_late) {
                entry.late_conf += 3;
            }
            if (entry.late_conf > 15) entry.late_conf = 15;

            // Depth Control: Increase degree if prefetches are consistently late
            if (entry.late_conf >= 12 && entry.depth < STRIDE_MAX_DEPTH) {
                entry.depth++;
                entry.late_conf = 7;
            } else if (entry.late_conf <= 3 && entry.depth > 1) {
                entry.depth--;
                entry.late_conf = 7;
            }

            entry.last_addr = addr;
        } else {
            // Train Logic: Reset on stride mismatch
            entry.conf--;
            entry.last_addr = addr;
            if (entry.conf <= 0) {
                entry.stride = new_stride;
                entry.depth = 1;
                entry.conf = 0;
                entry.late_conf = 7;
            }
        }

        // Issue Logic: Prefetch N lines (depth) if confidence threshold met
        if (entry.conf >= 2) {
            int start_depth = (cache_hit == 0) ? std::max(1, entry.depth - 4) : 1;
            for (int d = start_depth; d <= entry.depth; d++) {
                uint64_t pf_addr = addr + (entry.stride * d);
                // Issue prefetch with global filter check
                issue_prefetch(champsim::address{pf_addr}, ip, metadata_in);
            }
        }

    } else {
        // MISS
        int victim = 0;
        uint64_t min_lru = UINT64_MAX;
        for(int i=0; i<STRIDE_WAYS; i++) {
            if(!stride_table[set_idx][i].valid) { victim = i; break; }
            if(stride_table[set_idx][i].lru_cycle < min_lru) {
                min_lru = stride_table[set_idx][i].lru_cycle;
                victim = i;
            }
        }
        SmartStrideEntry& entry = stride_table[set_idx][victim];
        entry.valid = true;
        entry.tag = tag;
        entry.last_addr = addr;
        entry.stride = 0;
        entry.conf = 0;
        entry.depth = 1;
        entry.late_conf = 7;
        entry.lru_cycle = current_cycle;
    }
}


// =====================================================================
// MAIN INTERFACE
// =====================================================================

void sberti::prefetcher_initialize()
{
  std::cout << "sBerti (Smart Stride + Berti) Prefetcher Initialized" << std::endl;
  l1d_init_current_pages_table();
  l1d_init_prev_requests_table();
  l1d_init_prev_prefetches_table();
  l1d_init_record_pages_table();
  l1d_init_ip_table();
  stride_initialize();
}

void sberti::stride_trigger_prefetch(uint64_t addr, uint64_t ip, uint32_t metadata_in) {
    uint64_t hash = strideHashPc(ip);
    int set_idx = hash % STRIDE_SETS;
    uint64_t tag = hash / STRIDE_SETS;

    int way_idx = -1;
    for (int i = 0; i < STRIDE_WAYS; i++) {
        if (stride_table[set_idx][i].valid && stride_table[set_idx][i].tag == tag) {
            way_idx = i;
            break;
        }
    }

    if (way_idx != -1) {
        SmartStrideEntry& entry = stride_table[set_idx][way_idx];

        // Hybrid Hook: Berti detects useful pattern, forces Stride to issue immediately
        if (entry.conf >= 2) {
            for (int d = 1; d <= entry.depth; d++) {
                uint64_t pf_addr = addr + (entry.stride * d);
                issue_prefetch(champsim::address{pf_addr}, ip, metadata_in);
            }
        }
    }
}

uint32_t sberti::prefetcher_cache_operate(champsim::address address, champsim::address ip_addr, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                         uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t ip = ip_addr.to<uint64_t>();
  auto current_core_cycle = (intern_->current_time.time_since_epoch() / intern_->clock_period);

  uint64_t line_addr = addr >> LOG2_BLOCK_SIZE;
  uint64_t page_addr = line_addr >> L1D_PAGE_BLOCKS_BITS;
  uint64_t offset = line_addr & L1D_PAGE_OFFSET_MASK;
  
  std::size_t pq_size = intern_->get_pq_size()[0];
  std::size_t pq_occupancy = intern_->get_pq_occupancy()[0];

  // Update current pages table
  uint64_t index = l1d_get_current_pages_entry(page_addr);

  // BERTI LOGIC
  if (index == L1D_CURRENT_PAGES_TABLE_ENTRIES || !l1d_requested_offset_current_pages_table(index, offset)) {
      if (index < L1D_CURRENT_PAGES_TABLE_ENTRIES) { 
        if (l1d_requested_offset_current_pages_table(index, offset)) goto SKIP_BERTI; 

        uint64_t first_ip = l1d_update_demand_current_pages_table(index, offset);
        
        if (cache_hit) {
            uint64_t pref_latency = l1d_get_latency_prev_prefetches_table(index, offset);
            if (pref_latency != 0) {
                int b[L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS];
                l1d_get_berti_prev_requests_table(index, offset, current_core_cycle - pref_latency, b);
                
                for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS; i++) {
                    if (b[i] == 0) break;
                    l1d_add_berti_current_pages_table(index, b[i]);
                }

                // Interaction Point: On Berti hit, trigger Smart Stride prediction
                stride_trigger_prefetch(addr, ip, metadata_in);

                l1d_reset_entry_prev_prefetches_table(index, offset);
            }
        }
        if (first_ip != ip) {
             l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = l1d_ip_table[first_ip & L1D_IP_TABLE_INDEX_MASK];
        }
      } else {
        // Find victim 
        uint64_t victim_index = l1d_get_lru_current_pages_entry(); 
        assert(victim_index < L1D_CURRENT_PAGES_TABLE_ENTRIES);
        l1d_reset_pointer_prev_requests(victim_index);   
        l1d_reset_pointer_prev_prefetches(victim_index); 

        l1d_record_current_page(victim_index);

        index = victim_index;
        l1d_add_current_pages_table(index, page_addr, ip & L1D_IP_TABLE_INDEX_MASK, offset);

        uint64_t index_record = l1d_get_entry_record_pages_table(page_addr, offset);
        if (l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] == L1D_IP_TABLE_NULL_POINTER) {
            if (index_record == L1D_RECORD_PAGES_TABLE_ENTRIES) {
                uint64_t new_pointer = l1d_get_lru_record_pages_entry();
                l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = new_pointer;
            } else {
                l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] = index_record;
            }
        } else if (l1d_ip_table[ip & L1D_IP_TABLE_INDEX_MASK] != index_record) {
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

    if (pgo_pointer != L1D_RECORD_PAGES_TABLE_ENTRIES && (l1d_record_pages_table[pgo_pointer].u_vector | l1d_current_pages_table[index].u_vector) == l1d_record_pages_table[pgo_pointer].u_vector) {
      u_vector = l1d_record_pages_table[pgo_pointer].u_vector;
      b = l1d_record_pages_table[pgo_pointer].berti;
      match_confidence = 1; 
      recorded = true;
    } else if (l1d_record_pages_table[ip_pointer].first_offset == first_offset && (l1d_record_pages_table[ip_pointer].u_vector | l1d_current_pages_table[index].u_vector) == l1d_record_pages_table[ip_pointer].u_vector) {
        u_vector = l1d_record_pages_table[ip_pointer].u_vector;
        b = l1d_record_pages_table[ip_pointer].berti;
        match_confidence = 1;
        recorded = true;
    } else if (current_berti != 0 && berti_confidence >= L1D_BERTI_CTR_MED_HIGH_CONFIDENCE) { 
          u_vector = l1d_current_pages_table[index].u_vector;
          b = current_berti;
    } else if (pg_pointer != L1D_RECORD_PAGES_TABLE_ENTRIES) { 
            u_vector = l1d_record_pages_table[pg_pointer].u_vector;
            b = l1d_record_pages_table[pg_pointer].berti;
            recorded = true;
    } else if (l1d_record_pages_table[ip_pointer].u_vector) { 
              u_vector = l1d_record_pages_table[ip_pointer].u_vector;
              b = l1d_record_pages_table[ip_pointer].berti;
              recorded = true;
    }

    // Burst Logic
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
            if ((int)i >= L1D_PAGE_BLOCKS) break; 
            uint64_t pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | i;
            uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << i) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                  // Issue using sBerti wrapper
                  bool prefetched = issue_prefetch(champsim::address{pf_addr}, ip, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
              } else { 
                l1d_current_pages_table[index].last_burst = i;
                break;
              }
            }
          }
        } else if (b < 0) {
          for (int i = (int)first_burst; i > ((int)offset) + b; i--) {
            if (i < 0) break;
            uint64_t pf_line_addr = (page_addr << L1D_PAGE_BLOCKS_BITS) | i;
            uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
            uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
            if ((((uint64_t)1 << i) & u_vector) && !l1d_requested_offset_current_pages_table(index, pf_offset)) {
              if (pq_occupancy < pq_size && bursts < L1D_MAX_NUM_BURST_PREFETCHES) {
                  bool prefetched = issue_prefetch(champsim::address{pf_addr}, ip, 0);
                  if (prefetched) {
                    l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
                    bursts++;
                  }
              } else { 
                l1d_current_pages_table[index].last_burst = i;
                break;
              }
            }
          }
        } 
      }
    }

    if (b != 0) {
      uint64_t pf_line_addr = line_addr + b;
      uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
      uint64_t pf_offset = pf_line_addr & L1D_PAGE_OFFSET_MASK;
      if (!l1d_requested_offset_current_pages_table(index, pf_offset)          
          && (!match_confidence || (((uint64_t)1 << pf_offset) & u_vector))) { 
          bool prefetched = issue_prefetch(champsim::address{pf_addr}, ip, 0);
          if (prefetched) {
            l1d_add_prev_prefetches_table(index, pf_offset, current_core_cycle);
          }
      }
    }
  }

  SKIP_BERTI:;

  // Standalone Execution: Run Smart Stride logic on every access (Hit/Miss)
  stride_operate(addr, ip, cache_hit, useful_prefetch, current_core_cycle, metadata_in);

  return metadata_in;
}

uint32_t sberti::prefetcher_cache_fill(champsim::address address, long set, long way, uint8_t prefetch, champsim::address evicted_address, uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t evicted_addr = evicted_address.to<uint64_t>();
  auto current_core_cycle = (intern_->current_time.time_since_epoch() / intern_->clock_period);

  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE);
  uint64_t page_addr = line_addr >> L1D_PAGE_BLOCKS_BITS;
  uint64_t offset = line_addr & L1D_PAGE_OFFSET_MASK;

  uint64_t pointer_prev = l1d_get_current_pages_entry(page_addr);

  if (pointer_prev < L1D_CURRENT_PAGES_TABLE_ENTRIES) { 
    uint64_t pref_latency = l1d_get_and_set_latency_prev_prefetches_table(pointer_prev, offset, current_core_cycle);
    uint64_t demand_latency = l1d_get_latency_prev_requests_table(pointer_prev, offset, current_core_cycle);

    if (pref_latency == 0) {
      pref_latency = demand_latency;
    }

    if (demand_latency != 0) { 
      int b[L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS];
      l1d_get_berti_prev_requests_table(pointer_prev, offset, current_core_cycle - (pref_latency + demand_latency), b);
      for (int i = 0; i < L1D_CURRENT_PAGES_TABLE_NUM_BERTI_PER_ACCESS; i++) {
        if (b[i] == 0) break;
        l1d_add_berti_current_pages_table(pointer_prev, b[i]);
      }
    }
  }

  uint64_t victim_index = l1d_get_current_pages_entry(evicted_addr >> LOG2_PAGE_SIZE);
  if (victim_index < L1D_CURRENT_PAGES_TABLE_ENTRIES) {
    l1d_record_current_page(victim_index);
    l1d_remove_current_table_entry(victim_index);
  }

  return metadata_in;
}

void sberti::prefetcher_cycle_operate() {}