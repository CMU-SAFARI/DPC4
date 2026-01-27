/*
 *  Author: Gilead Posluns
 *  Date: Dec 20, 2025
 *
 *  Implements Global Berti, a version of Berti that also supports spatial patterns.
 *  Base berti is implemented as described in the MICRO paper, as opposed to what comes
 *  with Champsim
 *
 *  Berti: https://doi.org/10.1109/MICRO56248.2022.00072
 * */

#ifndef __GBERTI_H__
#define __GBERTI_H__

#include "champsim.h"
#include "modules.h"
#include "cache.h"
#include <unordered_map>
#include <queue>

#define HISTORY_TABLE_SETS 64
#define HISTORY_TABLE_WAYS 16
#define HISTORY_TABLE_TAG_MASK 0xFE0
#define HISTORY_TABLE_ADDR_MASK 0x3FFFFFC0

#define DELTA_TABLE_SIZE 64
#define NUM_DELTAS 32
#define DELTA_TABLE_TAG_MASK 0xFFF
#define MAX_DELTAS_PER_SEARCH 8
#define DELTA_BITS 13

//+6 because delta tracks lines, not addrs
//-1 bc signed
#define MAX_DELTA (1 << (DELTA_BITS + 6 - 1))

#define NO_PREF 0
#define L1_PREF 1
#define L2_PREF 2
#define REPLACE 3
#define LOW_CONFIDENCE 5
#define MED_CONFIDENCE 8
#define HIGH_CONFIDENCE 11
#define EXTREME_CONFIDENCE 80
#define MAX_PREFS 12

#define MAX_LATENCY ((1 << 12) - 1)
#define LINE_ADDR(addr) (addr >> 6)

/*
 * Total storage:
 *    HIST_SETS*(HIST_WAYS*40 + log2(HIST_WAYS)) +
 *    DELTA_SIZE*(NUM_DELTAS*20 + 18) + log2(DELTA_SIZE) +
 *    N_MSHRS*16 + N_L1_BLOCKS*12 bits
 *
 *    MAX= 32*1024*8 = 262144 bits
 *    CURRENT = 64*(16*47) + 64*(32*20 + 18) + 6 + 16*16 + 64*12*12
 *            = 48128 + 39808 + 262 + 9216 = 97414 = plenty of space
 *
 * */

struct history_table_entry
{
  uint64_t ip_tag;      // 7 bits
  uint64_t addr;        // 24 bits
  uint64_t timestamp;   // 16 bits
};


struct history_table
{
  history_table_entry entries[HISTORY_TABLE_SETS][HISTORY_TABLE_WAYS];
  uint8_t fifo[HISTORY_TABLE_SETS]; // 0 bits if we use a shifter instead of a ptr
};


struct delta_table_entry
{
  uint64_t ip_tag;                // 13 bits
  uint64_t ctr;                   // 4 bits
  int32_t delta[NUM_DELTAS];     // 13 bits each
  uint64_t coverage[NUM_DELTAS];  // 4 bits each
  uint64_t status[NUM_DELTAS];    // 2 bits each
  uint8_t coverage_increment[NUM_DELTAS]; //1 bit each
  bool has_local_delta;           // 1 bit
};


struct delta_table
{
  delta_table_entry entries[DELTA_TABLE_SIZE];
  uint64_t fifo; // 6 bits
};


struct gberti : public champsim::modules::prefetcher {
  private:
    history_table histories;
    delta_table deltas;
    std::unordered_map<uint64_t, uint64_t> fetch_latencies; //12 bits each
    std::unordered_map<uint64_t, uint64_t> miss_timestamps;  //16 bits each
    std::unordered_map<uint64_t, uint64_t> miss_ips; //already in MSHR/packet

    void record_access(uint64_t addr, uint64_t ip, uint64_t cycle);
    void search_for_deltas(uint64_t addr, uint64_t, uint64_t max_timely_timestamp);
    void search_for_global_deltas(uint64_t addr, uint64_t ip);
    void send_prefetches(uint64_t addr, uint64_t ip, uint64_t cycle);

    bool valid_delta(int64_t delta);

    void init_delta_table(delta_table& table);
    void init_delta_entry(delta_table_entry& entry);
    void init_history_entry(history_table_entry& entry);
    void init_history_table(history_table& table);

  public:
    using champsim::modules::prefetcher::prefetcher;

    // champsim interface prototypes
    void prefetcher_initialize();
    uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
    uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
    void prefetcher_cycle_operate() {}
};

#endif /* __GBERTI_H__ */
