#ifndef __BERTIGO_H_
#define __BERTIGO_H_

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <queue>
#include <stdlib.h>
#include <vector>

#include "BertiGo_parameters.h"
#include "champsim.h"
#include "modules.h"
#include "Filter.h"

class BertiGo : public champsim::modules::prefetcher
{
private:
  /*****************************************************************************
   *                              Stats                                        *
   *****************************************************************************/
  // Get average latency: Welford's method
  typedef struct welford {
    uint64_t num = 0;
    float average = 0.0;
  } welford_t;

  welford_t average_latency;

  // Get more info
  uint64_t pf_to_l1;
  uint64_t pf_to_l2;
  uint64_t pf_to_l2_bc_mshr;
  uint64_t cant_track_latency;
  uint64_t cross_page;
  uint64_t no_cross_page;
  uint64_t no_found_berti;
  uint64_t found_berti;
  uint64_t average_issued;
  uint64_t average_num;
  // Bloom filter metrics (optional)
  uint64_t filtered = 0;
  uint64_t bloom_issued = 0;
  
  // PC_Path signature tracking
  std::array<uint64_t, PC_PATH_HISTORY_SIZE> last_pcs = {};

  /*****************************************************************************
   *                      General Structs                                      *
   *****************************************************************************/

  typedef struct Delta {
    uint64_t conf;
    int64_t delta;
    uint8_t rpl;
    Delta() : conf(0), delta(0), rpl(BERTI_R){};
  } delta_t;

  /*****************************************************************************
   *                      Berti structures                                     *
   *****************************************************************************/
  class LatencyTable
  {
    /* Latency table simulate the modified PQ and MSHR */
  private:
    struct latency_table {
      uint64_t addr = 0; // Addr
      uint64_t tag = 0;  // IP-Tag
      uint64_t time = 0; // Event cycle
      bool pf = false;   // Is the entry accessed by a demand miss
    };
    int size;

    latency_table* latencyt;

  public:
    LatencyTable(const int size) : size(size) { latencyt = new latency_table[size]; }
    ~LatencyTable() { delete latencyt; }

    uint8_t add(uint64_t addr, uint64_t tag, bool pf, uint64_t cycle);
    uint64_t get(uint64_t addr);
    uint64_t del(uint64_t addr);
    uint64_t get_tag(uint64_t addr);
  };

  class ShadowCache
  {
    /* Shadow cache simulate the modified L1D Cache */
  private:
    struct shadow_cache {
      uint64_t addr = 0; // Addr
      uint64_t lat = 0;  // Latency
      bool pf = false;   // Is a prefetch
    };                   // This struct is the vberti table

    int sets;
    int ways;
    shadow_cache** scache;

  public:
    ShadowCache(const int sets, const int ways)
    {
      scache = new shadow_cache*[sets];
      for (int i = 0; i < sets; i++)
        scache[i] = new shadow_cache[ways];

      this->sets = sets;
      this->ways = ways;
    }

    ~ShadowCache()
    {
      for (int i = 0; i < sets; i++)
        delete scache[i];
      delete scache;
    }

    bool add(uint32_t set, uint32_t way, uint64_t addr, bool pf, uint64_t lat);
    bool get(uint64_t addr);
    void set_pf(uint64_t addr, bool pf);
    bool is_pf(uint64_t addr);
    uint64_t get_latency(uint64_t addr);
  };

  class HistoryTable
  {
    /* History Table */
  private:
    struct history_table {
      uint64_t tag = 0;  // IP Tag
      uint64_t addr = 0; // IP @ accessed
      uint64_t time = 0; // Time where the line is accessed
    };                   // This struct is the history table

    const int sets = HISTORY_TABLE_SETS;
    const int ways = HISTORY_TABLE_WAYS;

    history_table** historyt;
    history_table** history_pointers;

    uint16_t get_aux(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle);

  public:
    HistoryTable()
    {
      history_pointers = new history_table*[sets];
      historyt = new history_table*[sets];

      for (int i = 0; i < sets; i++)
        historyt[i] = new history_table[ways];
      for (int i = 0; i < sets; i++)
        history_pointers[i] = historyt[i];
    }

    ~HistoryTable()
    {
      for (int i = 0; i < sets; i++)
        delete historyt[i];
      delete historyt;

      delete history_pointers;
    }

    int get_ways();
    void add(uint64_t tag, uint64_t addr, uint64_t cycle);
    uint16_t get(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle);
  };

  class InnerBerti
  {
    /* Berti Table */
  private:
    struct berti {
      std::array<delta_t, BERTI_TABLE_DELTA_SIZE> deltas;
      uint64_t conf = 0;
      uint64_t total_used = 0;
    };

    std::map<uint64_t, berti*> bertit;
    std::queue<uint64_t> bertit_queue;

    uint64_t size = 0;


    void increase_conf_tag(uint64_t tag);
    void conf_tag(uint64_t tag);
    void add(uint64_t tag, int64_t delta);

  public:
    LatencyTable* latencyt;
    ShadowCache* scache;
    HistoryTable* historyt;

    InnerBerti(uint64_t p_size, int latency_table_size, int sets, int ways) : size(p_size)
    {
      latencyt = new LatencyTable(latency_table_size);
      scache = new ShadowCache(sets, ways);
      historyt = new HistoryTable();
    };
    void find_and_update(uint64_t latency, uint64_t tag, uint64_t cycle, uint64_t line_addr);
    uint8_t get(uint64_t tag, std::vector<delta_t>& res);
    uint64_t ip_hash(uint64_t ip);
    bool static compare_rpl(delta_t a, delta_t b);
    bool static compare_greater_delta(delta_t a, delta_t b);
  };

  InnerBerti* berti;

  friend class LatencyTable;
  friend class ShadowCache;
  friend class HistoryTable;
  friend class InnerBerti;

public:
  using prefetcher::prefetcher;

  // champsim interface prototypes
  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();

private:
  // Per-prefetcher Bloom filter instance
  bertigo::Filter filter;
};
#endif
