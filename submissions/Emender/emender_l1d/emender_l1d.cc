// Based on VBerti impl from https://github.com/ChampSim/ChampSim/pull/486

#include "emender_l1d.h"

#include <algorithm>
#include <cassert>

#include "dpc_api.h"

/*****************************************************************************
 *                              SIZES                                        *
 *****************************************************************************/
// BERTI
#define BERTI_TABLE_SIZE_LOG2 (6)
#define BERTI_TABLE_SIZE (1 << BERTI_TABLE_SIZE_LOG2)
#define BERTI_TABLE_DELTA_SIZE (16)
#define BERTI_MISS_CONF_WIDTH (10)
#define BERTI_MISS_CONF_MAX (1 << (BERTI_MISS_CONF_WIDTH - 1) - 1)
#define BERTI_MISS_CONF_MIN -(1 << (BERTI_MISS_CONF_WIDTH - 1))
#define DELTA_WIDTH (13)
#define IP_TAG_WIDTH (16)
#define TIME_WIDTH (16)
#define VA_WIDTH (64)
#define LINE_ADDR_WIDTH (VA_WIDTH - LOG2_BLOCK_SIZE)
#define HIST_ADDR_WIDTH (24)
#define LAT_WIDTH (12)
#define CUCKOO_FILTER_SETS_LOG2 (10)
#define CUCKOO_FILTER_SETS (1 << CUCKOO_FILTER_SETS_LOG2)
#define CUCKOO_FILTER_WAYS (4)
#define CUCKOO_FILTER_FINGERPRINT_WIDTH (12)
#define CUCKOO_FILTER_FINGERPRINT_MASK ((1 << CUCKOO_FILTER_FINGERPRINT_WIDTH) - 1)
#define CUCKOO_FILTER_MAX_NUM_KICKS (2)

// HISTORY
#define HISTORY_TABLE_SETS (16)
#define HISTORY_TABLE_WAYS_LOG2 (5)
#define HISTORY_TABLE_WAYS (1 << HISTORY_TABLE_WAYS_LOG2)

// Hash Function
// #define HASH_FN
// # define HASH_ORIGINAL
// # define THOMAS_WANG_HASH_1
// # define THOMAS_WANG_HASH_2
// # define THOMAS_WANG_HASH_3
// # define THOMAS_WANG_HASH_4
// # define THOMAS_WANG_HASH_5
// # define THOMAS_WANG_HASH_6
// # define THOMAS_WANG_HASH_7
// # define THOMAS_WANG_NEW_HASH
// # define THOMAS_WANG_HASH_HALF_AVALANCHE
// # define THOMAS_WANG_HASH_FULL_AVALANCHE
// # define THOMAS_WANG_HASH_INT_1
// # define THOMAS_WANG_HASH_INT_2
#define ENTANGLING_HASH

/*****************************************************************************
 *                              MASKS                                        *
 *****************************************************************************/
#define IP_MASK ((1 << IP_TAG_WIDTH) - 1)
#define TIME_MASK ((1 << TIME_WIDTH) - 1)
#define LAT_MASK ((1 << LAT_WIDTH) - 1)
#define ADDR_MASK ((1 << HIST_ADDR_WIDTH) - 1)
#define TABLE_SET_MASK (HISTORY_TABLE_SETS - 1)

/*****************************************************************************
 *                      CONFIDENCE VALUES                                    *
 *****************************************************************************/
// confidence values
#define CONFIDENCE_WIDTH (5)
#define CONFIDENCE_MAX (16)
#define CONFIDENCE_INC (1)
#define CONFIDENCE_INIT (1)

#define CONFIDENCE_L1 (10)
#define CONFIDENCE_L2 (8)
#define CONFIDENCE_L2R (6)

#define CONFIDENCE_MIDDLE_L1 (14)
#define CONFIDENCE_MIDDLE_L2 (12)
#define LAUNCH_MIDDLE_CONF (8)

/*****************************************************************************
 *                              LIMITS                                       *
 *****************************************************************************/
#define MSHR_LIMIT (70)

#define GENERATION_OP_WIDTH (4)
#define GENERATION_OP_MASK ((1 << GENERATION_OP_WIDTH) - 1)
#define PENDING_WINDOW_OPS (5) // recent N operates
#define PENDING_MAX_SIZE (PENDING_WINDOW_OPS * BERTI_TABLE_DELTA_SIZE)
#define PENDING_TRIES_WIDTH (1)
#define PENDING_MAX_TRIES ((1 << PENDING_TRIES_WIDTH) - 1)
#define ISSUE_BUDGET_PER_CYCLE (1)

/*****************************************************************************
 *                              CONSTANT PARAMETERS                          *
 *****************************************************************************/
#define BERTI_R (0x0)
#define BERTI_L1 (0x1)
#define BERTI_L2 (0x2)
#define BERTI_L2R (0x3)

namespace emender_l1d_ns
{

/*****************************************************************************
 *                              Stats                                        *
 *****************************************************************************/
// Get average latency: Welford's method
typedef struct welford {
  uint64_t num = 0;
  float average = 0.0;
} welford_t;

/*****************************************************************************
 *                      General Structs                                      *
 *****************************************************************************/

// total width: CONFIDENCE_WIDTH + DELTA_WIDTH + 2
typedef struct Delta {
  // width: CONFIDENCE_WIDTH
  // delta confidence, from 0 to CONFIDENCE_MAX
  uint64_t conf;
  // width: DELTA_WIDTH
  // signed delta, from -(1 << DELTA_WIDTH) to (1 << DELTA_WIDTH) - 1
  int64_t delta;
  // width: 2
  // BERTI_R/BERTI_L1/BERTI_L2/BERTI_L2R
  uint8_t rpl;
  Delta() : conf(0), delta(0), rpl(BERTI_R) {};
} delta_t;

/*****************************************************************************
 *                      Berti structures                                     *
 *****************************************************************************/
class LatencyTable
{
  /* Latency table simulate the modified PQ and MSHR */
private:
  struct latency_table {
    // width: LINE_ADDR_WIDTH
    // Addr: the cache line address (without offset) of the inflight request
    uint64_t addr = 0;
    // width: IP_TAG_WIDTH
    // IP-Tag: the hashed ip of the triggering load
    uint64_t tag = 0;
    // width: TIME_WIDTH
    // Event cycle: the time when entering the table
    uint64_t time = 0;
    // width: 1
    // Is the entry accessed by a demand miss?
    // false: from miss (mshr)
    // true: from prefetch (pq)?
    bool pf = false;
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
    // width: LINE_ADDR_WIDTH
    // Addr: full tag of the cache line address
    uint64_t addr = 0;
    // width: LAT_WIDTH
    // Latency: the latency refilling the cacheline
    uint64_t lat = 0;
    // width: 1
    // Is a prefetch: the cacheline was refilled due to prefetch?
    bool pf = false; // Is a prefetch
  };
  // cuckoo filter
  // width: CUCKOO_FILTER_SETS * CUCKOO_FILTER_WAYS * (CUCKOO_FILTER_FINGERPRINT_WIDTH + 1)
  struct cuckoo_filter_element {
    bool valid = false;
    uint16_t fingerprint;
  };
  std::vector<std::vector<cuckoo_filter_element>> cuckoo_filter;

  int sets;
  int ways;
  shadow_cache** scache;

  // cuckoo filter functions
  uint64_t fingerprint_hash(uint64_t addr)
  {
    // hash from here: http://burtleburtle.net/bob/hash/integer.html
    addr = (addr ^ 61) ^ (addr >> 16);
    addr = addr + (addr << 3);
    addr = addr ^ (addr >> 4);
    addr = addr * 0x27d4eb2d;
    addr = addr ^ (addr >> 15);
    return addr;
  }

  uint64_t index_hash(uint64_t addr)
  {
    // hash from here: http://burtleburtle.net/bob/hash/integer.html
    addr = (addr + 0x7ed55d16) + (addr << 12);
    addr = (addr ^ 0xc761c23c) ^ (addr >> 19);
    addr = (addr + 0x165667b1) + (addr << 5);
    addr = (addr + 0xd3a2646c) ^ (addr << 9);
    addr = (addr + 0xfd7046c5) + (addr << 3);
    addr = (addr ^ 0xb55a4f09) ^ (addr >> 16);
    return addr;
  }

  void add(uint64_t addr)
  {
    uint64_t fingerprint = fingerprint_hash(addr) & CUCKOO_FILTER_FINGERPRINT_MASK;
    uint64_t i1 = index_hash(addr) % CUCKOO_FILTER_SETS;
    uint64_t i2 = i1 ^ (index_hash(fingerprint) % CUCKOO_FILTER_SETS);

    // find empty slot in i1
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (!cuckoo_filter[i1][j].valid) {
        cuckoo_filter[i1][j].valid = true;
        cuckoo_filter[i1][j].fingerprint = fingerprint;
        return;
      }
    }

    // find empty slot in i2
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (!cuckoo_filter[i2][j].valid) {
        cuckoo_filter[i2][j].valid = true;
        cuckoo_filter[i2][j].fingerprint = fingerprint;
        return;
      }
    }

    // relocate
    int i = (rand() % 2 == 0) ? i1 : i2;
    for (int n = 0; n < CUCKOO_FILTER_MAX_NUM_KICKS; n++) {
      // randomly seelct a victim
      int j = rand() % CUCKOO_FILTER_WAYS;
      assert(cuckoo_filter[i][j].valid);

      // swap
      uint16_t temp = cuckoo_filter[i][j].fingerprint;
      cuckoo_filter[i][j].fingerprint = fingerprint;
      fingerprint = temp;

      // evict to another slot
      i = i ^ (index_hash(fingerprint) % CUCKOO_FILTER_SETS);

      // find empty entry
      for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
        if (!cuckoo_filter[i][j].valid) {
          cuckoo_filter[i][j].valid = true;
          cuckoo_filter[i][j].fingerprint = fingerprint;
          return;
        }
      }
    }
  }

  void remove(uint64_t addr)
  {
    uint64_t fingerprint = fingerprint_hash(addr) & CUCKOO_FILTER_FINGERPRINT_MASK;
    uint64_t i1 = index_hash(addr) % CUCKOO_FILTER_SETS;
    uint64_t i2 = i1 ^ (index_hash(fingerprint) % CUCKOO_FILTER_SETS);

    // remove in i1
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (cuckoo_filter[i1][j].valid && cuckoo_filter[i1][j].fingerprint == fingerprint) {
        cuckoo_filter[i1][j].valid = false;
        return;
      }
    }

    // find empty slot in i2
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (cuckoo_filter[i2][j].valid && cuckoo_filter[i2][j].fingerprint == fingerprint) {
        cuckoo_filter[i2][j].valid = false;
        return;
      }
    }
  }

public:
  ShadowCache(const int sets, const int ways)
  {
    scache = new shadow_cache*[sets];
    for (int i = 0; i < sets; i++)
      scache[i] = new shadow_cache[ways];

    this->sets = sets;
    this->ways = ways;

    // initialize cuckoo filter
    cuckoo_filter.resize(CUCKOO_FILTER_SETS);
    for (int i = 0; i < CUCKOO_FILTER_SETS; i++) {
      cuckoo_filter[i].resize(CUCKOO_FILTER_WAYS);
    }
  }

  ~ShadowCache()
  {
    for (int i = 0; i < sets; i++)
      delete scache[i];
    delete scache;
  }

  bool add(uint32_t set, uint32_t way, uint64_t addr, bool pf, uint64_t lat, uint64_t evicted_addr);
  bool get(uint64_t addr);
  bool get_cuckoo_filter(uint64_t addr)
  {
    uint64_t fingerprint = fingerprint_hash(addr) & CUCKOO_FILTER_FINGERPRINT_MASK;
    uint64_t i1 = index_hash(addr) % CUCKOO_FILTER_SETS;
    uint64_t i2 = i1 ^ (index_hash(fingerprint) % CUCKOO_FILTER_SETS);

    // find in i1
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (cuckoo_filter[i1][j].valid && cuckoo_filter[i1][j].fingerprint == fingerprint) {
        return true;
      }
    }

    // find in i2
    for (int j = 0; j < CUCKOO_FILTER_WAYS; j++) {
      if (cuckoo_filter[i2][j].valid && cuckoo_filter[i2][j].fingerprint == fingerprint) {
        return true;
      }
    }
    return false;
  }
  void set_pf(uint64_t addr, bool pf);
  bool is_pf(uint64_t addr);
  uint64_t get_latency(uint64_t addr);
};

class HistoryTable
{
  /* History Table */
private:
  struct history_table {
    // width: IP_TAG_WIDTH
    uint64_t tag = 0; // IP Tag
    // width: HIST_ADDR_WIDTH
    uint64_t addr = 0; // IP @ accessed
    // width: TIME_WIDTH
    uint64_t time = 0; // Time where the line is accessed
    // hidden replacement policy: log2(HISTORY_TABLE_WAYS) per set
  }; // This struct is the history table

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

  void add(uint64_t tag, uint64_t addr, uint64_t cycle);
  uint16_t get(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle);
};

class Berti
{
  /* Berti Table */
private:
  /* fully associated berti table */
  // FIFO replacement policy: log2(BERTI_TABLE_SIZE) bits to record the victim index
  // tag: IP_TAG_WIDTH bit hashed ip
  struct berti {
    // delta table
    std::array<delta_t, BERTI_TABLE_DELTA_SIZE> deltas;
    // width: CONFIDENCE_WIDTH
    uint64_t conf = 0;
    // width: BERTI_MISS_CONF_WIDTH
    int64_t miss_conf = BERTI_MISS_CONF_MIN;
  };
  std::map<uint64_t, berti*> bertit;
  std::queue<uint64_t> bertit_queue;

  uint64_t size = 0;
  emender_l1d_ns::vberti* parent;
  emender_l1d_ns::HistoryTable* historyt;

  bool static compare_greater_delta(delta_t a, delta_t b);
  bool static compare_rpl(delta_t a, delta_t b);

  void increase_conf_tag(uint64_t tag);
  void conf_tag(uint64_t tag);
  void add(uint64_t tag, int64_t delta);

public:
  Berti(uint64_t p_size, emender_l1d_ns::vberti* p_parent, HistoryTable* p_historyt) : size(p_size), parent(p_parent), historyt(p_historyt) {};
  void find_and_update(uint64_t latency, uint64_t tag, uint64_t cycle, uint64_t line_addr);
  uint8_t get(uint64_t tag, std::vector<delta_t>& res, bool hit);
  uint64_t ip_hash(uint64_t ip);
};

class vberti : public champsim::modules::prefetcher
{
private:
  uint64_t pf_to_l1 = 0;
  uint64_t pf_to_l2 = 0;
  uint64_t pf_to_l2_bc_mshr = 0;
  uint64_t cant_track_latency = 0;
  uint64_t cross_page = 0;
  uint64_t no_cross_page = 0;
  uint64_t no_found_berti = 0;
  uint64_t found_berti = 0;
  uint64_t average_issued = 0;
  uint64_t average_num = 0;
  uint64_t delta_too_large = 0;
  emender_l1d_ns::welford_t average_latency;
  emender_l1d_ns::welford_t average_mbw;
  friend emender_l1d_ns::Berti;
  emender_l1d_ns::LatencyTable* latencyt;
  emender_l1d_ns::ShadowCache* scache;
  emender_l1d_ns::HistoryTable* historyt;
  emender_l1d_ns::Berti* berti;
  bool throttle = false;
  uint64_t filter_correct = 0;
  uint64_t filter_wrong = 0;

  struct inst_stat {
    uint64_t execution_count = 0;
    uint64_t miss_count = 0;
    uint64_t prefetch_l1_count = 0;
    uint64_t prefetch_l2_count = 0;
    uint64_t prefetch_l2r_count = 0;
  };
  std::map<uint64_t, inst_stat> inst_stats;
  // Pending Target Buffer
  struct pending_target_t {
    // width: LINE_ADDR_WIDTH
    // cache line addr
    uint64_t p_b_addr;
    // width: CONFIDENCE_WIDTH
    // ranking by confidence
    uint64_t rank_conf;
    // width: 2
    // BERTI_L1/BERTI_L2/BERTI_L2R
    uint8_t rpl;
    // width: IP_TAG_WIDTH
    // hash of load ip
    uint64_t ip_hash;
    // width: GENERATION_OP_WIDTH
    // op generation id
    uint64_t gen_op;
    // width: PENDING_TRIES_WIDTH
    // the number of issue fails
    uint8_t tries;
  };
  // width: GENERATION_OP_WIDTH
  // op generation
  uint64_t operate_seq = 0;
  std::vector<pending_target_t> pending_targets;

  uint64_t pending_drop_age = 0;
  uint64_t pending_drop_full = 0;
  uint64_t pending_drop_dup = 0;

  void prune_pending();
  void enqueue_pending(const pending_target_t& t);
  int issue_pending(int budget, bool from_cycle);

public:
  using prefetcher::prefetcher;

  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
};

/******************************************************************************/
/*                      Latency table functions                               */
/******************************************************************************/
uint8_t LatencyTable::add(uint64_t addr, uint64_t tag, bool pf, uint64_t cycle)
{
  /*
   * Save if possible the new miss into the pqmshr (latency) table
   *
   * Parameters:
   *  - addr: address without cache offset
   *  - access: is theh entry accessed by a demand request
   *  - cycle: time to use in the latency table
   *
   * Return: pf
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_LATENCY_TABLE] " << __func__;
    std::cout << " addr: " << std::hex << addr << " tag: " << tag;
    std::cout << " prefetch: " << std::dec << +pf << " cycle: " << cycle;
  }

  latency_table* free;
  free = nullptr;

  for (int i = 0; i < size; i++) {
    // Search if the addr already exists. If it exist we does not have
    // to do nothing more
    if (latencyt[i].addr == addr) {
      if constexpr (champsim::debug_print) {
        std::cout << " line already found; find_tag: " << latencyt[i].tag;
        std::cout << " find_pf: " << +latencyt[i].pf << std::endl;
      }
      // latencyt[i].time = cycle;
      latencyt[i].pf = pf;
      latencyt[i].tag = tag;
      return latencyt[i].pf;
    }

    // We discover a free space into the latency table, save it for later
    if (latencyt[i].tag == 0)
      free = &latencyt[i];
  }

  // we allocated a large enough latency table to cover all inflight requests
  if (free == nullptr)
    assert(0 && "No free space latency table");

  // We save the new entry into the latency table
  free->addr = addr;
  free->time = cycle;
  free->tag = tag;
  free->pf = pf;

  if constexpr (champsim::debug_print)
    std::cout << " new entry" << std::endl;
  return free->pf;
}

uint64_t LatencyTable::del(uint64_t addr)
{
  /*
   * Remove the address from the latency table
   *
   * Parameters:
   *  - addr: address without cache offset
   *
   *  Return: the latency of the address
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_LATENCY_TABLE] " << __func__;
    std::cout << " addr: " << std::hex << addr;
  }

  for (int i = 0; i < size; i++) {
    // Line already in the table
    if (latencyt[i].addr == addr) {
      // Calculate latency
      uint64_t time = latencyt[i].time;

      if constexpr (champsim::debug_print) {
        std::cout << " tag: " << latencyt[i].tag;
        std::cout << " prefetch: " << std::dec << +latencyt[i].pf;
        std::cout << " cycle: " << latencyt[i].time << std::endl;
      }

      latencyt[i].addr = 0; // Free the entry
      latencyt[i].tag = 0;  // Free the entry
      latencyt[i].time = 0; // Free the entry
      latencyt[i].pf = 0;   // Free the entry

      // Return the latency
      return time;
    }
  }

  // We should always track the misses
  if constexpr (champsim::debug_print)
    std::cout << " TRANSLATION" << std::endl;
  return 0;
}

uint64_t LatencyTable::get(uint64_t addr)
{
  /*
   * Return time or 0 if the addr is or is not in the pqmshr (latency) table
   *
   * Parameters:
   *  - addr: address without cache offset
   *
   * Return: time if the line is in the latency table, otherwise 0
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_LATENCY_TABLE] " << __func__;
    std::cout << " addr: " << std::hex << addr << std::dec;
  }

  for (int i = 0; i < size; i++) {
    // Search if the addr already exists
    if (latencyt[i].addr == addr) {
      if constexpr (champsim::debug_print) {
        std::cout << " time: " << latencyt[i].time << std::endl;
      }
      return latencyt[i].time;
    }
  }

  if constexpr (champsim::debug_print)
    std::cout << " NOT FOUND" << std::endl;
  return 0;
}

uint64_t LatencyTable::get_tag(uint64_t addr)
{
  /*
   * Return IP-Tag or 0 if the addr is or is not in the pqmshr (latency) table
   *
   * Parameters:
   *  - addr: address without cache offset
   *
   * Return: ip-tag if the line is in the latency table, otherwise 0
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_LATENCY_TABLE] " << __func__;
    std::cout << " addr: " << std::hex << addr;
  }

  for (int i = 0; i < size; i++) {
    if (latencyt[i].addr == addr && latencyt[i].tag) // This is the address
    {
      if constexpr (champsim::debug_print) {
        std::cout << " tag: " << latencyt[i].tag << std::endl;
      }
      return latencyt[i].tag;
    }
  }

  if constexpr (champsim::debug_print)
    std::cout << " NOT_FOUND" << std::endl;
  return 0;
}

/******************************************************************************/
/*                       Shadow Cache functions                               */
/******************************************************************************/
bool ShadowCache::add(uint32_t set, uint32_t way, uint64_t addr, bool pf, uint64_t lat, uint64_t evicted_addr)
{
  /*
   * Add block to shadow cache
   *
   * Parameters:
   *      - cpu: cpu
   *      - set: cache set
   *      - way: cache way
   *      - addr: cache block v_addr
   *      - access: the cache is access by a demand
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_SHADOW_CACHE] " << __func__;
    std::cout << " set: " << set << " way: " << way;
    std::cout << " addr: " << std::hex << addr << std::dec;
    std::cout << " pf: " << +pf;
    std::cout << " latency: " << lat << std::endl;
  }
  assert(lat <= LAT_MASK);

  // update cuckoo filter
  assert(scache[set][way].addr == evicted_addr);
  remove(evicted_addr);
  add(addr);

  scache[set][way].addr = addr;
  scache[set][way].pf = pf;
  scache[set][way].lat = lat;
  return scache[set][way].pf;
}

bool ShadowCache::get(uint64_t addr)
{
  /*
   * Parameters:
   *      - addr: cache block v_addr
   *
   * Return: true if the addr is in the l1d cache, false otherwise
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_SHADOW_CACHE] " << __func__;
    std::cout << " addr: " << std::hex << addr << std::endl;
  }

  for (int i = 0; i < sets; i++) {
    for (int ii = 0; ii < ways; ii++) {
      if (scache[i][ii].addr == addr) {
        if constexpr (champsim::debug_print) {
          std::cout << " set: " << i << " way: " << i << std::endl;
        }
        return true;
      }
    }
  }

  return false;
}

void ShadowCache::set_pf(uint64_t addr, bool pf)
{
  /*
   * Parameters:
   *      - addr: cache block v_addr
   *
   * Return: change value of pf field
   */
  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_SHADOW_CACHE] " << __func__;
    std::cout << " addr: " << std::hex << addr << std::dec;
  }

  for (int i = 0; i < sets; i++) {
    for (int ii = 0; ii < ways; ii++) {
      if (scache[i][ii].addr == addr) {
        if constexpr (champsim::debug_print) {
          std::cout << " set: " << i << " way: " << ii;
          std::cout << " old_pf_value: " << +scache[i][ii].pf;
          std::cout << " new_pf_value: " << +pf << std::endl;
        }
        scache[i][ii].pf = pf;
        return;
      }
    }
  }

  // The address should always be in the cache
  assert((0) && "Address is must be in shadow cache");
}

bool ShadowCache::is_pf(uint64_t addr)
{
  /*
   * Parameters:
   *      - addr: cache block v_addr
   *
   * Return: True if the saved one is a prefetch
   */

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_SHADOW_CACHE] " << __func__;
    std::cout << " addr: " << std::hex << addr << std::dec;
  }

  for (int i = 0; i < sets; i++) {
    for (int ii = 0; ii < ways; ii++) {
      if (scache[i][ii].addr == addr) {
        if constexpr (champsim::debug_print) {
          std::cout << " set: " << i << " way: " << ii;
          std::cout << " pf: " << +scache[i][ii].pf << std::endl;
        }

        return scache[i][ii].pf;
      }
    }
  }

  assert((0) && "Address is must be in shadow cache");
  return 0;
}

uint64_t ShadowCache::get_latency(uint64_t addr)
{
  /*
   * Init shadow cache
   *
   * Parameters:
   *      - addr: cache block v_addr
   *
   * Return: the saved latency
   */
  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_SHADOW_CACHE] " << __func__;
    std::cout << " addr: " << std::hex << addr << std::dec;
  }

  for (int i = 0; i < sets; i++) {
    for (int ii = 0; ii < ways; ii++) {
      if (scache[i][ii].addr == addr) {
        if constexpr (champsim::debug_print) {
          std::cout << " set: " << i << " way: " << ii;
          std::cout << " latency: " << scache[i][ii].lat << std::endl;
        }

        return scache[i][ii].lat;
      }
    }
  }

  assert((0) && "Address is must be in shadow cache");
  return 0;
}

/******************************************************************************/
/*                       History Table functions                               */
/******************************************************************************/
void HistoryTable::add(uint64_t tag, uint64_t addr, uint64_t cycle)
{
  /*
   * Save the new information into the history table
   *
   * Parameters:
   *  - tag: PC tag
   *  - addr: addr access
   */
  uint16_t set = tag & TABLE_SET_MASK;
  // If the latest entry is the same, we do not add it
  if (history_pointers[set] == &historyt[set][ways - 1]) {
    if (historyt[set][0].addr == (addr & ADDR_MASK))
      return;
  } else if (history_pointers[set] != &historyt[set][0] && (history_pointers[set] - 1)->addr == (addr & ADDR_MASK))
    return;

  // Save new element into the history table
  history_pointers[set]->tag = tag;
  history_pointers[set]->time = cycle & TIME_MASK;
  history_pointers[set]->addr = addr & ADDR_MASK;

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_HISTORY_TABLE] " << __func__;
    std::cout << " tag: " << std::hex << tag << " line_addr: " << addr << std::dec;
    std::cout << " cycle: " << cycle << " set: " << set << std::endl;
  }

  if (history_pointers[set] == &historyt[set][ways - 1]) {
    history_pointers[set] = &historyt[set][0]; // End the cycle
  } else
    history_pointers[set]++; // Pointer to the next (oldest) entry
}

uint16_t HistoryTable::get_aux(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle)
{
  /*
   * Return an array (by parameter) with all the possible PC that can launch
   * an on-time and late prefetch
   *
   * Parameters:
   *  - tag: PC tag
   *  - latency: latency of the processor
   */

  uint16_t num_on_time = 0;
  uint16_t set = tag & TABLE_SET_MASK;

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_HISTORY_TABLE] " << __func__;
    std::cout << " tag: " << std::hex << tag << " line_addr: " << addr << std::dec;
    std::cout << " cycle: " << cycle << " set: " << set << std::endl;
  }

  // This is the begin of the simulation
  if (cycle < latency)
    return num_on_time;

  // The IPs that is launch in this cycle will be able to launch this prefetch
  cycle -= latency;

  // Pointer to guide
  history_table* pointer = history_pointers[set];

  do {
    // Look for the IPs that can launch this prefetch
    if (pointer->tag == tag && pointer->time <= cycle) {
      // Test that addr is not duplicated
      if (pointer->addr == act_addr)
        return num_on_time;

      // This IP can launch the prefetch
      tags[num_on_time] = pointer->tag;
      addr[num_on_time] = pointer->addr;
      num_on_time++;
    }

    if (pointer == historyt[set]) {
      // We get at the end of the history, we start again
      pointer = &historyt[set][ways - 1];
    } else
      pointer--;
  } while (pointer != history_pointers[set]);

  return num_on_time;
}

uint16_t HistoryTable::get(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle)
{
  /*
   * Return an array (by parameter) with all the possible PC that can launch
   * an on-time and late prefetch
   *
   * Parameters:
   *  - tag: PC tag
   *  - latency: latency of the processor
   *  - on_time_ip (out): ips that can launch an on-time prefetch
   *  - on_time_addr (out): addr that can launch an on-time prefetch
   *  - num_on_time (out): number of ips that can launch an on-time prefetch
   */

  act_addr &= ADDR_MASK;

  uint16_t num_on_time = get_aux(latency, tag, act_addr, tags, addr, cycle & TIME_MASK);

  // We found on-time prefetchs
  return num_on_time;
}

/******************************************************************************/
/*                        Berti table functions                               */
/******************************************************************************/
void Berti::increase_conf_tag(uint64_t tag)
{
  /*
   * Increase the global confidence of the deltas associated to the tag
   *
   * Parameters:
   *  tag : tag to find
   */
  if constexpr (champsim::debug_print)
    std::cout << "[BERTI_BERTI] " << __func__ << " tag: " << std::hex << tag << std::dec;

  if (bertit.find(tag) == bertit.end()) {
    // Tag not found
    if constexpr (champsim::debug_print)
      std::cout << " TAG NOT FOUND" << std::endl;

    return;
  }

  // Get the entries and the deltas

  bertit[tag]->conf += CONFIDENCE_INC;

  if constexpr (champsim::debug_print)
    std::cout << " global_conf: " << bertit[tag]->conf;

  if (bertit[tag]->conf == CONFIDENCE_MAX) {
    int conf_l1 = CONFIDENCE_L1;
    int conf_l2 = CONFIDENCE_L2;
    int conf_l2r = CONFIDENCE_L2R;
    // dynamic confidence
    if (bertit[tag]->miss_conf >= 0 && ENABLE_DYNAMIC_CONF) {
      conf_l1 += 4 * CONFIDENCE_INC;
      conf_l2 = conf_l1;
      conf_l2r = conf_l1;
    }

    // Max confidence achieve
    for (auto& i : bertit[tag]->deltas) {
      // Set bits to prefetch level
      if (i.conf > conf_l1)
        i.rpl = BERTI_L1;
      else if (i.conf > conf_l2)
        i.rpl = BERTI_L2;
      else if (i.conf > conf_l2r)
        i.rpl = BERTI_L2R;
      else
        i.rpl = BERTI_R;

      if constexpr (champsim::debug_print) {
        std::cout << "Delta: " << i.delta;
        std::cout << " Conf: " << i.conf << " Level: " << +i.rpl;
        std::cout << "|";
      }

      i.conf = 0; // Reset confidence
    }

    bertit[tag]->conf = 0; // Reset global confidence
  }

  if constexpr (champsim::debug_print)
    std::cout << std::endl;
}

void Berti::add(uint64_t tag, int64_t delta)
{
  /*
   * Save the new information into the history table
   *
   * Parameters:
   *  - tag: PC tag
   *  - cpu: actual cpu
   *  - stride: actual cpu
   */
  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_BERTI] " << __func__;
    std::cout << " tag: " << std::hex << tag << std::dec;
    std::cout << " delta: " << delta;
  }

  auto add_delta = [&](auto delta, auto entry) {
    // Lambda function to add a new element
    delta_t new_delta;
    new_delta.delta = delta;
    new_delta.conf = CONFIDENCE_INIT;
    new_delta.rpl = BERTI_R;
    auto it = std::find_if(std::begin(entry->deltas), std::end(entry->deltas), [](const auto i) { return (i.delta == 0); });
    assert(it != std::end(entry->deltas));
    *it = new_delta;
  };

  if (bertit.find(tag) == bertit.end()) {
    if constexpr (champsim::debug_print)
      std::cout << " allocating a new entry;";

    // We are not tracking this tag
    if (bertit_queue.size() > BERTI_TABLE_SIZE) {
      // FIFO replacent algorithm
      uint64_t key = bertit_queue.front();
      berti* entry = bertit[key];

      if constexpr (champsim::debug_print)
        std::cout << " removing tag: " << std::hex << key << std::dec << ";";

      delete entry; // Free previous entry

      bertit.erase(bertit_queue.front());
      bertit_queue.pop();
    }

    bertit_queue.push(tag); // Add new tag
    assert((bertit.size() <= BERTI_TABLE_SIZE) && "Tracking too much tags");

    // Confidence IP
    berti* entry = new berti;
    entry->conf = CONFIDENCE_INC;

    // Saving the new stride
    add_delta(delta, entry);

    if constexpr (champsim::debug_print)
      std::cout << " confidence: " << CONFIDENCE_INIT << std::endl;

    // Save the new tag
    bertit.insert(std::make_pair(tag, entry));
    return;
  }

  // Get the delta
  berti* entry = bertit[tag];

  for (auto& i : entry->deltas) {
    if (i.delta == delta) {
      // We already track the delta
      i.conf += CONFIDENCE_INC;

      if (i.conf > CONFIDENCE_MAX)
        i.conf = CONFIDENCE_MAX;

      if constexpr (champsim::debug_print)
        std::cout << " confidence: " << i.conf << std::endl;

      return;
    }
  }

  // We have space to add a new entry
  auto ssize = std::count_if(std::begin(entry->deltas), std::end(entry->deltas), [](const auto i) { return i.delta != 0; });

  if (ssize < size) {
    add_delta(delta, entry);
    assert((std::size(entry->deltas) <= size) && "I remember too much deltas");
    return;
  }

  // We find the delta with less confidence
  std::sort(std::begin(entry->deltas), std::end(entry->deltas), compare_rpl);
  if (entry->deltas.front().rpl == BERTI_R || entry->deltas.front().rpl == BERTI_L2R) {
    if constexpr (champsim::debug_print)
      std::cout << " replaced_delta: " << entry->deltas.front().delta << std::endl;
    entry->deltas.front().delta = delta;
    entry->deltas.front().conf = CONFIDENCE_INIT;
    entry->deltas.front().rpl = BERTI_R;
  }
}

uint8_t Berti::get(uint64_t tag, std::vector<delta_t>& res, bool hit)
{
  /*
   * Save the new information into the history table
   *
   * Parameters:
   *  - tag: PC tag
   *
   * Return: the stride to prefetch
   */
  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI_BERTI] " << __func__ << " tag: " << std::hex << tag;
    std::cout << std::dec;
  }

  if (!bertit.count(tag)) {
    if constexpr (champsim::debug_print)
      std::cout << " TAG NOT FOUND" << std::endl;
    parent->no_found_berti++;
    return 0;
  }
  parent->found_berti++;

  if constexpr (champsim::debug_print)
    std::cout << std::endl;

  // We found the tag
  berti* entry = bertit[tag];

  // update misp confidence
  // miss: += 1
  // hit: x -= 99
  // target miss rate: y = 0.99
  // E(miss_conf) = y - (1 - y) * 99 = 0
  if (!hit && entry->miss_conf < BERTI_MISS_CONF_MAX) {
    entry->miss_conf++;
  } else if (hit) {
    entry->miss_conf -= 99;
    if (entry->miss_conf < BERTI_MISS_CONF_MIN) {
      entry->miss_conf = BERTI_MISS_CONF_MIN;
    }
  }

  for (auto& i : entry->deltas)
    if (i.delta != 0 && i.rpl != BERTI_R)
      res.push_back(i);

  if (res.empty() && entry->conf >= LAUNCH_MIDDLE_CONF) {
    // We do not find any delta, so we will try to launch with small confidence
    for (auto& i : entry->deltas) {
      if (i.delta != 0) {
        delta_t new_delta;
        new_delta.delta = i.delta;
        if (i.conf > CONFIDENCE_MIDDLE_L1)
          new_delta.rpl = BERTI_L1;
        else if (i.conf > CONFIDENCE_MIDDLE_L2)
          new_delta.rpl = BERTI_L2;
        else
          continue;
        res.push_back(new_delta);
      }
    }
  }

  // Sort the entries
  std::sort(std::begin(res), std::end(res), compare_greater_delta);
  return 1;
}

void Berti::find_and_update(uint64_t latency, uint64_t tag, uint64_t cycle, uint64_t line_addr)
{
  // We were tracking this miss
  uint64_t tags[HISTORY_TABLE_WAYS];
  uint64_t addr[HISTORY_TABLE_WAYS];
  uint16_t num_on_time = 0;

  // Get the IPs that can launch a prefetch
  num_on_time = historyt->get(latency, tag, line_addr, tags, addr, cycle);

  for (uint32_t i = 0; i < num_on_time; i++) {
    // Increase conf tag
    if (i == 0)
      increase_conf_tag(tag);

    // Add information into berti table
    int64_t stride;
    line_addr &= ADDR_MASK;

    // Usually applications go from lower to higher memory position.
    // The operation order is important (mainly because we allow
    // negative strides)
    stride = (int64_t)(line_addr - addr[i]);

    // delta can be saved in DELTA_WIDTH-bit signed integer
    if (stride < (1 << (DELTA_WIDTH - 1)) && stride >= -(1 << (DELTA_WIDTH - 1))) {
      add(tags[i], stride);
    } else {
      // out of bounds
      parent->delta_too_large++;
    }
  }
}

bool Berti::compare_rpl(delta_t a, delta_t b)
{
  if (a.rpl == BERTI_R && b.rpl != BERTI_R)
    return 1;
  else if (b.rpl == BERTI_R && a.rpl != BERTI_R)
    return 0;
  else if (a.rpl == BERTI_L2R && b.rpl != BERTI_L2R)
    return 1;
  else if (b.rpl == BERTI_L2R && a.rpl != BERTI_L2R)
    return 0;
  else {
    if (a.conf < b.conf)
      return 1;
    else
      return 0;
  }
}

bool Berti::compare_greater_delta(delta_t a, delta_t b)
{
  // Sorted stride when the confidence is full
  if (a.rpl == BERTI_L1 && b.rpl != BERTI_L1)
    return 1;
  else if (a.rpl != BERTI_L1 && b.rpl == BERTI_L1)
    return 0;
  else {
    if (a.rpl == BERTI_L2 && b.rpl != BERTI_L2)
      return 1;
    else if (a.rpl != BERTI_L2 && b.rpl == BERTI_L2)
      return 0;
    else {
      if (a.rpl == BERTI_L2R && b.rpl != BERTI_L2R)
        return 1;
      if (a.rpl != BERTI_L2R && b.rpl == BERTI_L2R)
        return 0;
      else {
        if (std::abs(a.delta) < std::abs(b.delta))
          return 1;
        return 0;
      }
    }
  }
}

uint64_t Berti::ip_hash(uint64_t ip)
{
  /*
   * IP hash function
   */
#ifdef HASH_ORIGINAL
  ip = ((ip >> 1) ^ (ip >> 4)); // Original one
#endif
  // IP hash from here: http://burtleburtle.net/bob/hash/integer.html
#ifdef THOMAS_WANG_HASH_1
  ip = (ip ^ 61) ^ (ip >> 16);
  ip = ip + (ip << 3);
  ip = ip ^ (ip >> 4);
  ip = ip * 0x27d4eb2d;
  ip = ip ^ (ip >> 15);
#endif
#ifdef THOMAS_WANG_HASH_2
  ip = (ip + 0x7ed55d16) + (ip << 12);
  ip = (ip ^ 0xc761c23c) ^ (ip >> 19);
  ip = (ip + 0x165667b1) + (ip << 5);
  ip = (ip + 0xd3a2646c) ^ (ip << 9);
  ip = (ip + 0xfd7046c5) + (ip << 3);
  ip = (ip ^ 0xb55a4f09) ^ (ip >> 16);
#endif
#ifdef THOMAS_WANG_HASH_3
  ip -= (ip << 6);
  ip ^= (ip >> 17);
  ip -= (ip << 9);
  ip ^= (ip << 4);
  ip -= (ip << 3);
  ip ^= (ip << 10);
  ip ^= (ip >> 15);
#endif
#ifdef THOMAS_WANG_HASH_4
  ip += ~(ip << 15);
  ip ^= (ip >> 10);
  ip += (ip << 3);
  ip ^= (ip >> 6);
  ip += ~(ip << 11);
  ip ^= (ip >> 16);
#endif
#ifdef THOMAS_WANG_HASH_5
  ip = (ip + 0x479ab41d) + (ip << 8);
  ip = (ip ^ 0xe4aa10ce) ^ (ip >> 5);
  ip = (ip + 0x9942f0a6) - (ip << 14);
  ip = (ip ^ 0x5aedd67d) ^ (ip >> 3);
  ip = (ip + 0x17bea992) + (ip << 7);
#endif
#ifdef THOMAS_WANG_HASH_6
  ip = (ip ^ 0xdeadbeef) + (ip << 4);
  ip = ip ^ (ip >> 10);
  ip = ip + (ip << 7);
  ip = ip ^ (ip >> 13);
#endif
#ifdef THOMAS_WANG_HASH_7
  ip = ip ^ (ip >> 4);
  ip = (ip ^ 0xdeadbeef) + (ip << 5);
  ip = ip ^ (ip >> 11);
#endif
#ifdef THOMAS_WANG_NEW_HASH
  ip ^= (ip >> 20) ^ (ip >> 12);
  ip = ip ^ (ip >> 7) ^ (ip >> 4);
#endif
#ifdef THOMAS_WANG_HASH_HALF_AVALANCHE
  ip = (ip + 0x479ab41d) + (ip << 8);
  ip = (ip ^ 0xe4aa10ce) ^ (ip >> 5);
  ip = (ip + 0x9942f0a6) - (ip << 14);
  ip = (ip ^ 0x5aedd67d) ^ (ip >> 3);
  ip = (ip + 0x17bea992) + (ip << 7);
#endif
#ifdef THOMAS_WANG_HASH_FULL_AVALANCHE
  ip = (ip + 0x7ed55d16) + (ip << 12);
  ip = (ip ^ 0xc761c23c) ^ (ip >> 19);
  ip = (ip + 0x165667b1) + (ip << 5);
  ip = (ip + 0xd3a2646c) ^ (ip << 9);
  ip = (ip + 0xfd7046c5) + (ip << 3);
  ip = (ip ^ 0xb55a4f09) ^ (ip >> 16);
#endif
#ifdef THOMAS_WANG_HASH_INT_1
  ip -= (ip << 6);
  ip ^= (ip >> 17);
  ip -= (ip << 9);
  ip ^= (ip << 4);
  ip -= (ip << 3);
  ip ^= (ip << 10);
  ip ^= (ip >> 15);
#endif
#ifdef THOMAS_WANG_HASH_INT_2
  ip += ~(ip << 15);
  ip ^= (ip >> 10);
  ip += (ip << 3);
  ip ^= (ip >> 6);
  ip += ~(ip << 11);
  ip ^= (ip >> 16);
#endif
#ifdef ENTANGLING_HASH
  ip = ip ^ (ip >> 2) ^ (ip >> 5);
#endif
  return ip; // No IP hash
}

/******************************************************************************/
/*                        Cache Functions                                     */
/******************************************************************************/
void vberti::prefetcher_initialize()
{
  // Calculate latency table size
  // handle all possible inflight memory requests
  uint64_t latency_table_size = intern_->get_mshr_size();
  for (auto const& i : intern_->get_pq_size())
    latency_table_size += i;

  // New structures
  latencyt = new LatencyTable((int)latency_table_size);
  scache = new ShadowCache(intern_->NUM_SET, intern_->NUM_WAY);
  historyt = new HistoryTable();
  berti = new Berti(BERTI_TABLE_DELTA_SIZE, this, historyt);

  std::cout << "Berti Prefetcher" << std::endl;

  // report space for each structure
  // each latency table entry:
  // 1. addr: LINE_ADDR_WIDTH bits
  // 2. tag: IP_TAG_WIDTH bits
  // 3. time: TIME_WIDTH bits
  // 4. pf: 1 bit
  int latency_table_space = latency_table_size * (LINE_ADDR_WIDTH + IP_TAG_WIDTH + TIME_WIDTH + 1);
  std::cout << "Latency table size: " << latency_table_space << " bits" << std::endl;

  // each shadow cache entry:
  // 1. addr: LINE_ADDR_WIDTH bits
  // 2. lat: LAT_WIDTH bits
  // 3. pf: 1 bit
  // 4. cuckoo filter if VIPT: CUCKOO_FILTER_SETS_LOG2 + CUCKOO_FILTER_FINGERPRINT_WIDTH bits
  int shadow_cache_space = intern_->NUM_SET * intern_->NUM_WAY * (LINE_ADDR_WIDTH + LAT_WIDTH + 1 + CUCKOO_FILTER_SETS_LOG2 + CUCKOO_FILTER_FINGERPRINT_WIDTH);
  std::cout << "Shadow cache size: " << shadow_cache_space << " bits" << std::endl;

  // cuckoo filter size
  int cuckoo_filter_space = CUCKOO_FILTER_SETS * CUCKOO_FILTER_WAYS * (1 + CUCKOO_FILTER_FINGERPRINT_WIDTH);
  std::cout << "Cuckoo filter size: " << cuckoo_filter_space << " bits" << std::endl;

  // each history table entry:
  // 1. tag: IP_TAG_WIDTH
  // 2. addr: LINE_ADDR_WIDTH
  // 3. time: TIME_WIDTH
  // additionaly, log2(HISTORY_TABLE_WAYS) bits per set
  int history_table_space =
      HISTORY_TABLE_SETS * HISTORY_TABLE_WAYS * (IP_TAG_WIDTH + HIST_ADDR_WIDTH + TIME_WIDTH) + HISTORY_TABLE_SETS * HISTORY_TABLE_WAYS_LOG2;
  std::cout << "History table size: " << history_table_space << " bits" << std::endl;

  // each delta:
  // 1. conf: CONFIDENCE_WIDTH bits
  // 2. delta: DELTA_WIDTH bits
  // 3. rpl: 2 bits
  int delta_width = CONFIDENCE_WIDTH + DELTA_WIDTH + 2;
  // fully associative berti table:
  // 1. fifo replacement policy: log2(BERTI_TABLE_SIZE) bits
  // 2. for each entry:
  //    1. array of deltas: delta_width bits
  //    2. conf: CONFIDENCE_WITH bits
  //    3. miss_conf: BERTI_MISS_CONF_WIDTH bits
  //    4. tag: IP_TAG_WIDTH bits
  int berti_table_space =
      BERTI_TABLE_SIZE_LOG2 + BERTI_TABLE_SIZE * (BERTI_TABLE_DELTA_SIZE * delta_width + CONFIDENCE_WIDTH + BERTI_MISS_CONF_WIDTH + IP_TAG_WIDTH);
  std::cout << "Berti table size: " << berti_table_space << " bits" << std::endl;

  // pending target buffer
  // each entry:
  // 1. line addr: LINE_ADDR_WIDTH bits
  // 2. rank conf: CONFIDENCE_WIDTH bits
  // 3. rpl: 2 bits
  // 4. ip hash: IP_TAG_WIDTH bits
  // 5. op generation id: GENERATION_OP_WIDTH bits
  // 6. tries: PENDING_TRIES_WIDTH bits
  // an additional generation id counter: GENERATION_OP_WIDTH bits
  int pending_target_buffer_space =
      PENDING_MAX_SIZE * (LINE_ADDR_WIDTH + CONFIDENCE_WIDTH + 2 + IP_TAG_WIDTH + GENERATION_OP_WIDTH + PENDING_TRIES_WIDTH) + GENERATION_OP_WIDTH;
  std::cout << "Pending table buffer size: " << pending_target_buffer_space << " bits" << std::endl;

  int total_space = latency_table_space + shadow_cache_space + cuckoo_filter_space + history_table_space + berti_table_space + pending_target_buffer_space;
  std::cout << "Total size: " << total_space << " bits = " << total_space / 1024 / 8 << " KiB" << std::endl;

#ifdef NO_CROSS_PAGE
  std::cout << "No Crossing Page" << std::endl;
#endif
#ifdef HASH_ORIGINAL
  std::cout << "BERTI HASH ORIGINAL" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_1
  std::cout << "BERTI HASH 1" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_2
  std::cout << "BERTI HASH 2" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_3
  std::cout << "BERTI HASH 3" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_4
  std::cout << "BERTI HASH 4" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_5
  std::cout << "BERTI HASH 5" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_6
  std::cout << "BERTI HASH 6" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_7
  std::cout << "BERTI HASH 7" << std::endl;
#endif
#ifdef THOMAS_WANG_NEW_HASH
  std::cout << "BERTI HASH NEW" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_HALF_AVALANCHE
  std::cout << "BERTI HASH HALF AVALANCHE" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_FULL_AVALANCHE
  std::cout << "BERTI HASH FULL AVALANCHE" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_INT_1
  std::cout << "BERTI HASH INT 1" << std::endl;
#endif
#ifdef THOMAS_WANG_HASH_INT_2
  std::cout << "BERTI HASH INT 2" << std::endl;
#endif
#ifdef ENTANGLING_HASH
  std::cout << "BERTI HASH ENTANGLING" << std::endl;
#endif
#ifdef FOLD_HASH
  std::cout << "BERTI HASH FOLD" << std::endl;
#endif
  std::cout << "BERTI IP MASK " << std::hex << IP_MASK << std::dec << std::endl;
}

void vberti::prune_pending()
{
  // drop unwanted entries
  pending_targets.erase(std::remove_if(pending_targets.begin(), pending_targets.end(),
                                       [&](const pending_target_t& t) {
                                         // 1. too old
                                         // wrap around
                                         if ((operate_seq - t.gen_op) & GENERATION_OP_MASK > PENDING_WINDOW_OPS) {
                                           pending_drop_age++;
                                           return true;
                                         }
                                         // 2. too many attempts
                                         if (t.tries >= PENDING_MAX_TRIES)
                                           return true;
                                         // 3. already in mshr/pq
                                         if (latencyt->get(t.p_b_addr))
                                           return true;
                                         // 4. already in cache
                                         if (ENABLE_CUCKOO_FILTER && scache->get_cuckoo_filter(t.p_b_addr))
                                           return true;
                                         return false;
                                       }),
                        pending_targets.end());
}

void vberti::enqueue_pending(const pending_target_t& t)
{
  // remove duplicate
  auto it = std::find_if(pending_targets.begin(), pending_targets.end(), [&](const pending_target_t& x) { return x.p_b_addr == t.p_b_addr; });

  if (it != pending_targets.end()) {
    pending_drop_dup++;
    if (t.rank_conf > it->rank_conf)
      it->rank_conf = t.rank_conf;
    if (t.rpl < it->rpl)
      it->rpl = t.rpl;

    it->ip_hash = t.ip_hash;
    it->gen_op = t.gen_op;
    return;
  }

  if (pending_targets.size() < PENDING_MAX_SIZE) {
    pending_targets.push_back(t);
    return;
  }

  // replace worst in buffer
  auto victim = std::min_element(pending_targets.begin(), pending_targets.end(), [&](const pending_target_t& a, const pending_target_t& b) {
    if (a.rank_conf != b.rank_conf)
      return a.rank_conf < b.rank_conf;
    return a.gen_op < b.gen_op;
  });

  if (victim != pending_targets.end() && (t.rank_conf > victim->rank_conf)) {
    *victim = t;
    pending_drop_full++;
  } else {
    pending_drop_full++;
  }
}

int vberti::issue_pending(int budget, bool from_cycle)
{
  int issued = 0;
  bool first_issue_in_this_call = true;

  int max_attempts = budget * 4 + 4;
  int attempts = 0;

  while (issued < budget && attempts < max_attempts) {
    prune_pending();
    if (pending_targets.empty())
      break;

    auto occup = intern_->get_pq_occupancy();
    auto pq_size = intern_->get_pq_size();
    if (occup.back() >= pq_size.back())
      break;

    size_t best = 0;
    for (size_t i = 1; i < pending_targets.size(); i++) {
      const auto& a = pending_targets[i];
      const auto& b = pending_targets[best];
      if (a.rank_conf > b.rank_conf || (a.rank_conf == b.rank_conf && a.gen_op > b.gen_op)) {
        best = i;
      }
    }

    auto current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
    current_cycle &= TIME_MASK;

    auto& t = pending_targets[best];
    uint64_t p_addr = t.p_b_addr << LOG2_BLOCK_SIZE;

    float mshr_load = intern_->get_mshr_occupancy_ratio() * 100;
    bool fill_this_level = (t.rpl == BERTI_L1) && (mshr_load < MSHR_LIMIT);

    if (t.rpl == BERTI_L1 && mshr_load >= MSHR_LIMIT)
      pf_to_l2_bc_mshr++;
    if (fill_this_level)
      pf_to_l1++;
    else
      pf_to_l2++;

    uint32_t metadata = (1 << 30) | (intern_->cpu << 28);
    if (!(throttle && ENABLE_L3_THROTTLE) && prefetch_line(champsim::address{p_addr}, fill_this_level, metadata)) {
      ++average_issued;
      issued++;

      if (first_issue_in_this_call) {
        first_issue_in_this_call = false;
        ++average_num;
      }
      if (fill_this_level && !scache->get(t.p_b_addr)) {
        latencyt->add(t.p_b_addr, t.ip_hash, true, current_cycle);
      }

      pending_targets.erase(pending_targets.begin() + best);
    } else {
      // record failed attempts
      t.tries++;
      if (t.tries >= PENDING_MAX_TRIES) {
        t.tries = PENDING_MAX_TRIES;
      }
      attempts++;
    }
  }
  return issued;
}

void vberti::prefetcher_cycle_operate()
{
  // Calculate average memory bandwidth for debugging
  // not in hardware
  int mbw = get_dram_bw();
  if (average_mbw.num == 0)
    average_mbw.average = (float)mbw;
  else {
    average_mbw.average = average_mbw.average + ((((float)mbw) - average_mbw.average) / average_mbw.num);
  }
  average_mbw.num++;

  if (ENABLE_PENDING_TARGET_BUFFER) {
    // issue pending prefetches in the
    // Pending Target Buffer
    issue_pending(ISSUE_BUDGET_PER_CYCLE, true);
  }
}

uint32_t vberti::prefetcher_cache_operate(champsim::address address, champsim::address ip_addr, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                          uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t ip = ip_addr.to<uint64_t>();
  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE); // Line addr

  // collect stats for debugging,
  // not in hardware
  inst_stats[ip].execution_count++;
  if (!cache_hit) {
    inst_stats[ip].miss_count++;
  }

  if (line_addr == 0)
    return metadata_in;

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI] operate";
    std::cout << " ip: " << std::hex << ip;
    std::cout << " full_address: " << addr;
    std::cout << " line_address: " << line_addr << std::dec << std::endl;
  }

  uint64_t ip_hash = berti->ip_hash(ip) & IP_MASK;

  auto current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  current_cycle &= TIME_MASK;
  if (!cache_hit) // This is a miss
  {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache miss" << std::endl;

    // track the miss for mshr
    latencyt->add(line_addr, ip_hash, false, current_cycle); // Add @ to latency
    historyt->add(ip_hash, line_addr, current_cycle);        // Add to the table
  } else if (cache_hit && scache->is_pf(line_addr))          // Hit bc prefetch
  {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache hit because of pf" << std::endl;

    scache->set_pf(line_addr, false);

    uint64_t latency = scache->get_latency(line_addr); // Get latency
    berti->find_and_update(latency, ip_hash, current_cycle, line_addr);
    historyt->add(ip_hash, line_addr, current_cycle);
  } else {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache hit" << std::endl;
  }

  if (ENABLE_PENDING_TARGET_BUFFER) {
    // get list of deltas
    std::vector<delta_t> deltas;
    deltas.reserve(BERTI_TABLE_DELTA_SIZE);
    berti->get(ip_hash, deltas, (bool)cache_hit);

    auto occup = intern_->get_pq_occupancy();
    auto pq_size = intern_->get_pq_size();

    // ===== pending buffer across cache_operate =====
    // new load instruction sequence id
    operate_seq = (operate_seq + 1) & GENERATION_OP_MASK;

    for (auto i : deltas) {
      // deltas are sorted: once we hit BERTI_R, the rest are low-confidence too
      if (i.rpl == BERTI_R)
        break;

      int64_t target_line = (int64_t)line_addr + i.delta;
      if (target_line <= 0)
        continue;

      uint64_t p_b_addr = (uint64_t)target_line;
      uint64_t p_addr = (uint64_t)target_line << LOG2_BLOCK_SIZE;

      if (latencyt->get(p_b_addr))
        continue;

      // check against groundtruth
      if (ENABLE_CUCKOO_FILTER) {
        bool correct = scache->get(p_b_addr) == scache->get_cuckoo_filter(p_b_addr);
        filter_correct += correct;
        filter_wrong += !correct;

        // do not prefetcher if already in cache (check via cuckoo filter),
        // free up prefetch queue
        if (scache->get_cuckoo_filter(p_b_addr))
          continue;
      }

      if ((p_addr >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE)) {
        cross_page++;
#ifdef NO_CROSS_PAGE
        // We do not cross virtual page
        continue;
#endif
      } else {
        no_cross_page++;
      }

      // enqueue to Pending Target Buffer
      // instead of sending to PQ directly
      pending_target_t t;
      t.p_b_addr = p_b_addr;
      t.rank_conf = i.conf;
      t.rpl = i.rpl;
      t.ip_hash = ip_hash;
      t.gen_op = operate_seq;
      t.tries = 0;

      if (i.rpl == BERTI_L1)
        inst_stats[ip].prefetch_l1_count++;
      else if (i.rpl == BERTI_L2)
        inst_stats[ip].prefetch_l2_count++;
      else if (i.rpl == BERTI_L2R)
        inst_stats[ip].prefetch_l2r_count++;

      enqueue_pending(t);
    }

    // If PQ has free slots now, opportunistically issue immediately (in addition to per-cycle issuing)
    int issued = 0;
    int free_slots = (int)pq_size.back() - (int)occup.back();
    int budget = std::min(free_slots, ISSUE_BUDGET_PER_CYCLE);
    if (budget > 0)
      issued = issue_pending(budget, false);
    return metadata_in;
  }

  // pending target buffer disabled
  std::vector<delta_t> deltas(BERTI_TABLE_DELTA_SIZE);
  berti->get(ip_hash, deltas, (bool)cache_hit);

  bool first_issue = true;
  for (auto i : deltas) {
    uint64_t p_addr = (line_addr + i.delta) << LOG2_BLOCK_SIZE;
    uint64_t p_b_addr = (p_addr >> LOG2_BLOCK_SIZE);

    if (latencyt->get(p_b_addr))
      continue;
    // do not prefetcher if already in cache (check via cuckoo filter),
    // free up prefetch queue
    if (scache->get_cuckoo_filter(p_b_addr))
      continue;
    if (i.rpl == BERTI_R)
      return metadata_in;
    if (p_addr == 0)
      continue;

    if ((p_addr >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE)) {
      cross_page++;
#ifdef NO_CROSS_PAGE
      // We do not cross virtual page
      continue;
#endif
    } else
      no_cross_page++;

    float mshr_load = intern_->get_mshr_occupancy_ratio() * 100;

    bool fill_this_level = (i.rpl == BERTI_L1) && (mshr_load < MSHR_LIMIT);

    if (i.rpl == BERTI_L1 && mshr_load >= MSHR_LIMIT)
      pf_to_l2_bc_mshr++;
    if (fill_this_level)
      pf_to_l1++;
    else
      pf_to_l2++;

    if (i.rpl == BERTI_L1)
      inst_stats[ip].prefetch_l1_count++;
    else if (i.rpl == BERTI_L2)
      inst_stats[ip].prefetch_l2_count++;
    else if (i.rpl == BERTI_L2R)
      inst_stats[ip].prefetch_l2r_count++;

    if (!(throttle && ENABLE_L3_THROTTLE) && prefetch_line(champsim::address{p_addr}, fill_this_level, metadata_in)) {
      ++average_issued;
      if (first_issue) {
        first_issue = false;
        ++average_num;
      }

      if constexpr (champsim::debug_print) {
        std::cout << "[BERTI] operate prefetch delta: " << i.delta;
        std::cout << " p_addr: " << std::hex << p_addr << std::dec;
        std::cout << " this_level: " << +fill_this_level << std::endl;
      }

      if (fill_this_level) {
        if (!scache->get(p_b_addr)) {
          latencyt->add(p_b_addr, ip_hash, true, current_cycle);
        }
      }
    }
  }

  return metadata_in;
}

uint32_t vberti::prefetcher_cache_fill(champsim::address address, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  // L3 request L1 to throttle
  if ((metadata_in >> 30) == 3) {
    int throttle = metadata_in & 1;
    if (throttle != this->throttle) {
      printf("L1 CPU %d throttle %d -> %d\n", intern_->cpu, this->throttle, throttle);
    }
    this->throttle = throttle;
  }

  uint64_t addr = address.to<uint64_t>();
  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE); // Line addr
  uint64_t tag = latencyt->get_tag(line_addr);
  uint64_t cycle = latencyt->del(line_addr);
  uint64_t latency = 0;

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI] fill addr: " << std::hex << line_addr;
    std::cout << " event_cycle: " << cycle;
    std::cout << " prefetch: " << +prefetch << std::endl;
    std::cout << " latency: " << latency << std::endl;
  }

  auto current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  current_cycle &= TIME_MASK;
  if (cycle != 0 && current_cycle > cycle)
    latency = current_cycle - cycle;

  if (latency > LAT_MASK) {
    latency = 0;
    cant_track_latency++;
  } else {
    if (latency != 0) {
      // Calculate average latency
      if (average_latency.num == 0)
        average_latency.average = (float)latency;
      else {
        average_latency.average = average_latency.average + ((((float)latency) - average_latency.average) / average_latency.num);
      }
      average_latency.num++;
    }
  }

  // Add to the shadow cache
  scache->add(set, way, line_addr, prefetch, latency, evicted_addr.to<uint64_t>() >> LOG2_BLOCK_SIZE);

  if (latency != 0 && !prefetch) {
    berti->find_and_update(latency, tag, cycle, line_addr);
  }
  return metadata_in;
}

void vberti::prefetcher_final_stats()
{
  std::cout << "BERTI " << "TO_L1: " << pf_to_l1 << " TO_L2: " << pf_to_l2;
  std::cout << " TO_L2_BC_MSHR: " << pf_to_l2_bc_mshr << std::endl;

  std::cout << "BERTI AVG_MBW: ";
  std::cout << average_mbw.average << std::endl;

  std::cout << "BERTI AVG_LAT: ";
  std::cout << average_latency.average << " NUM_TRACK_LATENCY: ";
  std::cout << average_latency.num << " NUM_CANT_TRACK_LATENCY: ";
  std::cout << cant_track_latency << std::endl;

  std::cout << "BERTI CROSS_PAGE " << cross_page;
  std::cout << " NO_CROSS_PAGE: " << no_cross_page << std::endl;

  std::cout << "BERTI";
  std::cout << " FOUND_BERTI: " << found_berti;
  std::cout << " NO_FOUND_BERTI: " << no_found_berti << std::endl;

  std::cout << "BERTI";
  std::cout << " AVERAGE_ISSUED: " << ((1.0 * average_issued) / average_num);
  std::cout << std::endl;

  std::cout << "BERTI";
  std::cout << " DELTA_TOO_LARGE: " << delta_too_large;
  std::cout << std::endl;

  std::cout << "BERTI";
  std::cout << " PENDING_DROP_AGE: " << pending_drop_age;
  std::cout << " DROP_FULL: " << pending_drop_full;
  std::cout << " DROP_DUP: " << pending_drop_dup;
  std::cout << std::endl;

  std::cout << "BERTI";
  std::cout << " FILTER_CORRECT: " << filter_correct;
  std::cout << " FILTER_WRONG: " << filter_wrong;
  std::cout << std::endl;

  std::vector<uint64_t> keys;
  for (auto& entry : inst_stats) {
    if (entry.second.miss_count >= 100) {
      keys.push_back(entry.first);
    }
  }
  std::sort(keys.begin(), keys.end(), [&](uint64_t a, uint64_t b) { return inst_stats[a].miss_count > inst_stats[b].miss_count; });
  int i = 0;
  for (auto& key : keys) {
    printf("0x%08lx: miss %ld/%ld=%.2lf%% prefetch %ld/%ld/%ld\n", key, inst_stats[key].miss_count, inst_stats[key].execution_count,
           100.0 * (double)inst_stats[key].miss_count / (double)inst_stats[key].execution_count, inst_stats[key].prefetch_l1_count,
           inst_stats[key].prefetch_l2_count, inst_stats[key].prefetch_l2r_count);
    if (i++ > 20)
      break;
  }
}

}; // namespace emender_l1d_ns

void emender_l1d::prefetcher_initialize()
{
  inner = new emender_l1d_ns::vberti(intern_);
  inner->prefetcher_initialize();
}

void emender_l1d::prefetcher_cycle_operate() { inner->prefetcher_cycle_operate(); }

uint32_t emender_l1d::prefetcher_cache_operate(champsim::address address, champsim::address ip_addr, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                               uint32_t metadata_in)
{
  return inner->prefetcher_cache_operate(address, ip_addr, cache_hit, useful_prefetch, type, metadata_in);
}

uint32_t emender_l1d::prefetcher_cache_fill(champsim::address address, long set, long way, uint8_t prefetch, champsim::address evicted_addr,
                                            uint32_t metadata_in)
{
  return inner->prefetcher_cache_fill(address, set, way, prefetch, evicted_addr, metadata_in);
}

void emender_l1d::prefetcher_final_stats() { inner->prefetcher_final_stats(); }