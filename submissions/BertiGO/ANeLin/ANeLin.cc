#include "ANeLin.h"
#include "cache.h"
#include "dpc_api.h"
#include "MyCache.h"

// #44 Pushing the limits of the Berti prefetcher

namespace anelin {

uint64_t current_cycle = 0;
double mean_dram_latency = 178;
bool warmup = true;

// Multicore detection
constexpr uint32_t MULTICORE_SIGNAL = 0xcafe;  // Signal to L1D prefetchers
constexpr int RESET_INTERVAL = 10000000;       // Reset core tracking every 10M fills
int cpu_seen[4] = {0, 0, 0, 0};
int fills = 0;

inline int get_active_cores() {
  return cpu_seen[0] + cpu_seen[1] + cpu_seen[2] + cpu_seen[3];
}
  
// Up saturating counter for unsigned
inline void ctrSatAdd(uint16_t& ctr, int nbits) {
  if (ctr < ((1 << nbits) - 1)) ctr++;
}

#define SHOULD_THROTTLE (warmup || get_dram_bw() >= 6)
#define ACCURACY_RATIO 26

// SAMPLE CACHE

#define SAMPLE_CACHE_WAYS 6 // Same as config
#define SAMPLE_CACHE_SET_BITS_1C 12 // Same as config
#define SAMPLE_CACHE_SET_BITS_4C 14 // Same as config
#define SAMPLE_CACHE_SETS_BITS_USED 12
#define SAMPLE_CACHE_NUM_SETS_USED (1 << SAMPLE_CACHE_SETS_BITS_USED)
#define TAG_BITS 32
#define IP_BITS 32
#define IP_MASK (((uint64_t)1 << IP_BITS) - 1)

enum Status { PREFETCHED, LATE, TIMELY, DEMANDED };
  
struct SampleCacheEntry {
  uint64_t tag = 0; // mandatory for MyCache
  Status status = DEMANDED; // two bits
  uint64_t ip = 0; // ip of the trigger of the access
  uint8_t cpu = 0; // cpu that triggers the access 
};

std::ostream& operator<<(std::ostream& os, const SampleCacheEntry& e) {
  os << "[tag: " << std::hex << e.tag << std::dec << ", status: " << e.status << ", ip: " << std::hex << e.ip << std::dec << ", cpu: " << (int)e.cpu << "]";
  return os;
}

// For the 1C configuration
  MyCache<SampleCacheEntry, SAMPLE_CACHE_SET_BITS_1C, SAMPLE_CACHE_WAYS, TAG_BITS> sample_cache_1c;

// For the 4C configuration
  MyCache<SampleCacheEntry, SAMPLE_CACHE_SET_BITS_4C, SAMPLE_CACHE_WAYS, TAG_BITS> sample_cache_4c;


// RUNNING STATS

#define IP_CACHE_WAYS 8
#define IP_CACHE_SET_BITS 7
#define GLOBAL_COUNTER_BITS 14
#define GLOBAL_EVICTS_PHASE ((1 << GLOBAL_COUNTER_BITS) - 1)
#define PREFETCH_COUNTER_BITS 13
#define PREFETCH_EVICTS_PHASE ((1 << PREFETCH_COUNTER_BITS) - 1)
#define IP_COUNTER_BITS 13
#define IP_EVICTS_PHASE ((1 << IP_COUNTER_BITS) - 1)

#define EVICTED 0
#define USEFUL 1
#define USELESS 2

struct PrefetchStats {
  uint64_t tag = 0; // mandatory for MyCache
  std::array<uint16_t, 3> counters = {0, 0, 0}; // Evicted, useful, useless
  bool enabled = false;
};
  
std::vector<PrefetchStats> global(NUM_CPUS);
std::vector<PrefetchStats> prefetches(NUM_CPUS);
std::vector< MyCache<PrefetchStats, IP_CACHE_SET_BITS, IP_CACHE_WAYS, IP_BITS> > ip_cache(NUM_CPUS);

bool isAboveThreshold(PrefetchStats &ps, uint64_t max) {
  if (SHOULD_THROTTLE) {
    return false;
  }
  return ps.counters[USEFUL] > ps.counters[USELESS] * ACCURACY_RATIO;
}

// Called when Global is active to filter some useless IPs
bool isIPBelowThreshold(size_t cpu, uint64_t ip) {
  if (ip == 0) {
    if (prefetches[cpu].counters[EVICTED] < (PREFETCH_EVICTS_PHASE >> 2)) return false;
    return !isAboveThreshold(prefetches[cpu], PREFETCH_EVICTS_PHASE);
  } else {
    auto entry = ip_cache[cpu].find(ip);
    if (!entry || entry->counters[EVICTED] < (IP_EVICTS_PHASE >> 2)) return false;
    return !isAboveThreshold(*entry, IP_EVICTS_PHASE);
  }
}

bool isIPEnabled(size_t cpu, uint64_t ip) {
  if (global[cpu].enabled && !isIPBelowThreshold(cpu, ip)) return true;
  if (ip == 0) return prefetches[cpu].enabled;
  return ip_cache[cpu].find(ip) && (ip_cache[cpu].find(ip))->enabled;
}
	  
void checkEnable(PrefetchStats &ps, uint64_t max) {
  if (ps.counters[EVICTED] >= max) {
    //std::cout << "STATS " << std::hex << ps.tag << std::dec << " " << ps.counters[EVICTED] << " " << ps.counters[USEFUL] << " " << ps.counters[USELESS] << " " << (int)get_dram_bw() << " " << "shouldThrottle " << SHOULD_THROTTLE;
    //std::cout << ((ps.enabled) ? "ENABLED" : "DISABLED") << std::endl;
    ps.enabled = isAboveThreshold(ps, max);
    //std::cout << " -> " << ((ps.enabled) ? "ENABLED" : "DISABLED") << std::endl;
    // Decrease counters
    for (size_t i = 0; i < 3; i++) {
      ps.counters[i] = (ps.counters[i] >> 1) + (ps.counters[i] >> 2); // By 75%
    }
  }
}

void countStat(size_t cpu, uint64_t ip, size_t stat_num) {
  ctrSatAdd(global[cpu].counters[stat_num], GLOBAL_COUNTER_BITS);
  if (stat_num == EVICTED) checkEnable(global[cpu], GLOBAL_EVICTS_PHASE);
  if (!ip) { // is a prefetch
    ctrSatAdd(prefetches[cpu].counters[stat_num], PREFETCH_COUNTER_BITS);
    if (stat_num == EVICTED) checkEnable(prefetches[cpu], PREFETCH_EVICTS_PHASE);
  } else { // is a load
    auto entry = ip_cache[cpu].find(ip);
    if (entry) {
      ctrSatAdd(entry->counters[stat_num], IP_COUNTER_BITS);
      if (stat_num == EVICTED) checkEnable(*entry, IP_EVICTS_PHASE);
    }
    else {
      PrefetchStats ps;
      ps.tag = ip;
      ps.counters[stat_num]++;
      ip_cache[cpu].insert(ps, cpu);
    }
  }
}

void countStats(Status st, uint64_t ip, uint8_t cpu) {
  countStat(cpu, ip, EVICTED);
  if (st == LATE) {
    if (rand() % 2) countStat(cpu, ip, USEFUL);
  } else if (st == TIMELY) {
    countStat(cpu, ip, USEFUL);
  } else if (st == PREFETCHED) {
    countStat(cpu, ip, USELESS);
  }
}

  
// TIME COMPUTATION

#define TIME_BITS 12
#define TIME_OVERFLOW ((uint64_t)1 << TIME_BITS)
#define TIME_MASK (TIME_OVERFLOW - 1)
  
uint64_t get_latency(uint64_t cycle, uint64_t cycle_prev) {
  uint64_t cycle_masked = cycle & TIME_MASK;
  uint64_t cycle_prev_masked = cycle_prev & TIME_MASK;
  if (cycle_prev_masked > cycle_masked) {
    return (cycle_masked + TIME_OVERFLOW) - cycle_prev_masked;
  }
  return cycle_masked - cycle_prev_masked;
}

  
// ONGOING

#define ONGOING_SIZE 128
#define ONGOING_TAG_BITS 32
#define ONGOING_TAG_MASK (((uint64_t)1 << ONGOING_TAG_BITS) - 1)

// We do not have access to ongoing misses (PQ+MSHR), so we aproximate it using this structure
struct ongoing_entry {
  bool valid; // 1 bit
  uint64_t tag; // ONGOING_TAG_BITS bits
  size_t timestamp; // log2(ONGOING_SIZE) * ONGOING_SIZE
  bool demand; // 1 bit
};

ongoing_entry ongoing_table[ONGOING_SIZE];

void init_ongoing_table() {
  for (size_t i = 0; i < ONGOING_SIZE; i++) {
    ongoing_table[i].valid = 0;
  }
}

size_t find_ongoing_entry(uint64_t line_addr) {
  for (size_t i = 0; i < ONGOING_SIZE; i++) {
    if (ongoing_table[i].tag == (line_addr & ONGOING_TAG_MASK)
	&& ongoing_table[i].valid) return i;
  }
  return ONGOING_SIZE;
}
  
size_t get_older_ongoing_entry() {
  uint64_t longer_latency = 0;
  size_t index = 0;
  for (size_t i = 0; i < ONGOING_SIZE; i++) {
    if (!ongoing_table[i].valid) return i;
    if (get_latency(current_cycle, ongoing_table[i].timestamp) > longer_latency) {
      longer_latency = get_latency(current_cycle, ongoing_table[i].timestamp);
      index = i;
    }
  }
  return index;
}

void add_ongoing_entry(uint64_t line_addr, bool demand) {
  // First find for coalescing
  if (find_ongoing_entry(line_addr) < ONGOING_SIZE) return;
  size_t i = get_older_ongoing_entry();
  ongoing_table[i].valid = true;
  ongoing_table[i].tag = line_addr & ONGOING_TAG_MASK;
  ongoing_table[i].timestamp = current_cycle & TIME_MASK;
  ongoing_table[i].demand = demand;
}

void invalid_ongoing_entry(uint64_t line_addr) {
  size_t index = find_ongoing_entry(line_addr);
  if (index < ONGOING_SIZE) {
    ongoing_table[index].valid = false;
  }
}

bool is_ongoing_request(uint64_t line_addr) {
  return find_ongoing_entry(line_addr) < ONGOING_SIZE;
}

bool is_ongoing_demand_request(uint64_t line_addr) {
  size_t index = find_ongoing_entry(line_addr);
  if (index == ONGOING_SIZE) return false;
  return ongoing_table[index].demand;
}

uint64_t get_latency_ongoing(uint64_t line_addr) {
  size_t index = find_ongoing_entry(line_addr);
  if (index == ONGOING_SIZE) return 0;
  return get_latency(current_cycle, ongoing_table[index].timestamp);
}


// PREFETCHER SIZE

void print_size() {
  uint64_t size = 0;
  uint64_t struct_size = 0;

  // Sampling
  struct_size += SAMPLE_CACHE_NUM_SETS_USED * (champsim::lg2(SAMPLE_CACHE_WAYS)+1 + SAMPLE_CACHE_WAYS * (TAG_BITS + 2 + IP_BITS + champsim::lg2(NUM_CPUS)));
  std::cout << "Sampling size: " << (double)struct_size / (1024 * 8.0) << " KB" << std::endl;
  size += struct_size;
  
  // Running stats
  struct_size = NUM_CPUS * (GLOBAL_COUNTER_BITS * 3 + 1);
  struct_size += NUM_CPUS * (PREFETCH_COUNTER_BITS * 3 + 1);
  struct_size += NUM_CPUS * (1 << IP_CACHE_SET_BITS) * (champsim::lg2(IP_CACHE_WAYS) + IP_CACHE_WAYS * (IP_BITS + (IP_COUNTER_BITS * 3) + 1));
  std::cout << "Stats size: " << (double)struct_size / (1024 * 8.0) << " KB" << std::endl;
  size += struct_size;

  // Ongoing
  struct_size = ONGOING_SIZE * (1 + ONGOING_TAG_BITS + TIME_BITS + 1);
  std::cout << "Ongoing size: " << (double)struct_size / (1024 * 8.0) << " KB" << std::endl;
  size += struct_size;
  
  std::cout << "Total size: " << (double)size / (1024 * 8.0) << " KB" << std::endl;
}

// INTERFACE

} // namespace anelin

using namespace anelin;

void ANeLin::prefetcher_initialize() {
  std::cout << "Adaptive Next Line" << std::endl;
  init_ongoing_table();
  print_size();
}

uint32_t ANeLin::prefetcher_cache_operate(champsim::address addr_, champsim::address ip_, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in)
{
  uint64_t addr = addr_.to<uint64_t>();
  uint64_t line_addr = addr >> LOG2_BLOCK_SIZE;
  uint64_t ip = ip_.to<uint64_t>();
  size_t cpu = intern_->cpu;
  current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  warmup = intern_->warmup;
  
  // if (warmup) return metadata_in; // Bandwidth and latencies are not trustable during warmup 
  
  if (!cache_hit || useful_prefetch) {
    uint64_t pf_addr = (line_addr + 1) << LOG2_BLOCK_SIZE;
    if (isIPEnabled(cpu, ip) && get_active_cores() <= 1) {
      prefetch_line(champsim::address{pf_addr}, true, metadata_in);
    }
    add_ongoing_entry(pf_addr >> LOG2_BLOCK_SIZE, false);
    size_t set = (pf_addr >> LOG2_BLOCK_SIZE) % ((1 << SAMPLE_CACHE_SET_BITS_1C) * NUM_CPUS);
    if (set < SAMPLE_CACHE_NUM_SETS_USED) { // We only implement a subset of sets due to storage restrictions
      SampleCacheEntry entry{pf_addr >> LOG2_BLOCK_SIZE, PREFETCHED, ip & IP_MASK, cpu};
      auto evicted_entry = (NUM_CPUS == 1) ? sample_cache_1c.insert(entry, cpu) : sample_cache_4c.insert(entry, cpu);
      if (evicted_entry.tag) countStats(evicted_entry.status, evicted_entry.ip, evicted_entry.cpu);
    }
  }
  
  size_t set = line_addr % ((1 << SAMPLE_CACHE_SET_BITS_1C) * NUM_CPUS);
  if (set < SAMPLE_CACHE_NUM_SETS_USED) { // We only implement a subset of sets due to storage restrictions
    auto entry = (NUM_CPUS == 1) ? sample_cache_1c.find(line_addr) : sample_cache_4c.find(line_addr);
    if (entry) { // Hit
      if (entry->status == PREFETCHED) {
	if (cache_hit && !useful_prefetch) {
	  entry->status = DEMANDED;
	} else if (!cache_hit || (cache_hit && useful_prefetch)) {
	  if (!is_ongoing_request(line_addr) || get_latency_ongoing(line_addr) > mean_dram_latency) {
	    entry->status = TIMELY;
	  } else if (get_latency_ongoing(line_addr) > mean_dram_latency * 0.2) {
	    entry->status = LATE;
	  } else {
	    entry->status = DEMANDED; // Too late to consider it
	  }
	} else {
	  assert(false);
	}
      }
    }
  }
  
  // Add miss in the ongoing table
  if (!cache_hit) {
    add_ongoing_entry(line_addr, true);
  }
  
  if (set < SAMPLE_CACHE_NUM_SETS_USED) { // We only implement a subset of sets due to storage restrictions
    SampleCacheEntry new_entry{line_addr, DEMANDED, ip & IP_MASK};
    auto evicted_entry = (NUM_CPUS == 1) ? sample_cache_1c.insert(new_entry, cpu) : sample_cache_4c.insert(new_entry, cpu);
    if (evicted_entry.tag) countStats(evicted_entry.status, evicted_entry.ip, evicted_entry.cpu);
  }
  
  return metadata_in;
}

uint32_t ANeLin::prefetcher_cache_fill(champsim::address addr_, long set, long way, uint8_t prefetch, champsim::address evicted_addr_, uint32_t metadata_in)
{
  uint64_t addr = addr_.to<uint64_t>();
  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE);
  current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;

  // Compute the moving average of the dram latency
  if (addr) {
    uint64_t latency = get_latency_ongoing(line_addr);
    if (latency > 10 && is_ongoing_demand_request(line_addr)) { // To avoid warmup small latencies
      mean_dram_latency = mean_dram_latency * 0.95 + latency * 0.05;
    }
    invalid_ongoing_entry(line_addr); // Not ongoing anymore
  }

  // MULTICORE DETECTION
  // Periodic reset to adapt to phase changes
  if (++fills >= RESET_INTERVAL) {
    for (int i = 0; i < 4; i++)
      cpu_seen[i] = 0;
    fills = 0;
  }

  // Track which cores have accessed the shared LLC
  auto cpu = intern_->cpu;
  if (cpu < 4)
    cpu_seen[cpu] = 1;

  // Signal multicore to L1D if more than one core detected
  int cores_active = cpu_seen[0] + cpu_seen[1] + cpu_seen[2] + cpu_seen[3];
  if (cores_active > 1) {
    return MULTICORE_SIGNAL;
  }
  // =========================================================================
 
  return metadata_in;
}

void ANeLin::prefetcher_final_stats() {
}
