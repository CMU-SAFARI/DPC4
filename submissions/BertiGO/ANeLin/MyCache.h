#include <iostream>
#include "cache.h"
#include "msl/fwcounter.h"

template <typename ENTRY, size_t NUM_SET_BITS, size_t NUM_WAYS, size_t TAG_BITS = 58, bool DRRIP = false>
class MyCache {
  
public:
  MyCache() {
    // Initialize queues for each set
    for (size_t set = 0; set < NUM_SETS; set++) {
      fifo_head[set] = 0;
      for (size_t way = 0; way < NUM_WAYS; way++) {
	cache[set][way].tag = 0;
	rrpv[set][way] = 0;
      }
    }
    brrip_counter = 0;
  }

  // Returns pointer to ENTRY if found, NULL otherwise
  ENTRY* find(const uint64_t line_addr) {
    size_t set = getSetIndex(line_addr);
    for (size_t way = 0; way < NUM_WAYS; way++) {
      if (cache[set][way].tag == getTag(line_addr)) {
	return &cache[set][way];
      }
    }
    return NULL;
  }

  size_t hitWay(const uint64_t line_addr) {
    size_t set = getSetIndex(line_addr);
    for (size_t way = 0; way < NUM_WAYS; way++) {
      if (cache[set][way].tag == getTag(line_addr)) {
	return way;
      }
    }
    assert(false);
  }

  size_t findVictim(const size_t set) {
    if (DRRIP) {
      for (size_t way = 0; way < NUM_WAYS; way++) {
	if (!cache[set][way].tag) {
	  return way;
	}
      }
      auto max = rrpv[set][0];
      size_t victim = 0;
      for (size_t way = 0; way < NUM_WAYS; way++) {
	if (max < rrpv[set][way]) {
	  victim = way;
	  max = rrpv[set][way];
	}
      }
      for (size_t way = 0; way < NUM_WAYS; way++) {
	rrpv[set][way] -= maxRRPV - max;
      }
      return victim;
    } else { // FIFO
      return fifo_head[set];
    }
  }

  void updateReplacementHit(const size_t set, const size_t way) {
    if (DRRIP) {
      rrpv[set][way] = 0; // for cache hit, DRRIP always promotes a cache line to the MRU position
    }
  }
  
  void updateReplacementMiss(const size_t set, const size_t way, uint32_t cpu) {
    if (DRRIP) {
      switch(get_set_type(set)) {
      case set_type::follower:
	if (psel[cpu] > (PSEL_MAX >> 1)) { // follow BRRIP
	  update_brrip(set, way);
	} else { // follow SRRIP
	  update_srrip(set, way);
	}
	break;
      case set_type::brrip_leader:
	ctrUpdate(psel[cpu], false, PSEL_MAX);
	update_brrip(set, way);
	break;
      case set_type::srrip_leader:
	ctrUpdate(psel[cpu], true, PSEL_MAX);
	update_srrip(set, way);
	break;
      }
    } else {
      fifo_head[set] = (fifo_head[set] + 1) % NUM_WAYS;
    }
  }
  
  // Insert or update an entry (returns evicted one)
  // Entry contains the full_line_address until parsed here
  ENTRY insert(ENTRY& entry, uint32_t cpu) {
    ENTRY evicted;
    size_t set = getSetIndex(entry.tag);
    if (find(entry.tag)) { // hit
      updateReplacementHit(set, hitWay(entry.tag));
      return evicted;
    }
    // Miss
    size_t way = findVictim(set);
    evicted = cache[set][way];
    entry.tag = getTag(entry.tag);
    cache[set][way] = entry;
    updateReplacementMiss(set, way, cpu);
    return evicted;
  }

  void print(size_t set) {
    std::cout << "Set " << set << ": ";
    for (size_t way = 0; way < NUM_WAYS; way++) {
      if (cache[set][way].tag) {
	std::cout << way << " " << cache[set][way] << " " << (int)rrpv[set][way] << " | ";
      }
    }
    std::cout << std::endl;
  }

  void print() {
    std::cout << "Cache: " << std::endl;
    for (size_t set = 0; set < NUM_SETS; set++) {
      print(set);
    }
    std::cout << std::endl;
  }

 private:

  static const size_t NUM_SETS = 1 << NUM_SET_BITS;
  static const uint64_t TAG_MASK = ((uint64_t)1 << TAG_BITS) - 1;

  static constexpr unsigned maxRRPV = 3;
  static constexpr unsigned BRRIP_MAX = 32;
  static constexpr unsigned PSEL_WIDTH = 10;
  static constexpr unsigned PSEL_MAX = (1 << PSEL_WIDTH) - 1;
  
  enum class set_type {
    follower, brrip_leader, srrip_leader
  };
  
  [[nodiscard]] set_type get_set_type(long set) {
    long set_sample_rate = 32; // 1 in 32
    if (NUM_SETS < 1024 && NUM_SETS >= 256) { // 1 in 16
      set_sample_rate = 16;
    } else if(NUM_SETS >= 64) { // 1 in 8
      set_sample_rate = 8;
    } else if(NUM_SETS >= 8) { // 1 in 4
      set_sample_rate = 4;
    } else {
      assert(false); // Not enough sets to sample for set dueling
    }
    
    auto mask = set_sample_rate - 1;
    auto shift = champsim::lg2(set_sample_rate);
    auto low_slice = set & mask;
    auto high_slice = (set >> shift) & mask;
    
    // This should return 0 when low_slice == high_slice and 1 ~ (set_sample_rate - 1) otherwise
    long category = (set_sample_rate + low_slice - high_slice) & mask;
    
    switch (category) {
    case 0:
      return set_type::brrip_leader;
    case 1:
      return set_type::srrip_leader;
    default:
      return set_type::follower;
    }
  }
  
  void update_brrip(long set, long way)
  {
    rrpv[set][way] = maxRRPV;
    brrip_counter++;
    if (brrip_counter == BRRIP_MAX) {
      brrip_counter = 0;
      rrpv[set][way] = maxRRPV - 1;
    }
  }

  void update_srrip(long set, long way) {
    rrpv[set][way] = maxRRPV - 1;
  }

  // up-down saturating counter for unsigned
  void ctrUpdate(uint16_t& ctr, bool plus, int nbits) {
    if (plus && ctr < ((1 << nbits) - 1)) ctr++;
    if (!plus && ctr > 0) ctr--;
  }
  
  // Truncate to tag bits
  uint64_t getTag(uint64_t line_addr) const {
    return (line_addr >> NUM_SET_BITS) & TAG_MASK;
  }
  
  // Hash function to map address to set
  size_t getSetIndex(const uint64_t line_addr) const {
    return line_addr % NUM_SETS;
  }

  // Cache storage
  ENTRY cache[NUM_SETS][NUM_WAYS];

  // Replacement
  size_t fifo_head[NUM_SETS]; // Pointer to the last way log2(ways)
  unsigned brrip_counter;
  std::vector<uint16_t> psel{NUM_CPUS, 0};
  uint8_t rrpv[NUM_SETS][NUM_WAYS];
};
