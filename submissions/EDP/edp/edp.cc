/*
 * The Entangling Data Prefetcher (EDP)
 *
 * Submission #49
 *
 * 4 Data Prefetching Championship
 */
#include "edp.h"

#include <queue>
#include <unordered_set>

#include "dpc_api.h"
#include "filter.h"

// INFO: This is a trick because EDP was first developed for an oldest version
// of champsim
#define NUM_CPUS 16

#include "cache.h"

// Lat Addr
constexpr uint64_t lastAddrSet = 1;
constexpr uint64_t lastAddrWay = 32;

// Entangling table
constexpr int entanglingTableSet = 1;
constexpr int entanglingTableWay = 64;
constexpr int entanglingSize = 4;

// Delta Table
constexpr uint64_t deltaTableSets = 1;
constexpr uint64_t deltaTableWays = 512;

// Histories
constexpr int setDataHistory = 32;
constexpr int wayDataHistory = 32;
constexpr int sizeInstructionHistory = 32;

constexpr int numFindIPTriggers = 4;

// Masks
constexpr int addrMask = 0xFFF;
constexpr int cycleMask = 0xFFFF;
constexpr int ipTagMask = 0x3FFF;

// Size of delta
constexpr int maxStride = 128;

// Number of deltas
constexpr int numDeltas = 16;

int maxConf = 16;
int confL1D = 7;
int confL2C = 5;
int confL2R = 5;

std::array<int, NUM_CPUS> num_issues = {2};
std::array<long long, NUM_CPUS> previous_retired = {0};
std::array<long long, NUM_CPUS> cycles = {0};
std::array<long long, NUM_CPUS> average_instr = {0};
std::array<bool, NUM_CPUS> first_time = {0};
std::array<long long, NUM_CPUS> big_cycles = {0};

// MSHR limit
constexpr double mshr_limit = 0.5;

// Stats
class Stats {
public:
  uint64_t not_found_trigger = 0;
  uint64_t found_trigger = 0;

  // Num Issues IP
  uint64_t num_issues_per_IP = 0;
  uint64_t total_issues_per_IP = 0;

  // Entangling entries average use
  uint64_t entangling_total = 0;
  uint64_t entangling_num = 0;

  uint64_t entangling_full = 0;
  uint64_t entangling_add = 0;
  uint64_t entangling_merge = 0;

  // Level
  uint64_t to_L1D = 0;
  uint64_t to_L2C = 0;
  uint64_t to_L2C_bc_high_use = 0;
  uint64_t to_L2C_bc_low_use = 0;

  // Late
  uint64_t late = 0;
  uint64_t timely = 0;

  // Issues per trigger call
  uint64_t issues_per_trigger_total = 0;
  uint64_t issues_per_trigger_num = 0;

  std::unordered_map<int, uint64_t> histogram_strides;
};
Stats stats;

class DataHistoryEntry {
public:
  uint64_t tag = 0;
  uint64_t addr = 0;
  uint64_t cycle = 0;
  DataHistoryEntry() = default;
  DataHistoryEntry(uint64_t _tag, uint64_t _addr, uint64_t _cycle)
      : tag(_tag), addr(_addr), cycle(_cycle) {}
};

std::array<std::map<uint64_t, std::deque<DataHistoryEntry>>, NUM_CPUS>
    dataHistory;
std::array<BloomFilter, NUM_CPUS> bitMap;

// INFO: this is a virtual PQ for L1 and L2 prefetch requests
std::array<std::queue<uint64_t>, NUM_CPUS> edp_vpq_to_l1;
std::array<std::unordered_set<uint64_t>, NUM_CPUS> edp_vpq_to_l1_set;
std::array<std::queue<uint64_t>, NUM_CPUS> edp_vpq_to_l2;
std::array<std::unordered_set<uint64_t>, NUM_CPUS> edp_vpq_to_l2_set;
std::array<int, NUM_CPUS> numcores;
const int edp_vpq_size = 16;

class InstructionHistory {
public:
  class InstructionHistoryEntry {
  public:
    uint64_t ip = 0;
    uint64_t cycle = 0;

    bool operator==(uint64_t other) { return ip == other; }
    InstructionHistoryEntry() = default;
    InstructionHistoryEntry(uint64_t _ip, uint64_t _cycle)
        : ip(_ip), cycle(_cycle) {}
  };

  std::deque<InstructionHistoryEntry> entries;
  // This is to avoid a 128 associativity search
  InstructionHistory() = default;
};

std::array<InstructionHistory, NUM_CPUS> instructionHistory;

class DeltaEntry {
public:
  // I is just inserted
  enum GoToLevel { L1D = 1, L2, L2R, I, R };
  class DeltaInnerEntry {
  public:
    int delta = 0;
    uint8_t conf = 0;
    bool used = false;
    GoToLevel level = GoToLevel::R;
    uint64_t tag = 0; // Debug purpose

    // Placeholder to the address that we should trigger
    uint64_t pfaddr = 0;
    DeltaInnerEntry() : used(false), level(GoToLevel::R) {}
    DeltaInnerEntry(int _delta)
        : delta(_delta), used(true), level(GoToLevel::I) {}
    bool operator==(const DeltaInnerEntry &b) {
      return b.delta == delta && used;
    };
    bool operator==(const int b) { return b == delta && used; };
    bool operator<(const DeltaInnerEntry &b) {
      if (b.level == level) {
        return abs(delta) < abs(b.delta);
      }
      // Levels are enum with numbers, higher means more changes to remove so
      // min
      return level < b.level;
    }
  };
  uint64_t tag = 0;
  uint8_t confGlobal = 0;
  uint8_t late = 0;
  uint8_t timely = 0;
  uint8_t skip = 0;
  std::array<DeltaInnerEntry, numDeltas> deltas;

  bool operator==(const DeltaEntry &b) { return b.tag == tag; };
  bool operator==(const uint64_t b) { return b == tag; };

  DeltaEntry() = default;
  DeltaEntry(uint64_t _tag) : tag(_tag) {};
};

std::array<Cache_Berti<DeltaEntry, deltaTableSets, deltaTableWays>, NUM_CPUS>
    deltaTable;

class LastAddrEntry {
public:
  // Allowing multiple lastAddr
  uint64_t lastAddr = 0;
  uint64_t tag = 0;
  int dir = 0; // 1 up, 0 equal, -1 down
  // Pointer to Delta Table
  uint64_t set = 0;
  uint64_t way = 0;
  LastAddrEntry() = default;
  LastAddrEntry(uint64_t _tag) : tag(_tag) {};
};
// Last addr table (L@T)
std::array<Cache_Berti<LastAddrEntry, lastAddrSet, lastAddrWay>, NUM_CPUS>
    lastAddrTable;

class EntanglingEntry {
public:
  uint64_t tag = 0;
  std::array<std::optional<std::pair<uint64_t, uint64_t>>, entanglingSize>
      entries;
  EntanglingEntry() = default;
  EntanglingEntry(uint64_t _tag) : tag(_tag) {};
};

std::array<Cache_Berti<EntanglingEntry, entanglingTableSet, entanglingTableWay>,
           NUM_CPUS>
    entanglingTable;

class CacheEntry {
public:
  uint64_t addr = 0;
  uint64_t lat = 0;
  bool pf = false;
  uint64_t ipTag = 0;

  CacheEntry() = default;
  CacheEntry(uint64_t _addr, uint64_t _lat, bool _pf)
      : addr(_addr), lat(_lat), pf(_pf) {}
  CacheEntry(uint64_t _addr, uint64_t _lat, bool _pf, uint64_t _ipTag)
      : addr(_addr), lat(_lat), pf(_pf), ipTag(_ipTag) {}
  CacheEntry(uint64_t _addr) : addr(_addr) {}
  bool operator==(const CacheEntry &b) { return b.addr == addr; }
};

// Tracking latencies, this structures should be part of the PQ/RQ/MSHR
std::array<std::unordered_map<uint64_t, uint64_t>, NUM_CPUS> trackLatency;
std::array<std::unordered_set<uint64_t>, NUM_CPUS> trackPF;
std::array<std::unordered_map<uint64_t, uint64_t>, NUM_CPUS> addrToIp;

// Cache
std::array<std::vector<std::vector<CacheEntry>>, NUM_CPUS> shadow_cache;
std::array<std::array<bool, 128>, NUM_CPUS> shadow_tlb;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Training process
// Find the trigger IP
std::array<uint64_t, numFindIPTriggers>
train_found_trigger_IP(uint64_t initCycle, uint64_t ipTag, uint32_t cpu) {
  std::array<uint64_t, numFindIPTriggers> triggerIP;
  int dx = 0;

  for (auto it = std::rbegin(instructionHistory[cpu].entries);
       it != std::rend(instructionHistory[cpu].entries); ++it) {
    if (initCycle < it->cycle) {
      continue;
    }
    triggerIP[dx] = it->ip;
    dx++;
    if (dx == numFindIPTriggers)
      break;
  }

  // There is no IP that can trigger this
  if (dx == 0) {
    stats.not_found_trigger++;
    // If we do not found trigger ip we choose as trigger the same IP
    triggerIP[dx] = ipTag;
  } else
    stats.found_trigger++;

  return triggerIP;
}

// Find deltas
int train_found_deltas(uint64_t initCycle, uint64_t ipTag, uint64_t addrTag,
                       bool diff_ip, uint32_t cpu,
                       std::vector<std::optional<int>> &strides) {
  uint64_t indexHistory = ipTag % setDataHistory;
  int numDeltasFound = 0;

  // Direction: 0 not found, 1 positive, -1 negative
  int dir = 0;
  // Get the latency
  for (auto it = std::rbegin(dataHistory[cpu][indexHistory]);
       it != std::rend(dataHistory[cpu][indexHistory]); ++it) {
    const auto &entry = *it;
    int stride = addrTag - entry.addr;

    if (ipTag != entry.tag)
      continue;

    if (initCycle < entry.cycle) {
      continue;
    }

    if (stride == 0 && diff_ip)
      break;

    if (abs(stride) > maxStride)
      continue;

    if (stride < 0 && dir == 0)
      dir = -1;
    else if (stride >= 0 && dir == 0)
      dir = 1;

    strides.emplace_back(stride);
    ++stats.histogram_strides[stride];

    // Maximum strides the number of deltas
    numDeltasFound++;
    if (numDeltasFound > numDeltas)
      break;

    if (stride == 0)
      break;
  }

  return dir;
}

// Train deltas and increase confidence
void train_confidence_deltas(uint64_t ipTag, uint32_t cpu,
                             std::vector<std::optional<int>> &strides) {
  auto entryDeltaC = deltaTable[cpu].get(ipTag);
  if (!entryDeltaC.has_value()) {
    deltaTable[cpu].insert(ipTag, new DeltaEntry(ipTag));
  }
  auto entryDelta = deltaTable[cpu].get(ipTag).value();

  // Fin the entry
  entryDelta->confGlobal++;

  // Select the delta
  std::for_each(std::begin(strides), std::end(strides),
                [&entryDelta](const auto &stride) {
                  if (!stride.has_value())
                    return;

                  // Is this delta already there?
                  auto specificDelta =
                      std::find(std::begin(entryDelta->deltas),
                                std::end(entryDelta->deltas), stride.value());
                  if (specificDelta == std::end(entryDelta->deltas)) {
                    auto rpl = std::find_if(
                        std::begin(entryDelta->deltas),
                        std::end(entryDelta->deltas), [](const auto &i) {
                          return i.level == DeltaEntry::GoToLevel::R ||
                                 i.level == DeltaEntry::GoToLevel::L2R;
                        });

                    if (rpl == std::end(entryDelta->deltas)) {
                      return;
                    }

                    // Replace delta
                    rpl->delta = stride.value();
                    rpl->conf = 1;
                    rpl->used = true;
                    rpl->level = DeltaEntry::GoToLevel::I;
                    return;
                  } else {
                    specificDelta->conf++;
                    if (specificDelta->conf > maxConf)
                      specificDelta->conf = maxConf;
                  }
                });

  // Max confidence, set confidence
  if (entryDelta->confGlobal == maxConf) {
    std::for_each(std::begin(entryDelta->deltas), std::end(entryDelta->deltas),
                  [ipTag](auto &entry) {
                    if (entry.conf >= confL1D)
                      entry.level = DeltaEntry::GoToLevel::L1D;
                    else if (entry.conf >= confL2C)
                      entry.level = DeltaEntry::GoToLevel::L2;
                    else if (entry.conf >= confL2R)
                      entry.level = DeltaEntry::GoToLevel::L2R;
                    else
                      entry.level = DeltaEntry::GoToLevel::R;
                    entry.conf = 0;
                  });
    entryDelta->confGlobal = 0;
  }

  return;
}

void train_last_addr_table(uint64_t ipTag, uint64_t set, uint64_t way,
                           uint64_t blockAddr, uint32_t cpu, int dir) {
  // Set the last address seeing by this ip
  auto lastAddrEntryC = lastAddrTable[cpu].get(ipTag);
  if (!lastAddrEntryC.has_value()) {
    lastAddrTable[cpu].insert(ipTag, new LastAddrEntry(ipTag));
  }
  lastAddrEntryC = lastAddrTable[cpu].get(ipTag);
  auto lastAddrEntry = lastAddrEntryC.value();

  // If not we should update addr and pointer, now only addr since we save all
  // options
  lastAddrEntry->tag = ipTag;
  // Check address to update @
  if (lastAddrEntry->dir == dir) {
    lastAddrEntry->lastAddr = blockAddr;
  } else {
    lastAddrEntry->dir = dir;
  }
  if (lastAddrEntry->lastAddr == 0) {
    lastAddrEntry->lastAddr = blockAddr;
    lastAddrEntry->dir = dir;
  }

  lastAddrEntry->set = set;
  lastAddrEntry->way = way;

  return;
}

void train_entangling_table(uint64_t triggerIP, uint64_t ipTag, uint32_t cpu,
                            uint64_t set, uint64_t way) {
  // Set the entangling prefetcher
  auto entanglingEntryC = entanglingTable[cpu].get(triggerIP);
  if (!entanglingEntryC.has_value()) {
    entanglingTable[cpu].insert(triggerIP, new EntanglingEntry(triggerIP));
  }
  entanglingEntryC = entanglingTable[cpu].get(triggerIP);
  auto entanglingEntry = entanglingEntryC.value();

  // Find if the entry is already there
  auto entry = std::find_if(
      std::begin(entanglingEntry->entries), std::end(entanglingEntry->entries),
      [ipTag, cpu](const auto &i) {
        return i.has_value() &&
               lastAddrTable[cpu]
                       .get(std::get<0>(i.value()), std::get<1>(i.value()))
                       ->tag == ipTag;
      });

  if (entry == std::end(entanglingEntry->entries)) {
    // Find the first element that is null to add it
    entry = std::find_if(std::begin(entanglingEntry->entries),
                         std::end(entanglingEntry->entries),
                         [](const auto &i) { return !i.has_value(); });

    if (entry != std::end(entanglingEntry->entries)) {
      // Set the value
      entry->emplace(std::pair<uint64_t, uint64_t>(set, way));
      stats.entangling_add++;
    } else {
      stats.entangling_full++;
    }
  } else {
    stats.entangling_merge++;
  }
}

void train(uint64_t blockAddr, uint64_t initCycle, uint64_t ipTag,
           uint64_t addrTag, uint64_t current_cycle, edp *cache, int32_t cpu) {
  auto endOfTrain = [cpu, ipTag, addrTag, current_cycle](auto indexHistory) {
    // Add into the history
    dataHistory[cpu][indexHistory].emplace_back(ipTag, addrTag, current_cycle);
    // Remove last element if necessary
    if (std::size(dataHistory[cpu][indexHistory]) > wayDataHistory)
      dataHistory[cpu][indexHistory].pop_front();
  };

  uint64_t indexHistory = ipTag % setDataHistory;

  // Remove the element
  trackLatency[cpu].erase(blockAddr);
  addrToIp[cpu].erase(blockAddr);

  // There is no ip, is a translation
  if (ipTag == 0) {
    return;
  }

  auto triggerIPs = train_found_trigger_IP(initCycle, ipTag, cpu);

  // Find strides
  std::vector<std::optional<int>> strides(wayDataHistory, std::nullopt);
  bool endTrain = true;
  int dir = 0;
  bool diff_ip = false;
  for (auto triggerIP : triggerIPs) {
    if (triggerIP == 0)
      continue;
    if (triggerIP != ipTag)
      diff_ip = true;
  }

  dir = train_found_deltas(initCycle, ipTag, addrTag, diff_ip, cpu, strides);
  if (dir != 0)
    endTrain = false;

  // Not stride found
  if (endTrain) {
    endOfTrain(indexHistory);
    return;
  }

  // Train deltas
  train_confidence_deltas(ipTag, cpu, strides);

  auto [set, way] = deltaTable[cpu].getSetWay(ipTag);

  // Train last addr table
  train_last_addr_table(ipTag, set, way, blockAddr, cpu, dir);

  auto [set_, way_] = lastAddrTable[cpu].getSetWay(ipTag);

  // Train entangling table
  for (auto triggerIP : triggerIPs) {
    if (triggerIP == 0)
      continue;
    train_entangling_table(triggerIP, ipTag, cpu, set_, way_);
  }
  // Add into the history
  endOfTrain(indexHistory);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Training process
/*
 * Return the number of prefetch that we should issue per every ip
 */
int trigger_Issues_per_ip(uint64_t triggerIP, edp *cache, int32_t cpu) {
  // Get occupancy and max. size of the PQ
  int total = 0;

  auto entanglingEntryC = entanglingTable[cpu].get(triggerIP);
  if (!entanglingEntryC.has_value()) {
    return 0;
  }
  auto entanglingEntry = entanglingEntryC.value();

  for (auto &j : entanglingEntry->entries) {
    if (!j.has_value()) {
      break;
    }
    ++total;
  }

  // Add stats
  stats.entangling_total += total;
  stats.entangling_num++;

  if (total == 0)
    return 0;

  // Add to stats
  stats.num_issues_per_IP++;

  return numDeltas;
}

int trigger_get_deltas_and_addr_from_entangling(
    uint64_t triggerIP,
    std::array<std::array<DeltaEntry::DeltaInnerEntry, numDeltas>,
               entanglingSize> &pfRequest_Matrix,
    uint32_t cpu, uint64_t num_issues_per_IP) {
  // Avoid duplicate address
  std::set<uint64_t> savedPF;

  int indexPfIp = 0;

  auto entanglingEntryC = entanglingTable[cpu].get(triggerIP);
  if (!entanglingEntryC.has_value()) {
    return 0;
  }
  auto entanglingEntry = entanglingEntryC.value();

  for (auto &j : entanglingEntry->entries) {
    // This entry has value
    if (!j.has_value()) {
      continue;
    }

    auto pair = j.value();
    auto entry = lastAddrTable[cpu].get(std::get<0>(pair), std::get<1>(pair));
    // std::cout << std::hex << entry->tag << std::endl;

    // Iterate over all base addr
    uint64_t baseAddr = entry->lastAddr;
    auto deltaEntry = deltaTable[cpu].get(entry->set, entry->way);

    // Sorted to insert by priority
    std::sort(std::begin(deltaEntry->deltas), std::end(deltaEntry->deltas));

    uint64_t pfRequestIP = 0;
    int indexPfRequest = 0;
    for (const auto delta : deltaEntry->deltas) {
      uint64_t pfaddr = (baseAddr + delta.delta) << LOG2_BLOCK_SIZE;
      uint64_t pfBlock = pfaddr >> LOG2_BLOCK_SIZE;
      if (delta.level == DeltaEntry::GoToLevel::R ||
          delta.level == DeltaEntry::GoToLevel::I)
        continue;

      // Do not save duplicate ones
      if (savedPF.count(pfaddr))
        continue;

      if (pfRequestIP >= num_issues[cpu])
        break;

      addrToIp[cpu][pfBlock] = deltaEntry->tag;

      pfRequest_Matrix[indexPfIp][indexPfRequest] = delta;
      pfRequest_Matrix[indexPfIp][indexPfRequest].pfaddr = pfaddr;
      pfRequest_Matrix[indexPfIp][indexPfRequest].tag = deltaEntry->tag;
      savedPF.insert(pfaddr);
      ++indexPfRequest;
      ++pfRequestIP;
    }
    // Sort elements
    std::sort(std::begin(pfRequest_Matrix[indexPfIp]),
              std::end(pfRequest_Matrix[indexPfIp]));

    j = std::nullopt;
    indexPfIp++;
  }
  return indexPfIp;
}

void trigger_interleave_pf_requests(
    std::array<DeltaEntry::DeltaInnerEntry, numDeltas * entanglingSize>
        &pfRequest,
    std::array<std::array<DeltaEntry::DeltaInnerEntry, numDeltas>,
               entanglingSize> &pfRequest_Matrix,
    int indexPfIp, uint64_t num_issues_per_IP) {
  auto level = DeltaEntry::L1D;
  int pfRequestDx = 0;

  while (true) {
    for (uint64_t i = 0; i < num_issues_per_IP; i++) {
      for (int j = 0; j < indexPfIp; j++) {
        // Add into the list of the prefetchers that will be issued
        if (pfRequest_Matrix[j][i].level == level) {
          pfRequest[pfRequestDx] = pfRequest_Matrix[j][i];
          pfRequestDx++;
        }
      }
    }
    if (level == DeltaEntry::L1D)
      level = DeltaEntry::L2;
    else if (level == DeltaEntry::L2)
      level = DeltaEntry::L2R;
    else if (level == DeltaEntry::L2R)
      break;
  }
}
// Trigger prefetch request
void trigger(edp *cache, uint64_t ipTag, uint64_t current_cycle, int32_t cpu) {
  auto entanglingEntryC = entanglingTable[cpu].get(ipTag);
  if (!entanglingEntryC.has_value()) {
    return;
  }

  // How many Pf should we issue
  int num_issues_per_IP = trigger_Issues_per_ip(ipTag, cache, cpu);

  auto pfRequest_Matrix =
      std::array<std::array<DeltaEntry::DeltaInnerEntry, numDeltas>,
                 entanglingSize>();
  int indexPfIp = trigger_get_deltas_and_addr_from_entangling(
      ipTag, pfRequest_Matrix, cpu, num_issues_per_IP);

  // We interleave prefetch request, so firt all prefeetch that goes to L1D are
  // prefetched, then all that goes to L2
  auto pfRequest =
      std::array<DeltaEntry::DeltaInnerEntry, numDeltas * entanglingSize>();
  trigger_interleave_pf_requests(pfRequest, pfRequest_Matrix, indexPfIp,
                                 num_issues_per_IP);

  for (auto delta : pfRequest) {
    bool fillThisLevel = delta.level == DeltaEntry::L1D;

    if (delta.pfaddr == 0)
      continue;

    if (delta.level == DeltaEntry::R || delta.level == DeltaEntry::I)
      continue;

    uint64_t pfBlock = (delta.pfaddr >> LOG2_BLOCK_SIZE);

    // Inserts into the virtual pq, from now unlimited size
    if (fillThisLevel) {
      if (!edp_vpq_to_l1_set[cpu].count(pfBlock)) {
        if (std::size(edp_vpq_to_l1[cpu]) == edp_vpq_size) {
          edp_vpq_to_l1_set[cpu].erase(edp_vpq_to_l1[cpu].front() >>
                                       LOG2_BLOCK_SIZE);
          edp_vpq_to_l1[cpu].pop();
        }
        edp_vpq_to_l1[cpu].push(delta.pfaddr);
        edp_vpq_to_l1_set[cpu].insert(pfBlock);
      }
    } else {
      if (!edp_vpq_to_l2_set[cpu].count(pfBlock) &&
          !edp_vpq_to_l1_set[cpu].count(pfBlock)) {
        if (std::size(edp_vpq_to_l2[cpu]) == edp_vpq_size) {
          edp_vpq_to_l2_set[cpu].erase(edp_vpq_to_l2[cpu].front() >>
                                       LOG2_BLOCK_SIZE);
          edp_vpq_to_l2[cpu].pop();
        }
        edp_vpq_to_l2[cpu].push(delta.pfaddr);
        edp_vpq_to_l2_set[cpu].insert(pfBlock);
      }
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void edp::prefetcher_initialize() {
  std::string _NAME = "EDP";
  // std::cout << _NAME << " BitMAP Mask: " << std::hex << bitMapEdpMask <<
  // std::dec << std::endl;
  std::cout << _NAME << " Max Conf: " << maxConf << " Conf to L1D: " << confL1D
            << " Conf to L2: " << confL2C << std::endl;
  std::cout << _NAME << " Num Deltas: " << numDeltas
            << " Max. Stride: " << maxStride << std::endl;
  std::cout << _NAME << " Masks IP_Tag: " << std::hex << ipTagMask
            << " Cycle: " << cycleMask << " Addr: " << addrMask << std::dec
            << std::endl;
  std::cout << _NAME << " Sizes Instruction History: " << sizeInstructionHistory
            << std::endl;
  std::cout << _NAME << " Way Data History: " << wayDataHistory
            << " Set Data History: " << setDataHistory << std::endl;
  std::cout << _NAME << " Entangling Table Inner: " << entanglingSize
            << " Entangling Set: " << entanglingTableSet
            << " Entangling Ways: " << entanglingTableWay << std::endl;
  std::cout << _NAME << " Last Addr Table Set: " << lastAddrSet
            << " Last Addr Table Way: " << lastAddrWay << std::endl;
  std::cout << _NAME << " Delta Table Size: " << deltaTableSets
            << " Ways: " << deltaTableWays << std::endl;
  for (std::size_t j = 0; j < NUM_CPUS; j++) {
    shadow_cache[j].resize(intern_->NUM_SET);
    std::for_each(std::begin(shadow_cache[j]), std::end(shadow_cache[j]),
                  [this](auto &i) { i.resize(intern_->NUM_WAY); });
    num_issues[j] = 16;
    numcores[j] = 1;
  }
}

uint32_t edp::prefetcher_cache_operate(champsim::address addr,
                                       champsim::address ip, uint8_t cache_hit,
                                       bool useful_prefetch, access_type type,
                                       uint32_t metadata_in) {
  uint64_t address = addr.to<uint64_t>();
  uint64_t ip_ = ip.to<uint64_t>();
  auto current_cycle =
      intern_->current_time.time_since_epoch() / intern_->clock_period;
  int32_t cpu = intern_->cpu;

  uint64_t blockAddr = (address >> LOG2_BLOCK_SIZE);
  uint64_t ipTag = ip_ ^ (ip_ >> 2) ^ (ip_ >> 5);
  ipTag = ipTag & ipTagMask;
  uint64_t addrTag = blockAddr & addrMask;
  uint64_t cycle = current_cycle & cycleMask;

  // bitMap[cpu][blockAddr & bitMapEdpMask] = true;
  bitMap[cpu].bloomSet(blockAddr);

  auto lastAddrEntryC = lastAddrTable[cpu].get(ipTag);
  if (lastAddrEntryC.has_value()) {
    lastAddrEntryC.value()->lastAddr = blockAddr;
  }

  // Usful prefetch, we have to train
  if (useful_prefetch && cache_hit) {

    uint64_t set =
        addr.slice(champsim::dynamic_extent{intern_->OFFSET_BITS,
                                            champsim::lg2(intern_->NUM_SET)})
            .to<long>();
    auto entry =
        std::find(std::begin(shadow_cache[cpu][set]),
                  std::end(shadow_cache[cpu][set]), CacheEntry(blockAddr));
    assert(entry != std::end(shadow_cache[cpu][set]));

    // This is late prefetch
    if (!entry->pf)
      return metadata_in;

    uint64_t latency = entry->lat;
    entry->pf = false;
    if (latency < cycle)
      train(blockAddr, cycle - latency, ipTag, addrTag, cycle, this, cpu);
    // trigger(this, ipTag, cycle, cpu);

    auto deltaEntry = deltaTable[cpu].get(ipTag);
    if (deltaEntry.has_value())
      deltaEntry.value()->timely++;
    stats.timely++;
  }

  trigger(this, ipTag, cycle, cpu);

  if (cache_hit) {
    return metadata_in;
  }

  // Check if this is a colascending
  if (trackLatency[cpu].count(blockAddr) != 0) {
    // V3_1
    if (trackPF[cpu].count(blockAddr)) {
      addrToIp[cpu][blockAddr] = ipTag;
      trackPF[cpu].erase(blockAddr);
    }
    // END V3_1
    return metadata_in; // This is a coalescending miss
  }

  // Track the latency of the miss
  trackLatency[cpu][blockAddr] = cycle;
  addrToIp[cpu][blockAddr] = ipTag;

  // Add previous ip if is not in the history
  instructionHistory[cpu].entries.emplace_back(ipTag, cycle);
  if (std::size(instructionHistory[cpu].entries) > sizeInstructionHistory)
    instructionHistory[cpu].entries.pop_front();

  assert(std::size(instructionHistory[cpu].entries) <= sizeInstructionHistory);

  return metadata_in;
}

uint32_t edp::prefetcher_cache_fill(champsim::address addr, long set, long way,
                                    uint8_t prefetch,
                                    champsim::address evicted_addr,
                                    uint32_t metadata_in) {
  uint64_t address = addr.to<uint64_t>();
  uint64_t evicted_address = evicted_addr.to<uint64_t>();
  auto current_cycle =
      intern_->current_time.time_since_epoch() / intern_->clock_period;
  int32_t cpu = intern_->cpu;

  if (metadata_in == 0xbeba) {
    num_issues[cpu] = 1;
  } else if (metadata_in == 0xcafe && num_issues[cpu] != 0) {
    num_issues[cpu] = 4;
  } else if (metadata_in == 0xbbbb && num_issues[cpu] != 0) {
    num_issues[cpu] = 16;
  }

  uint64_t block_evicted_address = (evicted_address >> LOG2_BLOCK_SIZE);
  bitMap[cpu].bloomClear(block_evicted_address);

  uint64_t blockAddr = (address >> LOG2_BLOCK_SIZE);
  // Calculate latency
  uint64_t cycle = current_cycle & cycleMask;
  uint64_t latency = 0;
  if (trackLatency[cpu][blockAddr] < cycle)
    latency = cycle - trackLatency[cpu][blockAddr];
  uint64_t initCycle = trackLatency[cpu][blockAddr] - latency;
  // Get the ipTag
  uint64_t addrTag = blockAddr;
  uint64_t ipTag = addrToIp[cpu][blockAddr];

  // Remove block
  trackLatency[cpu].erase(blockAddr);
  prefetch = trackPF[cpu].count(blockAddr) != 0;

  if (prefetch)
    trackPF[cpu].erase(blockAddr);
  if (deltaTable[cpu].get(ipTag).has_value() && prefetch) {
    shadow_cache[cpu][set][way] =
        CacheEntry(blockAddr, latency, prefetch, ipTag);
  } else {
    shadow_cache[cpu][set][way] = CacheEntry(blockAddr, latency, prefetch);
  }

  if (prefetch)
    return metadata_in;

  if (latency != 0 && !prefetch)
    train(blockAddr, initCycle, ipTag, addrTag, cycle, this, cpu);

  return metadata_in;
}

void edp::prefetcher_final_stats() {
  std::string _NAME = "Entangling Berti";
  std::cout << std::endl;
  std::cout << std::endl;

  std::cout << _NAME << " Not_Found_Trigger: " << stats.not_found_trigger
            << " Found_ | 7Trigger: " << stats.found_trigger << std::endl;

  std::cout << _NAME << " Avg. Num Issues Per IP: "
            << static_cast<double>(stats.total_issues_per_IP) /
                   static_cast<double>(stats.num_issues_per_IP)
            << std::endl;

  std::cout << _NAME << " Avg. Num Entangling: "
            << static_cast<double>(stats.entangling_total) /
                   static_cast<double>(stats.entangling_num)
            << std::endl;
  std::cout << _NAME << " Entangling Add: " << stats.entangling_add
            << " Entangling Full: " << stats.entangling_full
            << " Entangling Merge: " << stats.entangling_merge << std::endl;

  std::cout << _NAME << " Avg. Issues per trigger: "
            << static_cast<double>(stats.issues_per_trigger_total) /
                   static_cast<double>(stats.issues_per_trigger_num)
            << std::endl;

  std::cout << _NAME << " To L1D: " << stats.to_L1D
            << " To L2 Bc High MSHR Use: " << stats.to_L2C_bc_high_use
            << " To L2 Bc Low MSHR Use: " << stats.to_L2C_bc_low_use
            << " To L2: " << stats.to_L2C << std::endl;

  std::cout << _NAME << " Skip_AVG: "
            << static_cast<double>(stats.total_issues_per_IP) /
                   static_cast<double>(stats.num_issues_per_IP)
            << std::endl;

  for (const auto &[key, value] : stats.histogram_strides) {
    std::cout << key << ": " << value << " | ";
  }

  std::cout << std::endl;
}

void edp::prefetcher_cycle_operate() {
  int occupancy =
      intern_->get_pq_occupancy()[0] + intern_->get_pq_occupancy()[1];
  int rq_occupancy =
      intern_->get_rq_occupancy()[0] + intern_->get_rq_occupancy()[1];
  auto current_cycle =
      intern_->current_time.time_since_epoch() / intern_->clock_period;
  auto cpu = intern_->cpu;

  if (get_dram_bw() > 15) {
    num_issues[cpu] = 0;
    return;
  } else if (get_dram_bw() > 10) {
    num_issues[cpu] = 5;
    return;
  } else if (num_issues[cpu] == 0 || num_issues[cpu] == 5) {
    num_issues[cpu] = 16;
  }

  // Should we fill this level
  bool fillThisLevel = intern_->get_mshr_occupancy_ratio() < mshr_limit;

  int insert = rq_occupancy;
  if ((occupancy + rq_occupancy) < (int)intern_->MAX_TAG) {
    // Iterate over virtual pq and extract them
    for (auto &ref_vpq : {
             std::make_tuple(std::ref(edp_vpq_to_l1[cpu]),
                             std::ref(edp_vpq_to_l1_set[cpu])),
             std::make_tuple(std::ref(edp_vpq_to_l2[cpu]),
                             std::ref(edp_vpq_to_l2_set[cpu])),
         }) {
      auto &vpq = std::get<0>(ref_vpq);
      auto &set = std::get<1>(ref_vpq);

      while (insert < (int)intern_->MAX_TAG) {

        if (std::size(vpq) == 0)
          break;

        if (bitMap[cpu].shouldPf(vpq.front() >> LOG2_BLOCK_SIZE)) {
          if (!prefetch_line(vpq.front(), fillThisLevel, 0)) {
            set.erase(vpq.front());
            vpq.pop();
            break;
          }

          // Only add if the block will return
          uint64_t pfBlock = vpq.front() >> LOG2_BLOCK_SIZE;
          if (trackLatency[cpu].count(pfBlock) == 0 && fillThisLevel) {
            int set =
                pfBlock & champsim::bitmask(champsim::lg2(intern_->NUM_SET));
            auto entry_ = std::find(std::begin(shadow_cache[cpu][set]),
                                    std::end(shadow_cache[cpu][set]),
                                    CacheEntry(pfBlock));
            if (entry_ == std::end(shadow_cache[cpu][set])) {
              trackLatency[cpu][pfBlock] = current_cycle;
              trackPF[cpu].insert(pfBlock);
            }
          }

          // No prefetch
          bitMap[cpu].bloomSet(vpq.front() >> LOG2_BLOCK_SIZE);
          insert++;
        }

        set.erase(vpq.front() >> LOG2_BLOCK_SIZE);
        vpq.pop();
      }
      // Never fill this level for L2 VPQ
      fillThisLevel = false;
    }
  }
}
