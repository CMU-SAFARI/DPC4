#include "BertiGo.h"
#include <unordered_set>
#include "cache.h"

// #44 Pushing the limits of the Berti prefetcher

/******************************************************************************/
/*                      Latency table functions                               */
/******************************************************************************/
uint8_t BertiGo::LatencyTable::add(uint64_t addr, uint64_t tag, bool pf, uint64_t cycle)
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

uint64_t BertiGo::LatencyTable::del(uint64_t addr)
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

uint64_t BertiGo::LatencyTable::get(uint64_t addr)
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

uint64_t BertiGo::LatencyTable::get_tag(uint64_t addr)
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
bool BertiGo::ShadowCache::add(uint32_t set, uint32_t way, uint64_t addr, bool pf, uint64_t lat)
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

  scache[set][way].addr = addr;
  scache[set][way].pf = pf;
  scache[set][way].lat = lat;
  return scache[set][way].pf;
}

bool BertiGo::ShadowCache::get(uint64_t addr)
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

void BertiGo::ShadowCache::set_pf(uint64_t addr, bool pf)
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

bool BertiGo::ShadowCache::is_pf(uint64_t addr)
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

uint64_t BertiGo::ShadowCache::get_latency(uint64_t addr)
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
void BertiGo::HistoryTable::add(uint64_t tag, uint64_t addr, uint64_t cycle)
{
  /*
   * Save the new information into the history table
   *
   * Parameters:
   *  - tag: PC tag
   *  - addr: addr access
   */
  uint16_t set = tag % HISTORY_TABLE_SETS;
  // If the latest entry is the same, we do not add it
  if (history_pointers[set] == &historyt[set][ways - 1]) {
    if (historyt[set][0].addr == (addr & ADDR_MASK))
      return;
  } else if ((history_pointers[set] - 1)->addr == (addr & ADDR_MASK))
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

uint16_t BertiGo::HistoryTable::get_aux(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle)
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
  uint16_t set = tag % HISTORY_TABLE_SETS;

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

uint16_t BertiGo::HistoryTable::get(uint32_t latency, uint64_t tag, uint64_t act_addr, uint64_t* tags, uint64_t* addr, uint64_t cycle)
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
void BertiGo::InnerBerti::increase_conf_tag(uint64_t tag)
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

    // Max confidence achieve
    for (auto& i : bertit[tag]->deltas) {
      // Set bits to prefetch level
      if (i.conf > CONFIDENCE_L1)
        i.rpl = BERTI_L1;
      else if (i.conf > CONFIDENCE_L2)
        i.rpl = BERTI_L2;
      else if (i.conf > CONFIDENCE_L2R)
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

void BertiGo::InnerBerti::add(uint64_t tag, int64_t delta)
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

  auto add_delta = [](auto delta, auto entry) {
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

uint8_t BertiGo::InnerBerti::get(uint64_t tag, std::vector<delta_t>& res)
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
    // no_found_berti++;
    return 0;
  }
  // found_berti++;

  if constexpr (champsim::debug_print)
    std::cout << std::endl;

  // We found the tag
  berti* entry = bertit[tag];

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

void BertiGo::InnerBerti::find_and_update(uint64_t latency, uint64_t tag, uint64_t cycle, uint64_t line_addr)
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

    if ((std::abs(stride) < (1 << DELTA_MASK)))
      add(tags[i], stride);
  }
}

bool BertiGo::InnerBerti::compare_rpl(delta_t a, delta_t b)
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

bool BertiGo::InnerBerti::compare_greater_delta(delta_t a, delta_t b)
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

uint64_t BertiGo::InnerBerti::ip_hash(uint64_t ip)
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
#ifdef FOLD_HASH
  uint64_t hash = 0;
  while (ip) {
    hash ^= (ip & IP_MASK);
    ip >>= SIZE_IP_MASK;
  }
  ip = hash;
#endif
  return ip; // No IP hash
}

/******************************************************************************/
/*                        Cache Functions                                     */
/******************************************************************************/
void BertiGo::prefetcher_initialize()
{
  pf_to_l1 = 0;
  pf_to_l2 = 0;
  pf_to_l2_bc_mshr = 0;
  cant_track_latency = 0;
  cross_page = 0;
  no_cross_page = 0;
  no_found_berti = 0;
  found_berti = 0;
  average_issued = 0;
  average_num = 0;
  filtered = 0;
  bloom_issued = 0;
  // Calculate latency table size
  uint64_t latency_table_size = intern_->get_mshr_size();
  for (auto const& i : intern_->get_rq_size())
    latency_table_size += i;
  for (auto const& i : intern_->get_wq_size())
    latency_table_size += i;
  for (auto const& i : intern_->get_pq_size())
    latency_table_size += i;

  // New structures
  berti = new InnerBerti(BERTI_TABLE_DELTA_SIZE, latency_table_size, intern_->NUM_SET, intern_->NUM_WAY);

  std::cout << "BertiGo Prefetcher" << std::endl;

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
  std::cout << "BertiGo HASH ENTANGLING" << std::endl;
#endif
#ifdef FOLD_HASH
  std::cout << "BERTI HASH FOLD" << std::endl;
#endif
  std::cout << "BertiGo IP MASK " << std::hex << IP_MASK << std::dec << std::endl;
  std::cout << "BertiGo L1D Conf: " << std::dec << CONFIDENCE_L1 << " L2C: " << CONFIDENCE_L2 << " L2R: " << CONFIDENCE_L2R << std::endl;

  std::cout << "BertiGo Parameters" << std::endl;
  // Sizes
  std::cout << "  SIZES: "
            << "BERTI_TABLE_SIZE=" << BERTI_TABLE_SIZE
            << " BERTI_TABLE_DELTA_SIZE=" << BERTI_TABLE_DELTA_SIZE
            << " PC_PATH_HISTORY_SIZE=" << PC_PATH_HISTORY_SIZE
            << std::endl;
  // History table
  std::cout << "  HISTORY: "
            << "SETS=" << HISTORY_TABLE_SETS
            << " WAYS=" << HISTORY_TABLE_WAYS
            << std::endl;
  // Masks
  std::cout << std::hex;
  std::cout << "  MASKS: "
            << "SIZE_IP_MASK=" << std::dec << SIZE_IP_MASK
            << " IP_MASK=0x" << std::hex << IP_MASK
            << " TIME_MASK=0x" << TIME_MASK
            << " LAT_MASK=0x" << LAT_MASK
            << " ADDR_MASK=0x" << ADDR_MASK
            << " TABLE_SET_MASK=0x" << TABLE_SET_MASK
            << std::dec
            << " DELTA_MASK=" << DELTA_MASK
            << std::endl;
  // Confidence
  std::cout << "  CONFIDENCE: "
            << "MAX=" << CONFIDENCE_MAX
            << " INC=" << CONFIDENCE_INC
            << " INIT=" << CONFIDENCE_INIT
            << " L1=" << CONFIDENCE_L1
            << " L2=" << CONFIDENCE_L2
            << " L2R=" << CONFIDENCE_L2R
            << " MIDDLE_L1=" << CONFIDENCE_MIDDLE_L1
            << " MIDDLE_L2=" << CONFIDENCE_MIDDLE_L2
            << " LAUNCH_MIDDLE_CONF=" << LAUNCH_MIDDLE_CONF
            << std::endl;
  // Limits
  std::cout << "  LIMITS: "
            << " MSHR_LIMIT=" << MSHR_LIMIT
            << std::endl;
}

uint32_t BertiGo::prefetcher_cache_operate(champsim::address address, champsim::address ip_addr, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                           uint32_t metadata_in)
{
  uint64_t addr = address.to<uint64_t>();
  uint64_t ip = ip_addr.to<uint64_t>();
  auto current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  size_t cpu = 0;

  // We select the structures for every cpu
  auto latencyt = berti->latencyt;
  auto scache = berti->scache;
  auto historyt = berti->historyt;

  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE); // Line addr

  if (line_addr == 0)
    return metadata_in;

  filter.markAccessed(addr);

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI] operate";
    std::cout << " ip: " << std::hex << ip;
    std::cout << " full_address: " << addr;
    std::cout << " line_address: " << line_addr << std::dec << std::endl;
  }

  uint64_t ip_hash = berti->ip_hash(ip) & IP_MASK;
  
  uint64_t pc_path_raw = 0;
  for (int i = PC_PATH_HISTORY_SIZE - 1; i >= 0; i--) {
    pc_path_raw = (pc_path_raw << 3) ^ (last_pcs[i] & 0xFFFF);
  }
  pc_path_raw = (pc_path_raw << 3) ^ (ip & 0xFFFF);
  uint64_t pc_path_hash = berti->ip_hash(pc_path_raw) & IP_MASK;

  if (!cache_hit) // This is a miss
  {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache miss" << std::endl;

    latencyt->add(line_addr, ip_hash, false, current_cycle);
    historyt->add(ip_hash, line_addr, current_cycle);
    historyt->add(pc_path_hash, line_addr, current_cycle);    // Also train PC_Path signature
  } else if (cache_hit && scache->is_pf(line_addr))          // Hit bc prefetch
  {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache hit because of pf" << std::endl;

    scache->set_pf(line_addr, false);

    uint64_t latency = scache->get_latency(line_addr); // Get latency

    if (latency > LAT_MASK)
      latency = 0;

    berti->find_and_update(latency, ip_hash, current_cycle & TIME_MASK, line_addr);
    historyt->add(ip_hash, line_addr, current_cycle & TIME_MASK);    
    berti->find_and_update(latency, pc_path_hash, current_cycle & TIME_MASK, line_addr);
    historyt->add(pc_path_hash, line_addr, current_cycle & TIME_MASK);
  } else {
    if constexpr (champsim::debug_print)
      std::cout << "[BERTI] operate cache hit" << std::endl;
  }

  std::vector<delta_t> deltas;
  std::unordered_set<int64_t> seen;
  
  auto add_deltas = [&](uint64_t hash) {
    std::vector<delta_t> tmp(BERTI_TABLE_DELTA_SIZE);
    berti->get(hash, tmp);
    for (auto& d : tmp) {
      if (d.delta != 0 && d.rpl != BERTI_R && seen.find(d.delta) == seen.end()) {
        deltas.push_back(d);
        seen.insert(d.delta);
      }
    }
  };
  
  add_deltas(ip_hash);
  add_deltas(pc_path_hash);

  bool first_issue = true;
  for (auto i : deltas) {
    uint64_t p_addr = (line_addr + i.delta) << LOG2_BLOCK_SIZE;
    uint64_t p_b_addr = (p_addr >> LOG2_BLOCK_SIZE);

    if (latencyt->get(p_b_addr))
      continue;
    if (i.rpl == BERTI_R)
      continue;
    if (p_addr == 0)
      continue;

    if ((p_addr >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE)) {
      cross_page++;
#ifdef NO_CROSS_PAGE
      continue;
#endif
    } else
      no_cross_page++;

    // Filter redundant prefetches using Bloom filter
    if (filter.shouldFilter(p_addr)) {
      ++filtered;
      continue;
    }

    float mshr_load = intern_->get_mshr_occupancy_ratio() * 100;

    bool fill_this_level = (i.rpl == BERTI_L1) && (mshr_load < MSHR_LIMIT);

    if (i.rpl == BERTI_L1 && mshr_load >= MSHR_LIMIT)
      pf_to_l2_bc_mshr++;
    if (fill_this_level)
      pf_to_l1++;
    else
      pf_to_l2++;

    if (prefetch_line(p_addr, fill_this_level, 0)) {
      ++bloom_issued;
      filter.markAccessed(p_addr);
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

  // Update PC history for PC_Path signature
  for (int i = PC_PATH_HISTORY_SIZE - 1; i > 0; i--) {
    last_pcs[i] = last_pcs[i-1];
  }
  last_pcs[0] = ip;

  return metadata_in;
}

uint32_t BertiGo::prefetcher_cache_fill(champsim::address address, long set, long way, uint8_t prefetch, champsim::address evicted_address,
                                        uint32_t metadata_in)
{
  filter.onFill(metadata_in);
  uint64_t addr = address.to<uint64_t>();
  uint64_t evicted_addr = evicted_address.to<uint64_t>();
  auto current_cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  size_t cpu = 0;

  // We select the structures for every cpu
  auto latencyt = berti->latencyt;
  auto scache = berti->scache;
  auto historyt = berti->historyt;

  uint64_t line_addr = (addr >> LOG2_BLOCK_SIZE); // Line addr
  uint64_t tag = latencyt->get_tag(line_addr);
  uint64_t cycle = latencyt->del(line_addr) & TIME_MASK;
  uint64_t latency = 0;

  if constexpr (champsim::debug_print) {
    std::cout << "[BERTI] fill addr: " << std::hex << line_addr;
    std::cout << " event_cycle: " << cycle;
    std::cout << " prefetch: " << +prefetch << std::endl;
    std::cout << " latency: " << latency << std::endl;
  }

  if (cycle != 0 && ((current_cycle & TIME_MASK) > cycle))
    latency = (current_cycle & TIME_MASK) - cycle;

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

  // Clear evicted address from bloom filter
  if (evicted_addr != 0) {
    filter.onEviction(evicted_addr);
  }

  // Add to the shadow cache
  scache->add(set, way, line_addr, prefetch, latency);

  if (latency != 0 && !prefetch) {
    berti->find_and_update(latency, tag, cycle, line_addr);
  }
  return metadata_in;
}

void BertiGo::prefetcher_cycle_operate() {}

void BertiGo::prefetcher_final_stats()
{
  std::cout << "BERTI "
            << "TO_L1: " << pf_to_l1 << " TO_L2: " << pf_to_l2;
  std::cout << " TO_L2_BC_MSHR: " << pf_to_l2_bc_mshr << std::endl;

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

  std::cout << "BLOOM FILTER";
  std::cout << " FILTERED: " << filtered;
  std::cout << " PREFETCHED: " << bloom_issued << std::endl;
}
