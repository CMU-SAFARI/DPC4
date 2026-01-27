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
#include "gberti.h"

#define DEBUG(args...) //printf(args)
#define LOG(args...) //printf(args)

void gberti::prefetcher_initialize()
{
  init_history_table(histories);
  init_delta_table(deltas);
}

/**
 * On cache req:
 *  Record in history table (do this LAST to avoid generating 0 deltas)
 *  If a prefetch hit, search for timely deltas
 *  Search for global deltas (for spatial patterns)
 *  Send prefetches
 *  */
uint32_t gberti::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in)
{
  DEBUG("Prefetcher cache operate: addr= %lx ip=%lx\n", addr.to<uint64_t>(), ip.to<uint64_t>());
  LOG("Access %lx by ip %lx: CACHE %s\n", addr.to<uint64_t>(), ip.to<uint64_t>(), (cache_hit) ? "HIT" : "MISS");
  uint64_t cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  if (cache_hit == 0)
  {
    miss_timestamps[LINE_ADDR(addr.to<uint64_t>())] = cycle;
    miss_ips[addr.to<uint64_t>()] = ip.to<uint64_t>();
  }
  if (cache_hit && fetch_latencies[LINE_ADDR(addr.to<uint64_t>())] > 0)
  {
    search_for_deltas(addr.to<uint64_t>(), ip.to<uint64_t>(), cycle-fetch_latencies[LINE_ADDR(addr.to<uint64_t>())]);
    fetch_latencies[LINE_ADDR(addr.to<uint64_t>())] = 0;
  }
  send_prefetches(addr.to<uint64_t>(), ip.to<uint64_t>(), cycle);
  search_for_global_deltas(addr.to<uint64_t>(), ip.to<uint64_t>());
  record_access(addr.to<uint64_t>(), ip.to<uint64_t>(), cycle);
  return 0;
}

/**
 * On cache fill:
 *  Make sure to throw out the evicted addresses fetch latency, since that's
 *    stored with the cache block, as well as the miss timestamp stored in the
 *    MSHR
 *  Calculate and save the fetch latency for this request
 *  If it's a demand access fill, search for timely deltas
 *  */
uint32_t gberti::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  DEBUG("Prefetcher cache fill: addr= %lx evicted=%lx\n", addr.to<uint64_t>(), evicted_addr.to<uint64_t>());
  LOG("Fill %lx evicting %lx %s\n", addr.to<uint64_t>(), evicted_addr.to<uint64_t>(), fetch_latencies[LINE_ADDR(evicted_addr.to<uint64_t>())] > 0 ? "USELESS" : "");
  uint64_t cycle = intern_->current_time.time_since_epoch() / intern_->clock_period;
  uint64_t latency = cycle - miss_timestamps[LINE_ADDR(addr.to<uint64_t>())];
  fetch_latencies.erase(LINE_ADDR(evicted_addr.to<uint64_t>()));
  miss_timestamps.erase(LINE_ADDR(addr.to<uint64_t>()));
  if (latency > MAX_LATENCY) latency = 0;
  if (!prefetch && latency > 0)
  {
    search_for_deltas(addr.to<uint64_t>(), miss_ips[addr.to<uint64_t>()], cycle-latency);
    miss_ips.erase(addr.to<uint64_t>());
  }
  else
  {
    fetch_latencies[LINE_ADDR(addr.to<uint64_t>())] = latency;
  }
  return 0;
}

/**
 * insert with fifo replacement
 * */
void gberti::record_access(uint64_t addr, uint64_t ip, uint64_t cycle)
{
  DEBUG("record_access: addr= %lx ip=%lx\n", addr, ip);
  uint32_t set = ip % HISTORY_TABLE_SETS;
  uint64_t tag = ip & HISTORY_TABLE_TAG_MASK;
  uint32_t way = histories.fifo[set];
  uint32_t prev_way = (way + HISTORY_TABLE_WAYS - 1) % HISTORY_TABLE_WAYS;
  history_table_entry& prev_entry = histories.entries[set][prev_way];
  if (prev_entry.ip_tag == tag && prev_entry.addr == (addr & HISTORY_TABLE_ADDR_MASK))
  {
    DEBUG("\t Duplicates last access, dropping\n");
    return;
  }
  histories.fifo[set] = (way + 1) % HISTORY_TABLE_WAYS;
  history_table_entry& entry = histories.entries[set][way];
  entry.addr = addr & HISTORY_TABLE_ADDR_MASK;
  entry.ip_tag = tag;
  entry.timestamp = cycle;
}

size_t insert_delta(int64_t delta, delta_table_entry& entry)
{
  int repl = -1;
  int field = 0;
  for (; field < NUM_DELTAS; field++)
  {
    if (delta == entry.delta[field])
    {
      entry.coverage[field] += entry.coverage_increment[field];
      entry.coverage_increment[field] = 0;
      DEBUG(" Found in delta #%d\n", field);
      break;
    }
    if ((entry.status[field] == NO_PREF || entry.status[field] == REPLACE) &&
        repl == -1 && entry.coverage[field] == 0)
    {
      repl = field;
    }
  }
  if (field == NUM_DELTAS && repl != -1)
  {
    entry.delta[repl] = delta;
    entry.coverage[repl] = 1;
    entry.coverage_increment[repl] = 0;
    entry.status[repl] = NO_PREF;
    DEBUG(" Replacing delta #%d\n", repl);
    return repl;
  }
  return field;
}

size_t find_delta_entry(uint64_t ip_tag, delta_table& deltas)
{
  size_t delta_entry = 0;
  while (delta_entry < DELTA_TABLE_SIZE)
  {
    if ((ip_tag & DELTA_TABLE_TAG_MASK) == deltas.entries[delta_entry].ip_tag) break;
    delta_entry++;
  }
  return delta_entry;
}

/**
 * When searching for deltas:
 *  Look from youngest entry to oldest in ips set of history table, stopping if we have
 *    found MAX entries with matching tags and timely deltas
 *  A delta is timely if its timestamp + the latency of the access for this
 *    address < the current core cycle.
 *  Once deltas are found, insert into delta table:
 *  First, look for an entry with matching ip_tag, if one is not found then
 *    replace the oldest entry
 *  Increment the entry's global counter
 *  For each delta, search for a matching delta and increment its counter
 *    If one is not found, then replace the lowest coverage delta with status
 *    NO or REPLACE, if such a delta exists
 *  If global counter == 16, then update all states (at most 12 set to prefetch) and reset counters
 *  */
void gberti::search_for_deltas(uint64_t addr, uint64_t ip, uint64_t max_timely_timestamp)
{
  DEBUG("Search for deltas: addr= %lx ip=%lx\n", addr, ip);
  if (!ip) return;
  uint32_t set = ip % HISTORY_TABLE_SETS;
  uint64_t tag = ip & HISTORY_TABLE_TAG_MASK;
  std::vector<int64_t> timely_deltas;
  bool has_local_delta = false;
  for (int way = histories.fifo[set] + HISTORY_TABLE_WAYS-1; way >= histories.fifo[set] && timely_deltas.size() <= MAX_DELTAS_PER_SEARCH; way--)
  {
    history_table_entry& entry = histories.entries[set][way % HISTORY_TABLE_WAYS];
    if (entry.timestamp <= max_timely_timestamp)
    {
      int64_t delta = (int64_t)(addr & HISTORY_TABLE_ADDR_MASK) - (int64_t)entry.addr;
      if (valid_delta(delta)) timely_deltas.push_back(delta);
    }
  }
  DEBUG("\tFound deltas:");
  for (int64_t d : timely_deltas)
  {
    DEBUG(" %ld", d);
  }
  size_t delta_entry = find_delta_entry(ip & DELTA_TABLE_TAG_MASK, deltas);
  if (delta_entry == DELTA_TABLE_SIZE)
  {   //no entry found for this ip, replace the oldest
      delta_entry = deltas.fifo;
      deltas.fifo = (delta_entry+1) % DELTA_TABLE_SIZE;
      init_delta_entry(deltas.entries[delta_entry]);
      deltas.entries[delta_entry].ip_tag = (ip & DELTA_TABLE_TAG_MASK);
  }
  delta_table_entry& entry = deltas.entries[delta_entry];
  DEBUG("\nDelta entry %d:", delta_entry);
  entry.ctr++;
  for (int64_t delta: timely_deltas)
  {
    size_t field = insert_delta(delta, entry);
    if (field != NUM_DELTAS && (entry.status[field] != NO_PREF || entry.coverage[field] >= LOW_CONFIDENCE))
    {
      has_local_delta = true;
    }
  }
  entry.has_local_delta = has_local_delta;
  for (int i = 0; i < NUM_DELTAS; i++)
  {
    entry.coverage_increment[i] = 1;
    if (entry.coverage[i] > 0)
    {
      DEBUG("(%ld:%d)", entry.delta[i], entry.coverage[i]);
    }
  }
  DEBUG(" /%d\n", entry.ctr);
  if (entry.ctr == 16)
  {
    entry.ctr = 0;
    uint32_t pref_count = 0;
    for (int i = 0; i < NUM_DELTAS; i++)
    {
      if (pref_count < MAX_PREFS){ //Ended up just prefetching everything to L1 to avoid repeat prefetches
        if (entry.coverage[i] >= HIGH_CONFIDENCE)
        {
          DEBUG("\tWill prefetch delta %ld to L1\n", entry.delta[i]);
          entry.status[i] = L1_PREF;
        }
        else if (entry.coverage[i] >= MED_CONFIDENCE)
        {
          DEBUG("\tWill prefetch delta %ld to L2\n", entry.delta[i]);
          entry.status[i] = L2_PREF;
        }
        else if (entry.coverage[i] >= LOW_CONFIDENCE)
        {
          DEBUG("\tWill prefetch delta %ld to L2 or replace\n", entry.delta[i]);
          entry.status[i] = REPLACE;
        }
        else entry.status[i] = NO_PREF;
      }
      else entry.status[i] = NO_PREF;
      entry.coverage[i] = 0;
      if (entry.status[i] != NO_PREF) pref_count++;
    }
  }
}

  //Also try to insert a delta for the last instance of every other insn, even
  //if not timely (bc if this is helpful, the other insn will miss)
  //
  //DO NOT DO THIS IF THERE MIGHT BE A LOCAL DELTA
  //  searching for global deltas when a local delta exists will learn to pull
  //  old data back into the cache
  //
  // The above was likely due to doing the global delta search at fill time
  // instead of demand time, still makes sense to disable for locals though bc
  // its a waste of space
void gberti::search_for_global_deltas(uint64_t addr, uint64_t ip)
{
  size_t delta_entry = find_delta_entry(ip & DELTA_TABLE_TAG_MASK, deltas);
  if (delta_entry == DELTA_TABLE_SIZE || !deltas.entries[delta_entry].has_local_delta)
  {
    DEBUG("Searching for global deltas\n");
    for (int set = 0; set < HISTORY_TABLE_SETS; set++)
    {
      int way = (histories.fifo[set] + HISTORY_TABLE_WAYS - 1) % HISTORY_TABLE_WAYS;
      if (histories.entries[set][way].ip_tag == (ip & HISTORY_TABLE_TAG_MASK)) continue;
      delta_entry = find_delta_entry(histories.entries[set][way].ip_tag | set, deltas);
      if (delta_entry == DELTA_TABLE_SIZE) continue;
      if (deltas.entries[delta_entry].has_local_delta) continue;
      int64_t d = (int64_t)(addr & HISTORY_TABLE_ADDR_MASK) - (int64_t)histories.entries[set][way].addr;
      if (!valid_delta(d)) continue;
      DEBUG("Inserting delta %ld into delta entry %d\n", d, delta_entry);
      insert_delta(d, deltas.entries[delta_entry]);
    }
  }
}

/**
 * To send prefetches:
 * search the delta table for a matching ip
 * For each saved delta, if sufficiently confident, send a prefetch.
 * We are sufficiently confident if either the status is L1/L2/REPLACE,
 * If prefetching to the L1, make sure to record the current cycle to calc
 *  latency
 *
 *  Got rid of the fast training from Berti, since global deltas could hit
 *  the collection requirement instantly
 *  */
void gberti::send_prefetches(uint64_t addr, uint64_t ip, uint64_t cycle)
{
  DEBUG("Send Prefetches: addr= %lx ip=%lx\n", addr, ip);
  uint64_t tag = ip & DELTA_TABLE_TAG_MASK;
  std::priority_queue<std::tuple<uint64_t, uint64_t>> prefetches;
  for (int entry = 0; entry < DELTA_TABLE_SIZE; entry++)
  {
    if (tag == deltas.entries[entry].ip_tag)
    {
      for (int delta = 0; delta < NUM_DELTAS; delta++)
      {
        int64_t d = deltas.entries[entry].delta[delta];
        if (d == 0) continue; //sanity check in case of overflow
        if (d > 0 && addr + d <= addr) continue;
        if (d < 0 && addr + d >= addr) continue;
        uint64_t target_addr = addr + d;
        if (miss_timestamps.count(LINE_ADDR(target_addr)) || fetch_latencies.count(LINE_ADDR(target_addr))) continue;
        if (deltas.entries[entry].status[delta] != NO_PREF)
        {
          prefetches.push({deltas.entries[entry].coverage[delta], target_addr});
          //miss_timestamps[LINE_ADDR(target_addr)] = cycle;
          //intern_->prefetch_line(champsim::address{target_addr}, true, 0);
          //LOG("Prefetch %lx to L1\n", target_addr);
        }
      }
      std::size_t pq_size = intern_->get_pq_size()[0];
      std::size_t pq_occupancy = intern_->get_pq_occupancy()[0];
      pq_size = std::min(pq_size, pq_occupancy + MAX_DELTAS_PER_SEARCH);
      while (pq_occupancy < pq_size && prefetches.size() > 0)
      {
        uint64_t target_addr = std::get<1>(prefetches.top());
        prefetches.pop();
        if (miss_timestamps.count(LINE_ADDR(target_addr)) || fetch_latencies.count(LINE_ADDR(target_addr))) continue;
        miss_timestamps[LINE_ADDR(target_addr)] = cycle;
        intern_->prefetch_line(champsim::address{target_addr}, true, 0);
        LOG("Prefetch %lx to L1\n", target_addr);
        pq_occupancy++;
      }
      return;
    }
  }
}

bool gberti::valid_delta(int64_t delta)
{
  return delta < MAX_DELTA && delta >= -MAX_DELTA;
}

void gberti::init_delta_table(delta_table& table)
{
  table.fifo = 0;
  for (int i = 0; i < DELTA_TABLE_SIZE; i++)
  {
    init_delta_entry(table.entries[i]);
  }
}

void gberti::init_delta_entry(delta_table_entry& entry)
{
  entry.ip_tag = 0;
  entry.ctr = 0;
  entry.has_local_delta = false;
  for (int i = 0; i < NUM_DELTAS; i++)
  {
    entry.delta[i] = 0;
    entry.coverage[i] = 0;
    entry.status[i] = NO_PREF;
    entry.coverage_increment[i] = 1;
  }
}

void gberti::init_history_entry(history_table_entry& entry)
{
  entry.addr = 0;
  entry.timestamp = 0;
}

void gberti::init_history_table(history_table& table)
{
  for (int set = 0; set < HISTORY_TABLE_SETS; set++)
  {
    table.fifo[set] = 0;
    for (int way = 0; way < HISTORY_TABLE_WAYS; way++)
    {
      init_history_entry(table.entries[set][way]);
    }
  }
}
