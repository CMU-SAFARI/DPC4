#include "sppam.h"
#include "../bingo_plus/bingo_plus.h" //added for Bingo plus feedback
#include "../berti_plus/berti_plus_parameters.h" //added for Berti feedback

#include <algorithm>

#include "cache.h"


template <typename T>
auto sppam::Sppam_Module::page_and_offset(T addr) -> std::pair<page, block_in_page>
{
  return std::pair{page{addr}, block_in_page{addr}};
}

std::pair<uint64_t,uint64_t> sppam::Sppam_Module::get_patterns(page pn, block_in_page page_offset) {
  auto region = regions.check_hit(region_type{pn,intern_->current_cycle()});

  if(region.has_value()) {
    uint64_t pos_access_pattern = 0;
    uint64_t neg_access_pattern = 0;
    auto region_forward = regions.check_hit(region_type{region->next_vpn,0});
    auto region_backward = regions.check_hit(region_type{region->prev_vpn,0});

    //construct the access map with shadow bits included from adjacent regions (if they exist)
    std::vector<bool> shadow_access_map;
    shadow_access_map.reserve(PATTERN_SIZE + region->access_map.size() + PATTERN_SIZE);
    if(region_backward.has_value()) {
      shadow_access_map.insert(shadow_access_map.end(),region_backward->access_map.end()-PATTERN_SIZE,region_backward->access_map.end());
      shadow_patterns_used_backward++;
    }
    else
      shadow_access_map.insert(shadow_access_map.end(),PATTERN_SIZE,false);
    shadow_access_map.insert(shadow_access_map.end(),region->access_map.begin(),region->access_map.end());
    if(region_forward.has_value()) {
      shadow_access_map.insert(shadow_access_map.end(),region_forward->access_map.begin(),region_forward->access_map.begin() + PATTERN_SIZE);
      shadow_patterns_used_forward++;
    }
    else
      shadow_access_map.insert(shadow_access_map.end(),PATTERN_SIZE,false);
    


    for(int i = page_offset.to<int64_t>() + 1; i <= page_offset.to<int64_t>() + PATTERN_SIZE; i++) {
      pos_access_pattern = (pos_access_pattern << 1) | (uint64_t(shadow_access_map.at(i)));
    }
    for(int i = page_offset.to<int64_t>() + (PATTERN_SIZE*2) - 1; i >= page_offset.to<int64_t>() + PATTERN_SIZE; i--) {
      neg_access_pattern = (neg_access_pattern << 1) | (uint64_t(shadow_access_map.at(i)));
    }
    regions.fill(region.value());

    /*fmt::print("Getting patterns for page: {} offset: {}\n",pn,page_offset);
    for(int i = 0; i < region->access_map.size() + PATTERN_SIZE*2; i++) {
      if(i == PATTERN_SIZE)
        fmt::print("]");
      else if(i == region->access_map.size() + PATTERN_SIZE)
        fmt::print("[");
      if(i == page_offset.to<uint64_t>() + PATTERN_SIZE)
        fmt::print("[{}]",(int)shadow_access_map.at(i));
      else
        fmt::print("{}",(int)shadow_access_map.at(i));
    }
    fmt::print("\nPos pattern: {:b} Neg Pattern: {:b}\n",pos_access_pattern,neg_access_pattern);*/
    return {pos_access_pattern, neg_access_pattern};
  } else {
    return {0,0};
  }
}

bool sppam::Sppam_Module::check_pagemap(champsim::address addr, bool prefetch)
{
  auto [pn, page_offset] = page_and_offset(addr);
  auto region = regions.check_hit(region_type{pn,intern_->current_cycle()});

  if(prefetch)
    return (region.has_value() && region->prefetch_map.at(page_offset.to<std::size_t>()));
  else
    return (region.has_value() && region->access_map.at(page_offset.to<std::size_t>()));
}

std::pair<uint64_t,bool> sppam::Sppam_Module::get_prefetch_pattern(uint64_t pattern, int order, bool negative) {

  //fmt::print("Getting pattern of size {} from {:b}\n",PATTERN_SIZE >> order, pattern);
  auto temp_pattern = pattern_type{pattern & ((1 << (PATTERN_SIZE >> order)) - 1)};
  auto demand_pattern = negative && SEPARATE_NEGATIVE_TABLES ? negative_pattern_tables.at(order).check_hit(temp_pattern) : pattern_tables.at(order).check_hit(temp_pattern);

  if(demand_pattern.has_value()) {
    //stat
    demand_pattern->occurrences++;
    //if(!use_pattern_confidence())
    //  demand_pattern->usefulness = global_usefulness_index;
    
    if(negative && SEPARATE_NEGATIVE_TABLES)
      negative_pattern_tables.at(order).fill(demand_pattern.value());
    else
      pattern_tables.at(order).fill(demand_pattern.value());
    return demand_pattern->get_prediction();
  } else {
    return {0,true};
  }
}

void sppam::Sppam_Module::increment_access_pattern(uint64_t pattern, uint64_t prediction, int order, bool negative) {
  auto temp_pattern = pattern_type{pattern & ((1 << (PATTERN_SIZE >> order)) - 1)};
  auto demand_pattern = negative && SEPARATE_NEGATIVE_TABLES ?  negative_pattern_tables.at(order).check_hit(temp_pattern) : pattern_tables.at(order).check_hit(temp_pattern);

  if(pattern == 0)
    return;

  if(demand_pattern.has_value()) {
    demand_pattern->increment_prediction(prediction);
    if(negative && SEPARATE_NEGATIVE_TABLES)
      negative_pattern_tables.at(order).fill(demand_pattern.value());
    else
      pattern_tables.at(order).fill(demand_pattern.value());
  } else {
    temp_pattern.increment_prediction(prediction);
    //bootstrap usefulness from global usefulness
    temp_pattern.usefulness = global_usefulness_index; 
    if(negative && SEPARATE_NEGATIVE_TABLES)
      negative_pattern_tables.at(order).fill(temp_pattern);
    else
      pattern_tables.at(order).fill(temp_pattern);
  }
}

void sppam::Sppam_Module::initialize(CACHE* cache) {
  intern_ = cache;

  if(NUM_CPUS > 1) {
    //shared system, prefetch degrees should be forced to 1 for fairness
    PREFETCH_DEGREES_USEFULNESS = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  }
  

  std::size_t table_sets = PATTERN_TABLE_SETS;
  for(int i = PATTERN_SIZE; i >= int(MIN_PATTERN_SIZE); i/=2) {
    pattern_tables.push_back(champsim::msl::lru_table<pattern_type,pattern_table_indexer,pattern_table_indexer>{table_sets,PATTERN_TABLE_WAYS});
    negative_pattern_tables.push_back(champsim::msl::lru_table<pattern_type,pattern_table_indexer,pattern_table_indexer>{table_sets,PATTERN_TABLE_WAYS});
    table_sets /= 2;
  }

  uint64_t total_state = get_state_bits();
  champsim::data::bytes total_bytes = champsim::data::bytes{(total_state/8) + 1};
  fmt::print("[{}] SPPAM Initialized. Total State: {}\n",intern_->NAME,champsim::data::kibibytes{total_bytes});
}



void sppam::Sppam_Module::add_to_pagemap(champsim::address addr, bool prefetch, champsim::address ip, uint64_t pf_trigger, page prev_page, bool direction, uint8_t sample_type) {
  auto [current_pn, page_offset] = page_and_offset(addr);
  auto temp_region = region_type{current_pn,intern_->current_cycle()};
  auto demand_region = regions.check_hit(temp_region);

  if(demand_region.has_value()) {
    regions_scraped++;
    //scrape regions that have been idle for too long
    if((demand_region->last_access_time + SCRAPE_IDLE_TIME < intern_->current_cycle()) && SCRAPE_ON_IDLE) {
      if(demand_region->since_last_scrape >= SCRAPE_MIN_COUNT)
        scrape_region(addr);
      demand_region->last_access_time = intern_->current_cycle();
    } else if (SCRAPE_ON_COUNT && (demand_region->since_last_scrape >= SCRAPE_ACCESS_COUNT)){
      scrape_region(addr);
    }

    if(prefetch) {
      demand_region->prefetch_map.at(page_offset.to<std::size_t>()) = true;
      demand_region->pf_trigger.at(page_offset.to<std::size_t>()) = pf_trigger;
      demand_region->sample_type.at(page_offset.to<std::size_t>()) = sample_type;
    }
    else {
      demand_region->access_map.at(page_offset.to<std::size_t>()) = true;
      demand_region->prefetch_map.at(page_offset.to<std::size_t>()) = true;
      if(demand_region->last_block < page_offset.to<uint64_t>())
        demand_region->momentum = demand_region->momentum < 7 ? demand_region->momentum + 1 : 7;
      else
        demand_region->momentum = demand_region->momentum > -8 ? demand_region->momentum - 1 : -8;
      demand_region->last_block = page_offset.to<uint64_t>();
      demand_region->since_last_scrape += 1;
      if(prev_page != page{}) {
        if(direction) {
          demand_region->prev_vpn = prev_page;
          //fmt::print("Set prev page to: {} for page: {}\n",demand_region->prev_vpn,current_pn);
        }
        else {
          demand_region->next_vpn = prev_page;
          //fmt::print("Set next page to: {} for page: {}\n",demand_region->next_vpn,current_pn);
        }
      }
    }
    regions.fill(demand_region.value());
  } else {
    if(prefetch) {
      temp_region.prefetch_map.at(page_offset.to<std::size_t>()) = true;
      temp_region.pf_trigger.at(page_offset.to<std::size_t>()) = pf_trigger;
      temp_region.sample_type.at(page_offset.to<std::size_t>()) = sample_type;
    }
    else {
      temp_region.access_map.at(page_offset.to<std::size_t>()) = true;
      temp_region.prefetch_map.at(page_offset.to<std::size_t>()) = true;
      temp_region.last_block = page_offset.to<uint64_t>();
      temp_region.since_last_scrape += 1;
      if(prev_page != page{}) {
        if(temp_region.momentum > FORWARD_MOMENTUM_MIN) {
          temp_region.prev_vpn = prev_page;
        }
        else if(temp_region.momentum < BACKWARD_MOMENTUM_MIN) {
          temp_region.next_vpn = prev_page;
        }
      }
    }
    regions.fill(temp_region);
  }
}

void sppam::Sppam_Module::add_to_llc_pagemap(champsim::address addr) {
  auto [current_pn, page_offset] = page_and_offset(addr);
  auto temp_region = region_type{current_pn,0};

  auto entry = llc_regions.check_hit(temp_region);
  if(entry.has_value()) {
    entry->access_map.at(page_offset.to<uint64_t>()) = true;
    llc_regions.fill(entry.value());
  } else {
    temp_region.access_map.at(page_offset.to<uint64_t>()) = true;
    llc_regions.fill(temp_region);
  }
}

bool sppam::Sppam_Module::check_llc_pagemap(champsim::address addr) {
  auto [current_pn, page_offset] = page_and_offset(addr);
  auto temp_region = region_type{current_pn,0};

  auto entry = llc_regions.check_hit(temp_region);
  if(entry.has_value())
    return entry->access_map.at(page_offset.to<uint64_t>());
  return false;
}

void sppam::Sppam_Module::remove_from_pagemap(champsim::address addr, bool prefetch) {
  auto [current_pn, page_offset] = page_and_offset(addr);
  auto demand_region = regions.check_hit(region_type{current_pn,intern_->current_cycle()});

  if(demand_region.has_value()) {
    if(prefetch)
      demand_region->prefetch_map.at(page_offset.to<std::size_t>()) = false;
    else
      demand_region->access_map.at(page_offset.to<std::size_t>()) = false;
    demand_region->last_access_time = intern_->current_cycle();
    regions.fill(demand_region.value());
  }
}

void sppam::Sppam_Module::remove_from_llc_pagemap(champsim::address addr) {
  auto [current_pn, page_offset] = page_and_offset(addr);
  auto temp_region = region_type{current_pn,0};

  auto entry = llc_regions.check_hit(temp_region);
  if(entry.has_value()) {
    entry->access_map.at(page_offset.to<uint64_t>()) = false;
    llc_regions.fill(entry.value());
  }
}


std::pair<sppam::Sppam_Module::page,bool> sppam::Sppam_Module::get_prev_page(uint32_t stream_id, page current_vpn) {
  //check cpt table
  stream_id = stream_id & BERTI_STREAM_ID_MASK;
  if(stream_id == 0)
    return {page{},true};

  bool direction = (stream_id & (1<<BERTI_DIRECTION_BIT)) != 0;

  auto temp_cpt = cross_page_tracker_type{stream_id,current_vpn};
  auto cpt_entry = cpt_table.check_hit(temp_cpt);

  if(cpt_entry.has_value()) {
    //found an entry
    //check if pages match
    if(cpt_entry->vpn != current_vpn) {
      auto temp_vpn = cpt_entry->vpn;
      auto temp_forward = cpt_entry->forward;
      cpt_table.invalidate(temp_cpt);
      return {temp_vpn,temp_forward};
    } else {
      return {page{},true};
    }
  } else {
    temp_cpt.forward = direction;
    cpt_table.fill(temp_cpt);
    return {page{}, true};
  }
}

uint32_t sppam::prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in)
{
  //if((type != access_type::LOAD) && engine.DEMAND_ONLY)
  //  return metadata_in;

  if(useful_prefetch) {
    engine.modify_pattern_usefulness(addr,true);
    engine.increase_usefulness_counter();
  }

  if(!cache_hit) {
    engine.add_to_llc_pagemap(addr);
  }
 // if(!cache_hit) {
 //   engine.add_to_llc_pagemap(addr);
//
//    //scrape misses
//    engine.scrape_region(addr);
//  }

  if(type == access_type::LOAD || !engine.TRAIN_DEMAND_ONLY) {
    auto [prev_page, dir] = engine.get_prev_page(metadata_in,Sppam_Module::page{addr});

    if((!cache_hit || !engine.ACCESS_MAP_MISS_ONLY))
      engine.add_to_pagemap(addr,false,ip,0,prev_page,dir);
  }



  if(type == access_type::LOAD || !engine.PREFETCH_DEMAND_ONLY)
    engine.do_prefetch(addr,ip,cache_hit,useful_prefetch,type,metadata_in);

  return metadata_in;
}

uint32_t sppam::prefetcher_cache_fill(champsim::address addr, long set, long way, bool prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  if(engine.USELESS_ON_FILL)
    engine.modify_pattern_usefulness(addr,false);
  if(prev_useless_prefetches < intern_->sim_stats.pf_useless) {
    if(!engine.USELESS_ON_FILL)
      engine.modify_pattern_usefulness(evicted_addr,false);
    engine.decrease_usefulness_counter();
    //engine.useless_timer = 0;
  } else {
    //prefetch victim into LLC
    if(engine.DO_VICTIM_PREFETCH && evicted_addr != champsim::address{})
      prefetch_line(evicted_addr,false,0);

    if(engine.SCRAPE_ON_EVICT)
      engine.scrape_region(evicted_addr);
  }
  prev_useless_prefetches = intern_->sim_stats.pf_useless;
  if(evicted_addr != champsim::address{}) {
    engine.remove_from_pagemap(evicted_addr,true);
    
    //engine.remove_from_pagemap(evicted_addr,false);
  }
  engine.track_llc_missrate(metadata_in,prefetch);
  

  return metadata_in;
}

void sppam::Sppam_Module::track_llc_missrate(uint32_t hit, bool prefetch) {
  
  if(TRACK_LLC_EVICTS)
  if((hit & bingo_plus_module::BINGO_LLC_EVICT) != 0) {
    //got evict address back
    auto evict_addr = champsim::address{champsim::block_number{hit >> 2}};
    //fmt::print("LLC says: {} was evicted\n",evict_addr);
    remove_from_llc_pagemap(evict_addr);
  }

  assert(!TRACK_LLC_MISSRATE);

  if((hit & bingo_plus_module::BINGO_PROPORTION) != 0) {
    //fmt::print("Recieved metadata: {:b} index: {}\n",hit,hit>>2);
    fairness_index = hit >> 2;
  }

}

void sppam::Sppam_Module::scrape_region(champsim::address addr) {
  //check this region for relevant patterns, log them, and then clear it out
  //fmt::print("Scraping region for Addr: {}\n",addr);
  auto [pn, page_offset] = page_and_offset(addr);
  auto region = regions.check_hit(region_type{pn,intern_->current_cycle()});
  if(region.has_value()) {
    //get forward and backward shadow regions
    auto region_forward = regions.check_hit(region_type{region->next_vpn,0});
    auto region_backward = regions.check_hit(region_type{region->prev_vpn,0});

    //construct the access map with shadow bits included from adjacent regions (if they exist)
    std::vector<bool> shadow_access_map;
    shadow_access_map.reserve(PATTERN_SIZE + region->access_map.size() + PATTERN_SIZE);
    if(region_backward.has_value()) {
      shadow_access_map.insert(shadow_access_map.end(),region_backward->access_map.end()-PATTERN_SIZE,region_backward->access_map.end());
      shadow_scrapes_used_backward++;
    }
    else
      shadow_access_map.insert(shadow_access_map.end(),PATTERN_SIZE,false);
    shadow_access_map.insert(shadow_access_map.end(),region->access_map.begin(),region->access_map.end());
    if(region_forward.has_value()) {
      shadow_access_map.insert(shadow_access_map.end(),region_forward->access_map.begin(),region_forward->access_map.begin() + PATTERN_SIZE);
      shadow_scrapes_used_forward++;
    }
    else
      shadow_access_map.insert(shadow_access_map.end(),PATTERN_SIZE,false);

    region->since_last_scrape = 0;
    uint64_t access_pattern = 0;
    uint64_t prefetched_pattern = 0;

    int start_forward = region_backward.has_value() ? 0 : PATTERN_SIZE;
    int end_forward = region_forward.has_value() ? region->access_map.size() : region->access_map.size() - PATTERN_SIZE;

    int start_backward = region_forward.has_value() ? (region->access_map.size() + PATTERN_SIZE*2 - 1) : (region->access_map.size() + PATTERN_SIZE - 1);
    int end_backward = region_backward.has_value() ? ((PATTERN_SIZE * 2) - 1) : (PATTERN_SIZE * 3) - 1;

    //fmt::print("Region scrape for page: {}\n",pn);
    //for(int i = 0; i < region->access_map.size() + PATTERN_SIZE*2; i++)
    //  fmt::print("{}",int(shadow_access_map.at(i)));
    //fmt::print("\n");
    if(region->momentum > FORWARD_MOMENTUM_MIN) {
      for(std::size_t i = start_forward; i <= end_forward; i++) {
        //if(region->access_map.at(i) == false)
        //  continue;
        access_pattern = 0;
        for(std::size_t j = i; j < i + PATTERN_SIZE*2 && j < region->access_map.size(); j++) {
          access_pattern = (access_pattern << 1) | (uint64_t(shadow_access_map.at(j)));
        }
        //grab each power of two pattern as well
        int order = 0;
        //if they line up, ignore (already useful, don't re-record what was successfully prefetched)
        for(std::size_t j = PATTERN_SIZE; j >= MIN_PATTERN_SIZE; j/=2) {
          
          uint64_t curr_predicted_pattern = (access_pattern & ((1 << j) - 1));
          uint64_t curr_access_pattern = (access_pattern >> j) & ((1 << j) - 1);
          if(curr_access_pattern == 0)
            continue;
          //if((curr_access_pattern & 1) == 0)
          //  continue; //only log patterns that were actually accessed
          //fmt::print("Logging Pattern from index {}: Access: {:b} Predicted: {:b}\n",i,curr_access_pattern, curr_predicted_pattern);
          increment_access_pattern(curr_access_pattern, curr_predicted_pattern,order, false);
          order++;
        }
      }
    }

    //do it for reverse direction as well
    if(DO_NEGATIVE && region->momentum < BACKWARD_MOMENTUM_MIN) {
      for(int64_t i = start_backward; i >= end_backward; i--) {
        //if(region->access_map.at(i) == false)
        //  continue;
        access_pattern = 0;
        for(int64_t j = i; j > i - int64_t(PATTERN_SIZE*2) && j >= (PATTERN_SIZE*2 - 1); j--) {
          access_pattern = (access_pattern << 1) | (uint64_t(shadow_access_map.at(j)));
        }
        //grab each power of two pattern as well
        int order = 0;
        for(std::size_t j = PATTERN_SIZE; j >= MIN_PATTERN_SIZE; j/=2) {
          
          uint64_t curr_predicted_pattern = access_pattern & ((1 << j) - 1);
          uint64_t curr_access_pattern = (access_pattern >> j) & ((1 << j) - 1);
          if(curr_access_pattern == 0)
            continue;
          //if((curr_access_pattern & 1) == 0)
          //  continue; //only log patterns that were actually accessed
          std::size_t final_block = i - j + 1;
          //fmt::print("Logging Reverse Pattern from index {}: Access: {:b} Predicted: {:b}\n",i,curr_access_pattern, curr_predicted_pattern);
          increment_access_pattern(curr_access_pattern, curr_predicted_pattern,order, true);
          order++;
        }
      }
    }

    //clear out region
    if(MARK_AFTER_SCRAPE)
    for(int i = 0; i < region->access_map.size(); i++) {
      if(region->access_map.at(i)) {
        if(CLEAR_AFTER_SCRAPE)
          region->access_map.at(i) = false;
        
        region->prefetch_map.at(i) = !CLEAR_FILTER_AFTER_SCRAPE;
      }
    }
    region->last_access_time = intern_->current_cycle();
    regions.fill(region.value());
  }
}

int sppam::Sppam_Module::get_momentum(champsim::address addr) {
  auto [pn, page_offset] = page_and_offset(addr);
  auto region = regions.check_hit(region_type{pn,intern_->current_cycle()});
  if(region.has_value()) {
    return region->momentum;
  } else {
    return false;
  }
}

void sppam::Sppam_Module::do_prefetch(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in) {
  champsim::block_number block_addr{addr};
  if(type != access_type::PREFETCH) {
    metadata_in = (ip.to<uint64_t>() & BERTI_IP_MASK) << BERTI_IP_ENCODING_OFFSET;
    //fmt::print("[SPPAM] Sent IP of: {}\n",ip.to<uint64_t>() & BERTI_IP_MASK);
  }

  int set = addr.to<uint64_t>() % intern_->NUM_SET;

  int free_space = std::max((int)intern_->get_mshr_size() - (int)intern_->get_mshr_occupancy() - (int)intern_->get_pq_occupancy().back(),0);
  int momentum = get_momentum(addr);
  //fmt::print("Got Patterns for Addr {}: Pos Pattern: {:b} Neg Pattern: {:b}\n",addr,pos_access_pattern,neg_access_pattern);

  int pf_issued = 0;

  int current_usefulness = 15;

  auto region = regions.check_hit(region_type{page{addr},intern_->current_cycle()});

  //drop most prefetch triggers as usefulness drops
  //if((intern_->current_cycle() % 128) < PREFETCH_DROP_CHANCE_USEFULNESS.at(global_usefulness_index))
  //  return;
  //scan region in both directions
  //fmt::print("Doing prefetch\n");

  if(use_ampm_or_sppam(set,addr)) {
    if((!in_warmup) && DO_DEBUG)
    fmt::print("Doing prefetch for address: {} momentum: {} free space: {}\n",addr,momentum,free_space);
    for (auto direction : {1, -1}) {
      //scan only the scan distance
      
      for(int s = (direction == 1 ? 0 : 1); s < (direction == 1 ? SCAN_DISTANCE_FORWARD : SCAN_DISTANCE_BACKWARD); s++) {
        current_usefulness = use_pattern_confidence() ? 15 : global_usefulness_index;
        if((!in_warmup) && DO_DEBUG)
        fmt::print("\tScan: {} Direction: {} Current usefulness: {}\n",s, direction, current_usefulness);
        //usefulness must remain above threshold to continue scan
        //if(current_usefulness < 0.4)
        //  break;
        //get negative and positive patterns, set base location for the scan
        auto pf_base_addr = champsim::block_number{addr} + (direction * s);
        auto [pos_access_pattern, neg_access_pattern] = get_patterns(page{pf_base_addr}, block_in_page{pf_base_addr});
        if((!in_warmup) && DO_DEBUG)
        fmt::print("\tGot patterns pos: {:b} neg: {:b}\n",pos_access_pattern,neg_access_pattern);
        int order = 0;
        //if we have forward momentum
        if(momentum > FORWARD_MOMENTUM_MIN) {
          //for all pattern sizes
          for(int i = PATTERN_SIZE; i >= int(MIN_PATTERN_SIZE); i/=2) {
            //get patterns
            auto [pos_prefetch_pattern, pos_valid] = get_prefetch_pattern(pos_access_pattern, order, false);
            if(pos_prefetch_pattern == 0 && !pos_valid) {
                if(USE_DEFAULT_PREDICTION && ((i>>1) < MIN_PATTERN_SIZE)) {
                  if((pos_access_pattern & DEFAULT_PATTERN) == DEFAULT_PATTERN) {
                    pos_prefetch_pattern = DEFAULT_PREDICTION;
                    pos_access_pattern = DEFAULT_PATTERN;
                    pos_valid = true;
                  }
                }
            }
            bool continue_outer = false;
            int lookaheads = 0;
            int lookahead_offset = 0;
            //lookahead loop
            prefetch_triggers++;
            while(true) {
              current_usefulness = do_4_bit_mult(do_4_bit_mult(LOOKAHEAD_CONF_FACTOR,set_prefetch_degree(pos_access_pattern, order, false, current_usefulness)),current_usefulness);
              if((!in_warmup) && DO_DEBUG)
              fmt::print("\t\tLookahead: {} Got prefetch pattern: {} from: {} current usefulness: {}\n",lookaheads,pos_prefetch_pattern,pos_access_pattern,current_usefulness);
              if((intern_->current_cycle() % 128) < PREFETCH_DROP_CHANCE_USEFULNESS.at(current_usefulness)) {
                if((global_usefulness_index < ALLOW_DROP_THRESH) && PROB_DROP_PREFETCHES) {
                  prefetches_dropped++;
                  if((!in_warmup) && DO_DEBUG)
                  fmt::print("\t\tDropped!\n");
                  break;
                }
              }
              if((intern_->current_cycle() % 128) < PREFETCH_DROP_CHANCE_FAIRNESS.at(get_fairness_factor())) {
                if(USE_FAIRNESS_DROP) {
                  prefetches_dropped++;
                  if((!in_warmup) && DO_DEBUG)
                  fmt::print("\t\tDropped!\n");
                  break;
                }
              }
              //fmt::print("Checking Pos Pattern of size {}: {:b}\n",i,pos_prefetch_pattern);
              //if no pattern found, continue to next pattern size
              if(pos_prefetch_pattern == 0 && !pos_valid) {
                continue_outer = true;
                if((!in_warmup) && DO_DEBUG)
                fmt::print("\t\tCouldn't find pattern!\n");
                break;
              }
              else if(pos_prefetch_pattern == 0) { //pattern predicted nothing, exit lookahead
                if((!in_warmup) && DO_DEBUG)
                fmt::print("\t\tPattern is 0, stopping!\n");
                break;
              }
              //prefetch according to pattern
              for(int j = 0; j < i; j++) {
                if(pf_issued >= int(current_pf_degree))
                  break;
                if((pos_prefetch_pattern & (1 << (i - 1 - j)))) {
                  if(!USE_PREFETCH_MASK || ((pos_prefetch_pattern & (1 << (i - 1 - j))) & PREFETCH_MASK.at(current_usefulness))) {

                    champsim::address pos_step_addr = champsim::address{champsim::block_number{pf_base_addr} + (j + 1) + lookahead_offset};
                    
                    if(champsim::page_number{pos_step_addr} != champsim::page_number{addr}) {
                      if(CROSS_PAGE && ((direction * s) + j + 1 + lookahead_offset < 64)) {
                        auto next_region = regions.check_hit(region_type{region->next_vpn,intern_->current_cycle()});
                        if(next_region.has_value()) {
                          //adjust base address to cross into next page
                          auto page_offset = block_in_page{pos_step_addr};
                          pos_step_addr = champsim::address{champsim::block_number{region->next_vpn} + page_offset.to<std::size_t>()};
                          page_crossings++;
                        }
                        else {
                          continue;
                        }
                      } else
                          continue; //don't prefetch out of page bounds
                    }
                    if(!check_pagemap(champsim::address{pos_step_addr},true)) {
                      if((pf_issued < free_space) || ((DO_LLC_PREFETCH && !check_llc_pagemap(pos_step_addr)) || !USE_LLC_MAP)) { //filter LLC prefetches
                        if(bool prefetch_success = intern_->prefetch_line(pos_step_addr, pf_issued < free_space, metadata_in); prefetch_success) {
                          if((!in_warmup) && DO_DEBUG)
                          fmt::print("\t\t\tPrefetched: {} LLC: {}\n",pos_step_addr, pf_issued < free_space);
                          //fmt::print("AMPM Prefetched: {} Metadata: {}\n",pos_step_addr,metadata_in);
                          //fmt::print("\tScan distance: {} direction: {}\n",s,direction);
                          //fmt::print("\t\tPrefetch issued, trigger address: {} pattern: {:b} prediction: {:b} pf_address: {} usefulness: {} level: {}\n",addr,pos_access_pattern,pos_prefetch_pattern,pos_step_addr,current_usefulness,pf_issued < free_space);
                          if(pf_issued < free_space)
                            add_to_pagemap(pos_step_addr,true,ip,pos_prefetch_pattern,page{},false,get_sample_type(set));
                          else {
                            to_llc++;
                            add_to_llc_pagemap(pos_step_addr);
                          }
                          pf_issued++;
                          //metadata_in = 0;
                          if(pf_issued >= int(current_pf_degree))
                            break;

                          prefetches_issued++;
                        }
                      } else {
                        if((!in_warmup) && DO_DEBUG)
                        fmt::print("\t\t\tWas filtered by LLC table!\n");
                        prefetches_filtered_llc++;
                      }
                    } else {
                      if((!in_warmup) && DO_DEBUG)
                      fmt::print("\t\t\tWas filtered!\n");
                      prefetches_filtered++;
                    }
                  }
                }
              }
              //lookahead check
              if(DO_LOOKAHEAD && pf_issued < int(current_pf_degree)) {
                //update pattern
                auto lookahead = get_prefetch_pattern(pos_prefetch_pattern, order,false);
                if((!in_warmup) && DO_DEBUG)
                fmt::print("\t\tLookahead access pattern: {} prefetch pattern: {}\n",pos_access_pattern,pos_prefetch_pattern);
                pos_access_pattern = pos_prefetch_pattern;
                pos_prefetch_pattern = lookahead.first;
                pos_valid = lookahead.second;
                lookaheads++;
                lookahead_offset += (i);
                if(lookaheads > int(LOOKAHEAD_DEPTH) || current_usefulness < LOOKAHEAD_CONF_CUTOFF) {
                  if((!in_warmup) && DO_DEBUG)
                  fmt::print("\t\tStopping Lookahead!\n");
                  break;
                }
                if(pos_prefetch_pattern != 0)
                  total_lookaheads++;
              } else {
                break;
              }
            }
            //should break to and not continue to smaller patterns
            if(!continue_outer)
              break; //only do highest positive pattern
            order++;
          }
        }
        
        //if momentum is negative
        current_usefulness = global_usefulness_index / 16.0;
        if(DO_NEGATIVE && (momentum < BACKWARD_MOMENTUM_MIN)) {
          order = 0;
          //for all pattern sizes
          for(int i = PATTERN_SIZE; i >= int(MIN_PATTERN_SIZE); i/=2) {
            //grab patterns
            auto [neg_prefetch_pattern, neg_valid] = get_prefetch_pattern(neg_access_pattern, order, true);
            if(neg_prefetch_pattern == 0 && !neg_valid) {
                if(USE_DEFAULT_PREDICTION && ((i>>1) < MIN_PATTERN_SIZE)) {
                  if((neg_access_pattern & DEFAULT_PATTERN) == DEFAULT_PATTERN) {
                    neg_prefetch_pattern = DEFAULT_PREDICTION;
                    neg_access_pattern = DEFAULT_PATTERN;
                    neg_valid = true;
                  }
                }
            }
            bool continue_outer = false;
            int lookaheads = 0;
            int lookahead_offset = 0;
            //lookahead loop
            while(true) {
              current_usefulness = do_4_bit_mult(do_4_bit_mult(LOOKAHEAD_CONF_FACTOR,set_prefetch_degree(neg_access_pattern, order, true, current_usefulness)),current_usefulness);
              if((intern_->current_cycle() % 128) < PREFETCH_DROP_CHANCE_USEFULNESS.at(current_usefulness)) {
                if((global_usefulness_index < ALLOW_DROP_THRESH) && PROB_DROP_PREFETCHES) {
                  prefetches_dropped++;
                  break;
                }
              }
              if((intern_->current_cycle() % 128) < PREFETCH_DROP_CHANCE_FAIRNESS.at(get_fairness_factor())) {
                if(USE_FAIRNESS_DROP) {
                  prefetches_dropped++;
                  if((!in_warmup) && DO_DEBUG)
                  fmt::print("\t\tDropped!\n");
                  break;
                }
              }
              //fmt::print("Current usefulness is: {} lookahead: {}\n",current_usefulness,lookaheads);
              //if no pattern found, continue to next pattern size
              if(neg_prefetch_pattern == 0 && !neg_valid) {
                continue_outer = true;
                break;
              }
              else if(neg_prefetch_pattern == 0) {
                //pattern predicted nothing, exit lookahead
                break;
              }

              //prefetch according to pattern
              for(int j = 0; j < i; j++) {
                if(pf_issued >= int(current_pf_degree))
                  break;
                if(neg_prefetch_pattern & (1 << (i - 1 - j))) {
                  if(!USE_PREFETCH_MASK || ((neg_prefetch_pattern & (1 << (i - 1 - j))) & PREFETCH_MASK.at(current_usefulness))) {
                    champsim::address neg_step_addr = champsim::address{champsim::block_number{pf_base_addr} - (j + 1) - lookahead_offset};
                    if(champsim::page_number{neg_step_addr} != champsim::page_number{addr}) {
                      if(CROSS_PAGE && ((direction * s) - (j + 1) - lookahead_offset > -64)) {
                        auto next_region = regions.check_hit(region_type{region->prev_vpn,intern_->current_cycle()});
                        if(next_region.has_value()) {
                          //adjust base address to cross into next page
                          auto page_offset = block_in_page{neg_step_addr};
                          neg_step_addr = champsim::address{champsim::block_number{region->prev_vpn} + page_offset.to<std::size_t>()};
                          page_crossings++;
                        }
                        else {
                          continue;
                        }
                      } else
                          continue; //don't prefetch out of page bounds
                    }
                    if(!check_pagemap(champsim::address{neg_step_addr},true)) {
                      if((pf_issued < free_space) || ((DO_LLC_PREFETCH && !check_llc_pagemap(neg_step_addr)) || !USE_LLC_MAP)) {
                        if(bool prefetch_success = intern_->prefetch_line(neg_step_addr, pf_issued < free_space, metadata_in); prefetch_success) {
                          //fmt::print("AMPM Prefetched: {} Metadata: {}\n",neg_step_addr,metadata_in);
                          if(pf_issued < free_space)
                            add_to_pagemap(neg_step_addr,true,ip,neg_prefetch_pattern,page{},false,get_sample_type(set));
                          else {
                            to_llc++;
                            add_to_llc_pagemap(neg_step_addr);
                          }
                          prefetches_issued++; 
                          pf_issued++;
                          //metadata_in = 0;
                          if(pf_issued >= int(current_pf_degree))
                            break;
                        }
                      } else {
                        prefetches_filtered_llc++;
                      }
                    } else {
                      prefetches_filtered++;
                    }
                  }
                }
              }
              //lookahead check
              if(DO_LOOKAHEAD && pf_issued < int(current_pf_degree)) {
                //update pattern
                auto lookahead = get_prefetch_pattern(neg_prefetch_pattern, order, true);
                neg_access_pattern = neg_prefetch_pattern;
                neg_prefetch_pattern = lookahead.first;
                neg_valid = lookahead.second;
                lookaheads++;
                lookahead_offset += (i);
                if(lookaheads > int(LOOKAHEAD_DEPTH) || current_usefulness < LOOKAHEAD_CONF_CUTOFF)
                  break;
                if(neg_prefetch_pattern != 0)
                  total_lookaheads++;
              } else {
                break;
              }
            }
            //should break to and not continue to smaller patterns
            if(!continue_outer)
              break; //only do highest negative pattern
            order++;
          }
        }
      }
    }
  } else {
    for (auto direction : {1, -1}) {
    for (int i = 1, prefetches_issued = 0; prefetches_issued < 2; i++) {
      const auto pos_step_addr = block_addr + (direction * i);
      const auto neg_step_addr = block_addr - (direction * i);
      const auto neg_2step_addr = block_addr - (direction * 2 * i);

      //goes off physical page
      if(Sppam_Module::page{pos_step_addr}.to<uint64_t>() != Sppam_Module::page{addr}.to<uint64_t>())
        break;

      if (check_pagemap(champsim::address{neg_step_addr},false) && check_pagemap(champsim::address{neg_2step_addr},false) && !check_pagemap(champsim::address{pos_step_addr},true)) {
        // found something that we should prefetch
        if (block_addr != champsim::block_number{pos_step_addr}) {
          champsim::address pf_addr{pos_step_addr};
          if (bool prefetch_success = intern_->prefetch_line(pf_addr, intern_->get_mshr_occupancy_ratio() < 0.5, metadata_in); prefetch_success) {
            //fmt::print("AMPM Prefetched: {} Metadata: {}\n",pf_addr,metadata_in);
            add_to_pagemap(champsim::address{pos_step_addr},true,ip,0,page{},false,get_sample_type(set));
            prefetches_issued++;
          }
        }
      }
    }
  }
  }


}

void sppam::Sppam_Module::print_patterns() {
  fmt::print("Current Patterns in Pattern Table:\n");
  int order = 0;
  for(uint64_t j = PATTERN_SIZE; j >= int(MIN_PATTERN_SIZE); j/=2) {
    fmt::print("Patterns of size {}:\n",j);
    if(TABLE_OR_COUNTER) {
      for(uint64_t i = 0; i < (1ul<<j); i++) {
        auto was_hit = pattern_tables.at(order).check_hit(pattern_type{i});
        auto was_hit_neg = negative_pattern_tables.at(order).check_hit(pattern_type{i});
        if(was_hit.has_value()) {
          fmt::print("Pattern: {:b} Triggers: {} Usefulness: {} Prediction: {:b}\n",i, was_hit->occurrences, was_hit->usefulness * (100/16.0),was_hit->get_prediction().first);
          for(int64_t len = j-1; len >= 0; len--) {
            fmt::print("Pattern: {:b} Prediction Bit {}: Conf: {}\n",i,len, was_hit->prediction_counter.at(len));
          }
        }
        if(was_hit_neg.has_value()) {
          fmt::print("Negative Pattern: {:b} Triggers: {} Usefulness: {} Prediction: {:b}\n",i, was_hit_neg->occurrences, was_hit_neg->usefulness * (100/16.0),was_hit->get_prediction().first);
          for(int64_t len = j-1; len >= 0; len--) {
            fmt::print("Negative Pattern: {:b} Prediction Bit {}: Conf: {}\n",i,len, was_hit_neg->prediction_counter.at(len));
          }
        }
      }
    }
    else {
      for(uint64_t i = 0; i < (1ul<<j); i++) {
        auto was_hit = pattern_tables.at(order).check_hit(pattern_type{i});
        auto was_hit_neg = negative_pattern_tables.at(order).check_hit(pattern_type{i});
        if(was_hit.has_value()) {
          fmt::print("Pattern: {:b} Prediction: {:b} Triggers: {} Usefulness: {}\n",i,was_hit->get_prediction().first, was_hit->occurrences, was_hit->usefulness * (100/16.0));
          was_hit->prediction_table.print_stats();
        }
        if(was_hit_neg.has_value()) {
          fmt::print("Negative Pattern: {:b} Prediction: {:b} Triggers: {} Usefulness: {}\n",i,was_hit_neg->get_prediction().first, was_hit_neg->occurrences, was_hit_neg->usefulness * (100/16.0));
          was_hit_neg->prediction_table.print_stats();
        }
      }
    }
    order++;
  }

}

uint64_t sppam::Sppam_Module::get_state_bits() {
  //Assuming 48-bit max address
  //calculate total bits needed for Sppam_Module
  uint64_t total_bits = 0;

  //start with region table
  uint64_t region_table_bits = (48 - champsim::lg2(SPPAM_PAGE_BITS) - champsim::lg2(REGION_SETS)) + (48 - champsim::lg2(SPPAM_PAGE_BITS))*2; //page nums
  region_table_bits += ((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE)*2; //bitmaps
  if(DO_DUEL)
    region_table_bits += SET_OR_REGION_DUEL ?  0 : (((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE)*2); //sampling
  region_table_bits += SCRAPE_ON_IDLE ? champsim::lg2(SCRAPE_IDLE_TIME) : 0; //scrape count
  region_table_bits += REGION_PATTERN_TAG ? (((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE) * PATTERN_SIZE) : 0;
  //region_table_bits += 4 //momentum (DISABLED)
  //region_table_bits += champsim::lg2(((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE)); //last block
  region_table_bits += SCRAPE_ON_COUNT ? champsim::lg2(SCRAPE_ACCESS_COUNT) : 0;
  region_table_bits += champsim::lg2(REGION_WAYS); //lru bits
  region_table_bits *= REGION_SETS * REGION_WAYS;

  //pattern table
  uint64_t pattern_table_bits = 0; //direct-mapped
  pattern_table_bits += (ADAPTIVE_USEFULNESS || GLOBAL_OR_PATTERN_USEFULNESS) ? ((champsim::lg2(PATTERN_USEFULNESS_SAMPLE)*2) + 4) : 0; //usefulness data
  if(TABLE_OR_COUNTER) {
    //need a 7 bit counter for each bit of the pattern
    pattern_table_bits += PATTERN_SIZE * 7;
  } else {
    pattern_table_bits += (pattern_type::PATTERN_CONF_SETS_WAYS * pattern_type::PATTERN_CONF_SETS)*(PATTERN_SIZE + 7 + champsim::lg2(pattern_type::PATTERN_CONF_SETS_WAYS)); //conf from 0 to 100 (7 bits)
    pattern_table_bits += champsim::lg2(PATTERN_TABLE_WAYS);
  }
  int total_pattern_table_sets = 0;
  int pt_sets = 1 << PATTERN_SIZE;
  //the pattern table can be multi-layered, so calculate each tables sizes
  for(int i = PATTERN_SIZE; i <= MIN_PATTERN_SIZE; i++) {
    total_pattern_table_sets += pt_sets;
    pt_sets = pt_sets / 2;
  }
  pattern_table_bits *= total_pattern_table_sets * PATTERN_TABLE_WAYS;
  pattern_table_bits *= (DO_NEGATIVE && SEPARATE_NEGATIVE_TABLES) ? 2 : 1; //doubled if separate tables

  //CPT table
  uint64_t cpt_bits = 8 + 1 + (48 - champsim::lg2(SPPAM_PAGE_BITS));
  cpt_bits *= CPT_WAYS * CPT_SETS;

  //LLC table
  uint64_t llc_bits = LLC_REGION_SETS * LLC_REGION_WAYS * ((48 - champsim::lg2(SPPAM_PAGE_BITS)) + ((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE)); 


  uint64_t misc_state = 0;
  misc_state += (ADAPTIVE_USEFULNESS || !GLOBAL_OR_PATTERN_USEFULNESS) ? (champsim::lg2((*std::max_element(GLOBAL_USEFULNESS_SAMPLE.begin(),GLOBAL_USEFULNESS_SAMPLE.end())))*2 + 4) : 0; //global usefulness counter
  misc_state += (champsim::lg2(REGION_LIFESPAN_SAMPLE) * 2) + 4; //lifespan sampler
  misc_state += TRACK_LLC_MISSRATE ? (champsim::lg2(LLC_HITRATE_SAMPLE) * 2) + 4 : 0;
  misc_state += DO_DUEL ? champsim::lg2(DUEL_COUNTER_MAX) : 0;
  misc_state += 4 + 4 + 8; //lookahead usefulness reg, bandwidth reg, duel counter


  //total state tally
  total_bits += region_table_bits + pattern_table_bits + cpt_bits + misc_state + llc_bits;


  return total_bits;
}

bool sppam::Sppam_Module::use_pattern_confidence() {
  //static 
  if(!ADAPTIVE_USEFULNESS)
    return GLOBAL_OR_PATTERN_USEFULNESS;

  if(region_lifespan_index >= PATTERN_USEFULNESS_CUTOFF) {
    return true;
  }
  else {
    return false;
  }
}

void sppam::Sppam_Module::modify_pattern_usefulness(champsim::address addr, bool useful) {
  auto [pn, page_offset] = page_and_offset(addr);
  auto region = regions.check_hit(region_type{pn,intern_->current_cycle()});

  //region dueling, track
  if(SET_OR_REGION_DUEL)
    set_duel_tally(get_sample_type(pn.to<uint64_t>()),useful);

  if(!region.has_value()) {
    if(!useful)
      regions_not_found++;
    if(regions_not_found + regions_found >= REGION_LIFESPAN_SAMPLE) {
      region_lifespan_index = std::min(15,int(regions_found) >> (champsim::lg2(REGION_LIFESPAN_SAMPLE) - 4));
      //fmt::print("regions found: {} regions not found: {} region lifespan index: {}\n",regions_found,regions_not_found,region_lifespan_index);
      regions_not_found = 0;
      regions_found = 0;
    }
  }
  if(region.has_value()) {
    if(!useful)
      regions_found++;
    if(regions_not_found + regions_found >= REGION_LIFESPAN_SAMPLE) {
      region_lifespan_index = std::min(15,int(regions_found) >> (champsim::lg2(REGION_LIFESPAN_SAMPLE) - 4));
      //fmt::print("regions found: {} regions not found: {} region lifespan index: {}\n",regions_found,regions_not_found,region_lifespan_index);
      regions_not_found = 0;
      regions_found = 0;
    }
    //don't want to falsely raise pattern confidence
    if(!SET_OR_REGION_DUEL)
      set_duel_tally(region->sample_type.at(page_offset.to<uint64_t>()),useful);
    region->sample_type.at(page_offset.to<uint64_t>()) = 0;
    regions.fill(region.value());
    //this region has gone out of scope
    if(!region->prefetch_map.at(page_offset.to<uint64_t>()))
      return;
    
    uint64_t pattern = 0;


    int order = 0;
    for(int i = PATTERN_SIZE; i >= int(MIN_PATTERN_SIZE); i/=2) {
      //scan forward/backward
      for(int o = 1; o <= i; o++) {

        if(region->momentum > FORWARD_MOMENTUM_MIN) {
          int true_offset = o * -1;
          if(page_offset.to<int>() + true_offset > region->access_map.size())
            continue;
          if(page_offset.to<int>() + true_offset < 0)
            continue;
          auto true_page_offset = page_offset + true_offset;
          if(REGION_PATTERN_TAG) {
            region->pf_trigger.at(page_offset.to<uint64_t>());
            //don't bother with the scan
            if(o > 1)
              continue;
          }
          else {
            auto [pos_pattern, neg_pattern] = get_patterns(pn,true_page_offset);
            pattern = pos_pattern;
          }
          //no pattern
          if(pattern == 0)
            continue;

          auto temp_pattern = pattern_type{pattern & ((1 << (i)) - 1)};
          auto demand_pattern = pattern_tables.at(order).check_hit(temp_pattern);

          if(demand_pattern.has_value()) {
            if(!use_pattern_confidence()) {
              demand_pattern->usefulness = global_usefulness_index;
            }
            else {
              //if(useful)
              //  fmt::print("Pattern: {:b} Usefulness: {}\n",pattern,demand_pattern->usefulness);
              //fmt::print("B4 Pattern: {:b} Useful: {} Confidence: {}\n",pattern,useful,demand_pattern->usefulness);
            
              //demand_pattern->usefulness = useful ? std::min(demand_pattern->usefulness + PATTERN_USEFULNESS_UP, int64_t(PATTERN_USEFULNESS_MAX) - 1) : std::max(demand_pattern->usefulness - PATTERN_USEFULNESS_DOWN, int64_t(0));
              //pattern_tables.at(order).fill(demand_pattern.value());

              if(useful)
                demand_pattern->useful++;
              else
                demand_pattern->useless++;

              if(demand_pattern->useful + demand_pattern->useless >= PATTERN_USEFULNESS_SAMPLE) {
                demand_pattern->usefulness = std::min(15,int(demand_pattern->useful) >> (champsim::lg2(PATTERN_USEFULNESS_SAMPLE) - 4));
                demand_pattern->useful = 0;
                demand_pattern->useless = 0;
              }
            }
            pattern_tables.at(order).fill(demand_pattern.value());

            //fmt::print("Pattern: {:b} Useful: {} Confidence: {}\n",pattern,useful,demand_pattern->usefulness);
          }
        }


        //do same for negative patterns
        if(region->momentum < BACKWARD_MOMENTUM_MIN) {
          int true_offset = o;
          if(page_offset.to<int>() + true_offset > region->access_map.size())
            continue;
          if(page_offset.to<int>() + true_offset < 0)
            continue;
          auto true_page_offset = page_offset + true_offset;
          if(REGION_PATTERN_TAG) {
            region->pf_trigger.at(page_offset.to<uint64_t>());
            //don't bother with the scan
            if(o > 1)
              continue;
          }
          else {
            auto [pos_pattern, neg_pattern] = get_patterns(pn,true_page_offset);
            pattern = neg_pattern;
          }
          //no pattern
          if(pattern == 0)
            continue;
          auto temp_pattern = pattern_type{pattern & ((1 << (i)) - 1)};;
          auto demand_pattern = negative_pattern_tables.at(order).check_hit(temp_pattern);
          if(demand_pattern.has_value()) {
            if(!use_pattern_confidence()) {
              demand_pattern->usefulness = global_usefulness_index;
            }
            else {
              //fmt::print("B4 Negative Pattern: {:b} Useful: {} Confidence: {}\n",pattern,useful,demand_pattern->usefulness);
              
              //demand_pattern->usefulness = useful ? std::min(demand_pattern->usefulness + PATTERN_USEFULNESS_UP, int64_t(PATTERN_USEFULNESS_MAX)- 1) : std::max(demand_pattern->usefulness - PATTERN_USEFULNESS_DOWN, int64_t(0));
              //negative_pattern_tables.at(order).fill(demand_pattern.value());

              if(useful)
                demand_pattern->useful++;
              else
                demand_pattern->useless++;

              if(demand_pattern->useful + demand_pattern->useless >= PATTERN_USEFULNESS_SAMPLE) {
                demand_pattern->usefulness = std::min(15,int(demand_pattern->useful) >> (champsim::lg2(PATTERN_USEFULNESS_SAMPLE) - 4));
                demand_pattern->useful = 0;
                demand_pattern->useless = 0;
              }
            }
            negative_pattern_tables.at(order).fill(demand_pattern.value());

            //fmt::print("Negative Pattern: {:b} Useful: {} Confidence: {}\n",pattern,useful,demand_pattern->usefulness);
          }
        }
      }
      order++;
    }
  }
}

void sppam::prefetcher_cycle_operate() {
  if(intern_->current_cycle() % 1000000 == 0) {
    fmt::print("[{}] SPPAM\n",intern_->NAME);
    fmt::print("\tDuel Counter: {}\n",engine.duel_counter);
    fmt::print("\tBandwidth utilization: {} - {}%\n",(100/16.0)*get_dram_bw(),(100/16.0)*(get_dram_bw()+1));
    fmt::print("\tLLC Miss rate (Bingo+): {} - {}% Misses: {} Hits: {}\n",(100/16.0)*engine.llc_missrate_index,(100/16.0)*(engine.llc_missrate_index+1),engine.llc_miss,engine.llc_hit);
    //fmt::print("Global Usefulness Counter: {} Index: {}\n",engine.global_usefulness_counter,engine.global_usefulness_index);
    fmt::print("\tGlobal usefulness: {} - {}% Useful: {} Useless: {}\n",(100/16.0)*engine.global_usefulness_index,(100/16.0)*(engine.global_usefulness_index+1),engine.global_useful_prefetch,engine.global_useless_prefetch);
    fmt::print("\tLast Prefetch Degree: {}\n",engine.current_pf_degree);
    fmt::print("\tRegions Found: {} Regions Not Found: {} Region Lifespan: {} Pattern Usefulness Enabled: {}\n",engine.regions_found,engine.regions_not_found,engine.region_lifespan_index,(engine.GLOBAL_OR_PATTERN_USEFULNESS && !engine.ADAPTIVE_USEFULNESS) || (engine.use_pattern_confidence() && engine.ADAPTIVE_USEFULNESS));
    fmt::print("\tLLC Unfairness: {}\n",engine.fairness_index);
  }
  //reset stats when exiting warmup
  if(!intern_->warmup && engine.in_warmup) {
    fmt::print("[{}] SPPAM Resetting internal stats\n",intern_->NAME);
    engine.in_warmup = false;
    engine.prefetches_issued = 0;
    engine.total_lookaheads = 0;
    engine.to_llc = 0;
    engine.shadow_patterns_used_forward = 0;
    engine.shadow_patterns_used_backward = 0;
    engine.shadow_scrapes_used_forward = 0;
    engine.shadow_scrapes_used_backward = 0;
    engine.regions_scraped = 0;
    engine.prefetch_triggers = 0;
    engine.prefetches_filtered = 0;
    engine.prefetches_filtered_llc = 0;
    engine.prefetches_dropped = 0;
  }
  //if(intern_->current_cycle() % engine.GLOBAL_USEFUL_INCREMENT_TIME == 0) {
  //    engine.global_usefulness_index = std::min(int(engine.global_usefulness_index + 1), 15);
  //}
  //engine.useless_timer++;
  //if(engine.useless_timer >= engine.GLOBAL_USELESS_INCREMENT_TIMER) {
  //  engine.useless_timer = 0;
  //  engine.global_usefulness_index = std::min(15,(int)engine.global_usefulness_index + 1);
  //}
}

int sppam::Sppam_Module::set_prefetch_degree(uint64_t pattern, int order, bool negative, int prev_usefulness = 15) {

  current_bw_utilization = std::min(15,do_4_bit_mult(get_dram_bw(),std::min(15,((int)llc_missrate_index))) + 1);

  if((GLOBAL_OR_PATTERN_USEFULNESS && !ADAPTIVE_USEFULNESS) || (use_pattern_confidence() && ADAPTIVE_USEFULNESS)) {
    auto demand_pattern = (negative && SEPARATE_NEGATIVE_TABLES) ? negative_pattern_tables.at(order).check_hit(pattern_type{pattern & ((1 << (PATTERN_SIZE >> order)) - 1)}) : pattern_tables.at(order).check_hit(pattern_type{pattern & ((1 << (PATTERN_SIZE >> order)) - 1)});
    if(demand_pattern.has_value()) {

      int new_usefulness = BW_MULT ? std::min(15,do_4_bit_mult(do_4_bit_mult(demand_pattern->usefulness,prev_usefulness),PREFETCH_DEGREES_BW.at(current_bw_utilization)) + 1) :
                                     do_4_bit_mult(demand_pattern->usefulness,prev_usefulness);
      current_pf_degree = BW_MULT ? PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) :
                                    std::max(0l,PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) - PREFETCH_DEGREES_BW.at(current_bw_utilization));
      return BW_MULT ? new_usefulness : std::min(new_usefulness + 1,15);
    } else {
      int new_usefulness = BW_MULT ? std::min(15,do_4_bit_mult(do_4_bit_mult(prev_usefulness,global_usefulness_index),PREFETCH_DEGREES_BW.at(current_bw_utilization)) + 1) :
                                     do_4_bit_mult(prev_usefulness,global_usefulness_index);
      current_pf_degree = BW_MULT ? PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) :
                                    std::max(PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) - PREFETCH_DEGREES_BW.at(current_bw_utilization), 0l );
      return BW_MULT ? new_usefulness : std::min(new_usefulness + 1,15);
    }
  }
  else {
    int new_usefulness = BW_MULT ? std::min(15,do_4_bit_mult(do_4_bit_mult(prev_usefulness,global_usefulness_index),PREFETCH_DEGREES_BW.at(current_bw_utilization)) + 1) :
                                   do_4_bit_mult(prev_usefulness,global_usefulness_index);
    
    current_pf_degree = BW_MULT ? PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) :
                                  std::max(PREFETCH_DEGREES_USEFULNESS.at(new_usefulness) - PREFETCH_DEGREES_BW.at(current_bw_utilization), 0l );
    return BW_MULT ? new_usefulness : std::min(new_usefulness + 1,15);
  }
}

uint8_t sppam::Sppam_Module::get_fairness_factor() {
  if(NUM_CPUS == 1)
    return 15; //cannot be unfair in a single-core sim
  return 15 - fairness_index;
}

void sppam::Sppam_Module::decrease_usefulness_counter() {
  /*
  if(global_usefulness_counter >= GLOBAL_USEFULNESS_DOWN)
    global_usefulness_counter -= GLOBAL_USEFULNESS_DOWN;
  else
    global_usefulness_counter = 0;
  if(global_usefulness_counter <= 0) {
    if(global_usefulness_index > 0)
      global_usefulness_index--;
    global_usefulness_counter = MAX_USEFULNESS_COUNTER >> 1;
  }
  */
  global_useless_prefetch++;
  if(global_useful_prefetch + global_useless_prefetch >= GLOBAL_USEFULNESS_SAMPLE.at(global_usefulness_index)) {
    global_usefulness_index = std::min(15,int(global_useful_prefetch) >> (champsim::lg2(GLOBAL_USEFULNESS_SAMPLE.at(global_usefulness_index)) - 4));
    global_useful_prefetch = 0;
    global_useless_prefetch = 0;
  }
}

void sppam::Sppam_Module::increase_usefulness_counter() {
  /*
  if(global_usefulness_counter + GLOBAL_USEFULNESS_UP <= MAX_USEFULNESS_COUNTER)
    global_usefulness_counter += GLOBAL_USEFULNESS_UP;
  if(global_usefulness_counter >= MAX_USEFULNESS_COUNTER) {
    global_usefulness_counter = global_usefulness_counter >> 1;
    global_usefulness_index = std::min(global_usefulness_index + 1, uint64_t(15));
  }
    */
  global_useful_prefetch++;
  if(global_useful_prefetch + global_useless_prefetch >= GLOBAL_USEFULNESS_SAMPLE.at(global_usefulness_index)) {
    global_usefulness_index = std::min(15,int(global_useful_prefetch) >> (champsim::lg2(GLOBAL_USEFULNESS_SAMPLE.at(global_usefulness_index)) - 4));
    global_useful_prefetch = 0;
    global_useless_prefetch = 0;
  }
}

//approximate a 4-bit multiply of two values representing fractions
int sppam::Sppam_Module::do_4_bit_mult(int op1, int op2) {
  int raw_product = op1 * op2;
  //need to normalize, op1 * op2 is accurate at 0 and diverges as they both approach 15
  //225 is the effective max -> 30 less than what it should be
  //0 * 0 = 0 (0)
  //4 * 4 = 16 (18.1333)  -> 2.133 / 4 = 0.5
  //8 * 8 = 64 (72.533)   -> 8.533 / 8
  //12 * 12 = 144 (163.2) -> 19.2
  //15 * 15 = 225 (255)   -> 30
  return (raw_product) >> 4;
}
  

void sppam::prefetcher_final_stats() {
  fmt::print("[{}] SPPAM Total Prefetches Issued: {} Filtered: {} Total Lookaheads: {} To LLC: {} Filtered: {}\n",intern_->NAME,engine.prefetches_issued,engine.prefetches_filtered + engine.prefetches_filtered_llc, engine.total_lookaheads,engine.to_llc, engine.prefetches_filtered_llc);
  fmt::print("\tAverage Lookahead Depth: {} Dropped: {}\n",(engine.total_lookaheads/(float)engine.prefetch_triggers) + 1,engine.prefetches_dropped);
  fmt::print("\tRegions Scraped: {} Shadow-filled fronts: {} Shadow-filled backs: {} Page Crossings: {}\n",engine.regions_scraped,engine.shadow_scrapes_used_backward,engine.shadow_scrapes_used_forward,engine.page_crossings);
  fmt::print("\tShadow-filled Pattern fronts: {} backs: {}\n",engine.shadow_patterns_used_backward,engine.shadow_patterns_used_forward);
  engine.print_patterns();
}

uint8_t sppam::Sppam_Module::get_sample_type(uint64_t set) {
  auto mask = SAMPLE_FREQ - 1;
  auto shift = champsim::lg2(SAMPLE_FREQ);
  auto low_slice = set & mask;
  auto high_slice = (set >> shift) & mask;

  //fmt::print("{}\n", (SAMPLE_FREQ + low_slice - high_slice) & mask);

  if(((SAMPLE_FREQ + low_slice - high_slice) & mask) == 0) {
    if(set % 2 == 0)
      return SAMPLE_SPPAM;
    else
      return SAMPLE_AMPM;
  }
  // This should return 0 when low_slice == high_slice and 1 ~ (set_sample_rate - 1) otherwise
  return 0;
}

bool sppam::Sppam_Module::use_ampm_or_sppam(int set, champsim::address addr) {
  if(!DO_DUEL)
    return true;

  if(SET_OR_REGION_DUEL) {
    auto pn = page{addr};
    if(get_sample_type(pn.to<uint64_t>()) == SAMPLE_SPPAM)
      return true;
    else if(get_sample_type(pn.to<uint64_t>()) == SAMPLE_AMPM)
      return false;
  } else {
    if(get_sample_type(set) == SAMPLE_SPPAM)
      return true;
    else if(get_sample_type(set) == SAMPLE_AMPM)
      return false;
  }
  return duel_counter > DUEL_COUNTER_MAX >> 1;
}

void sppam::Sppam_Module::set_duel_tally(uint8_t sample_data, bool useful) {
  if((sample_data == SAMPLE_AMPM && useful) || (sample_data == SAMPLE_SPPAM && !useful)) {
    if(!useful)
      duel_counter = duel_counter > 0 ? duel_counter - 1 : 0;
    else
      duel_counter = duel_counter > 0 ? duel_counter - 1 : 0;
    //fmt::print("Tallied for AMPM\n");

  } else if((sample_data == SAMPLE_SPPAM && useful) || (sample_data == SAMPLE_AMPM && !useful)) {
    if(!useful)
      duel_counter = (duel_counter < DUEL_COUNTER_MAX-1) ? (duel_counter + 1) : (DUEL_COUNTER_MAX-1);
    else
      duel_counter = (duel_counter < DUEL_COUNTER_MAX-1) ? (duel_counter + 1) : (DUEL_COUNTER_MAX-1);
    //fmt::print("Tallied for SPPAM\n");
  }
}