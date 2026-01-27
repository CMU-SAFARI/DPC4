#ifndef PREFETCHER_SPPAM_H
#define PREFETCHER_SPPAM_H

#include <array>
#include <bitset>
#include <cstdint>
#include <vector>
#include <optional>

#include "champsim.h"
#include "modules.h"
#include "msl/lru_table.h"
#include "conf_table.h"
#include "dpc_api.h"

class sppam : public champsim::modules::prefetcher
{
  public:
    struct Sppam_Module {
      CACHE* intern_;
      static constexpr std::size_t REGION_SETS = 128;
      static constexpr std::size_t REGION_WAYS = 24;
      static constexpr bool ACCESS_MAP_MISS_ONLY = false;
      static constexpr int FORWARD_MOMENTUM_MIN = -9; //essentially disabled
      static constexpr int BACKWARD_MOMENTUM_MIN = 8; //essentially disabled

      static constexpr std::size_t LLC_REGION_SETS = 64;
      static constexpr std::size_t LLC_REGION_WAYS = 16;

      static constexpr std::size_t PATTERN_TABLE_SETS = 256;
      static constexpr std::size_t PATTERN_TABLE_WAYS = 1;

      static constexpr unsigned int SPPAM_PAGE_BITS = 12;
      static constexpr uint32_t PATTERN_SIZE = 6;
      static constexpr uint32_t MIN_PATTERN_SIZE = 6;

      static constexpr std::size_t CPT_SETS = 128;
      static constexpr std::size_t CPT_WAYS = 1;

      static constexpr uint8_t SAMPLE_AMPM = 1;
      static constexpr uint8_t SAMPLE_SPPAM = 2;



      static constexpr uint64_t MIN_CONFIDENCE_TO_PREFETCH = 50;
      static constexpr bool TRAIN_DEMAND_ONLY = false;
      static constexpr bool PREFETCH_DEMAND_ONLY = false;

      static constexpr bool MARK_AFTER_SCRAPE = true;
      static constexpr bool CLEAR_AFTER_SCRAPE = false;
      static constexpr bool CLEAR_FILTER_AFTER_SCRAPE = true;
      static constexpr bool SCRAPE_ON_EVICT = false;

      static constexpr bool SCRAPE_ON_IDLE = true;
      static constexpr uint64_t SCRAPE_IDLE_TIME = 1000;
      
      static constexpr bool SCRAPE_ON_COUNT = true;
      static constexpr uint64_t SCRAPE_MIN_COUNT = 2;
      static constexpr uint64_t SCRAPE_ACCESS_COUNT = 14;

      static constexpr bool DO_NEGATIVE = false; //originally ran with this disabled, could be issue, unsure
      static constexpr bool SEPARATE_NEGATIVE_TABLES = true;


      static constexpr bool DO_LOOKAHEAD = true;
      static constexpr uint64_t LOOKAHEAD_DEPTH = 100;
      static constexpr uint64_t LOOKAHEAD_CONF_CUTOFF = 7; //min confidence to do a lookahead
      static constexpr uint64_t LOOKAHEAD_CONF_FACTOR = 13; //confidence decay over lookahead (multiplied)

      static constexpr bool TABLE_OR_COUNTER = false; // false = table, true = counter
      static constexpr uint64_t COUNTER_UP = 1;
      static constexpr uint64_t COUNTER_DOWN = 2;

      static constexpr bool SCAN_FORWARD = true;
      static constexpr uint64_t SCAN_DISTANCE_FORWARD = 16;
      static constexpr bool SCAN_BACKWARD = true;
      static constexpr uint64_t SCAN_DISTANCE_BACKWARD = 16;

      static constexpr bool REGION_PATTERN_TAG = false;

      static constexpr bool USELESS_ON_FILL = false;
      //static constexpr uint64_t MAX_USEFULNESS_COUNTER = 2047;
      static constexpr uint64_t GLOBAL_OR_PATTERN_USEFULNESS = true; //false = global, true = pattern
      static constexpr bool     ADAPTIVE_USEFULNESS = true;
      static constexpr uint64_t PATTERN_USEFULNESS_CUTOFF = 7; //pattern or global conf based on region lifespan

      //static constexpr int64_t GLOBAL_USEFULNESS_UP = 4;
      //static constexpr int64_t GLOBAL_USEFULNESS_DOWN = 8;
      //static constexpr int64_t GLOBAL_USEFULNESS_SAMPLE = 1024;

      static constexpr int64_t LLC_HITRATE_SAMPLE = 1024;
      static constexpr bool TRACK_LLC_MISSRATE = false;
      static constexpr bool TRACK_LLC_EVICTS = true;
      static constexpr bool DO_LLC_PREFETCH = true;
      static constexpr bool USE_LLC_MAP = true;
      static constexpr bool DO_VICTIM_PREFETCH = false;

      static constexpr int64_t PATTERN_USEFULNESS_SAMPLE = 256;
      //static constexpr int64_t PATTERN_USEFULNESS_MAX = 512;
      //static constexpr int64_t PATTERN_USEFULNESS_UP = 1;
      //static constexpr int64_t PATTERN_USEFULNESS_DOWN = 3;
      static constexpr bool PROB_DROP_PREFETCHES = true;

      static constexpr uint64_t REGION_LIFESPAN_SAMPLE = 256;

      static constexpr uint64_t DUEL_COUNTER_MAX = 256;
      static constexpr bool DO_DUEL = false;
      static constexpr bool SET_OR_REGION_DUEL = true; //false == set duel, true == region duel
      static constexpr uint64_t SAMPLE_FREQ = 64;

      //debug 
      static constexpr uint64_t DO_DEBUG = false;

      static constexpr uint64_t ALLOW_DROP_THRESH = 8;

      static constexpr bool CROSS_PAGE = false;
      static constexpr uint64_t AGGRESSION_COUNTER_MAX = 256;

      static constexpr bool USE_DEFAULT_PREDICTION = true;
      static constexpr uint64_t DEFAULT_PATTERN = 0b111;
      static constexpr uint64_t DEFAULT_PREDICTION = 1 << (PATTERN_SIZE-1);
      static constexpr bool USE_PREFETCH_MASK = false;

      static constexpr bool USE_FAIRNESS_DROP = false;

      bool BW_MULT = false;
      //std::array<int64_t,16> PREFETCH_DEGREES_BW = {0,0,1,1,1,1,2,2,4,4,8,8,12,12,16,16}; //heavy
      std::array<int64_t,16> PREFETCH_DEGREES_BW = {0,0,0,0,1,1,1,1,2,2,2,4,4,4,8,8}; //normal
      //std::array<int64_t,16> PREFETCH_DEGREES_BW = {0,0,0,0,0,0,0,1,1,1,2,2,2,4,4,4}; //lighter
      //std::array<int64_t,16> PREFETCH_DEGREES_BW = {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}; //lightest
      //std::array<int64_t,16> PREFETCH_DEGREES_BW = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //off
      std::array<int64_t,16> PREFETCH_MASK = {0x20,0x20,0x20,0x20,0x20,0x30,0x30,0x38,0x38,0x3c,0x3c,0x3e,0x3e,0x3f,0x3f,0x3f};
      //std::array<int64_t,16> PREFETCH_DEGREES_BW = {15,15,15,14,14,14,14,14,13,13,13,12,12,12,8,8};
      std::array<int64_t,16> PREFETCH_DEGREES_USEFULNESS = {1,1,2,2,2,3,3,3,4,4,8,8,12,12,16,16}; //normal
      //std::array<int64_t,16> PREFETCH_DEGREES_USEFULNESS = {1,1,1,1,1,1,1,1,2,2,2,2,4,4,4,4}; //light
      //std::array<int64_t,16> PREFETCH_DEGREES_USEFULNESS = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; //heavy throttle
      std::array<int64_t,16> PREFETCH_DROP_CHANCE_USEFULNESS = {123,120,110,110,80,50,10,0,0,0,0,0,0,0,0,0}; //out of 128 //heavy
      std::array<int64_t,16> PREFETCH_DROP_CHANCE_FAIRNESS = {115,115,100,100,90,90,60,60,30,30,0,0,0,0,0,0}; //out of 128 //heavy
      //std::array<int64_t,16> PREFETCH_DROP_CHANCE_USEFULNESS = {80,60,50,40,30,20,10,0,0,0,0,0,0,0,0,0}; //out of 128 //light
      //std::array<int64_t,16> PREFETCH_DROP_CHANCE_USEFULNESS = {123,110,100,90,80,50,10,0,0,0,0,0,0,0,0,0}; //out of 128 //medium
      std::array<int64_t,16> GLOBAL_USEFULNESS_SAMPLE = {1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024};

      uint64_t global_useful_prefetch = 0;
      uint64_t global_useless_prefetch  = 0;

      uint64_t current_bw_utilization = 0;
      int64_t current_pf_degree = 0;

      uint64_t regions_found = 0;
      uint64_t regions_not_found = 0;

      uint64_t llc_hit = 0;
      uint64_t llc_miss = 0;

      uint64_t region_lifespan_index = 0;

      uint64_t llc_missrate_index = 15;

      uint64_t duel_counter = DUEL_COUNTER_MAX >> 1;
      uint64_t fairness_index = 0;

      //stats
      bool in_warmup = true;
      uint64_t prefetches_issued = 0;
      uint64_t total_lookaheads = 0;
      uint64_t to_llc = 0;
      uint64_t shadow_patterns_used_forward = 0;
      uint64_t shadow_patterns_used_backward = 0;
      uint64_t shadow_scrapes_used_forward = 0;
      uint64_t shadow_scrapes_used_backward = 0;
      uint64_t regions_scraped = 0;
      uint64_t prefetch_triggers = 0;
      uint64_t prefetches_filtered = 0;
      uint64_t prefetches_filtered_llc = 0;
      uint64_t prefetches_dropped = 0;
      uint64_t page_crossings = 0;


      //uint64_t global_usefulness_counter = MAX_USEFULNESS_COUNTER >> 1;
      uint64_t global_usefulness_index = 0;
      
      struct page_extent : champsim::dynamic_extent {
        page_extent() : dynamic_extent(champsim::data::bits{64}, champsim::data::bits{SPPAM_PAGE_BITS}) {}
      };
      using page = champsim::address_slice<page_extent>;

      struct block_in_page_extent : champsim::dynamic_extent {
        block_in_page_extent() : dynamic_extent(champsim::data::bits{SPPAM_PAGE_BITS}, champsim::data::bits{LOG2_BLOCK_SIZE}) {}
      };
      using block_in_page = champsim::address_slice<block_in_page_extent>;

      struct region_type {
        page vpn;
        page next_vpn;
        page prev_vpn;
        std::vector<bool> access_map{};
        std::vector<bool> prefetch_map{};
        std::vector<uint8_t> sample_type{}; 
        uint64_t last_access_time = 0;
        std::vector<uint64_t> pf_trigger{};
        int momentum = 0;
        uint64_t last_block = 0;
        uint64_t since_last_scrape = 0;




        region_type() : region_type(page{},0) {}
        explicit region_type(page allocate_vpn, uint64_t last_access_time_)
          : vpn(allocate_vpn), next_vpn(page{}), prev_vpn(page{}), access_map((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE), prefetch_map((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE), sample_type((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE),last_access_time(last_access_time_), pf_trigger(((1 << SPPAM_PAGE_BITS) / BLOCK_SIZE))
        {
        }
      };


      struct sppam_indexer {
        auto operator()(const region_type& entry) const { return entry.vpn; }
      };
      champsim::msl::lru_table<region_type,sppam_indexer,sppam_indexer> regions{REGION_SETS,REGION_WAYS};

      champsim::msl::lru_table<region_type,sppam_indexer,sppam_indexer> llc_regions{LLC_REGION_SETS,LLC_REGION_WAYS};

      struct cross_page_tracker_type {
        uint32_t stream_id;
        page vpn;
        bool forward;

        cross_page_tracker_type() : cross_page_tracker_type(0,page{}) {}
        explicit cross_page_tracker_type(uint32_t stream_id_, page vpn_) : stream_id(stream_id_), vpn(vpn_), forward(true) {}
      };

      struct cpt_indexer {
        auto operator()(const cross_page_tracker_type& entry) const { return entry.stream_id; }
      };

      champsim::msl::lru_table<cross_page_tracker_type,cpt_indexer,cpt_indexer> cpt_table{CPT_SETS,CPT_WAYS};



      struct pattern_type {
        static constexpr std::size_t PATTERN_CONF_SETS = 1;
        static constexpr std::size_t PATTERN_CONF_SETS_WAYS = 16;
        uint64_t pattern = 0;
        uint64_t occurrences = 0;
        uint64_t useful = 0; //8 bit value
        uint64_t useless = 0; //8 bit value
        int64_t usefulness = 8; //4 bit value
        struct pattern_conf_indexer {
            auto operator()(const uint64_t& entry) const { return entry; }
        };
        sppam_util::msl::conf_table<uint64_t,pattern_conf_indexer,pattern_conf_indexer> prediction_table{PATTERN_CONF_SETS,PATTERN_CONF_SETS_WAYS};

        std::array<uint64_t,PATTERN_SIZE> prediction_counter{};

        std::pair<uint64_t,bool> get_prediction() {
          if(TABLE_OR_COUNTER) {
            uint64_t prediction = 0;
            for(int i = 0; i < int(prediction_counter.size()); i++) {
              if(prediction_counter[i] >= MIN_CONFIDENCE_TO_PREFETCH) {
                prediction |= (1ULL << i);
              }
            }
            if(prediction != 0) {
              return {prediction, true};
            } else {
              return {0, true};
            }
          }
          else {
            auto prediction = prediction_table.get_highest_conf(0,MIN_CONFIDENCE_TO_PREFETCH);
            if(prediction.has_value()) {
              return {prediction.value(), true};
            } else {
              return {0, false};
            }
          }
        }

        void increment_prediction(uint64_t prediction) {
          if(TABLE_OR_COUNTER) {
            for(int i = 0; i < int(prediction_counter.size()); i++) {
              if(prediction & (1ULL << i)) {
                if(prediction_counter[i] + COUNTER_UP < 100)
                  prediction_counter[i] += COUNTER_UP;
                else
                  prediction_counter[i] = 100;
              } else {
                if(prediction_counter[i] > COUNTER_DOWN)
                  prediction_counter[i] -= COUNTER_DOWN;
                else
                  prediction_counter[i] = 0;
              }
            }
          } else {
            if(!prediction_table.incr_conf(prediction).has_value())
              prediction_table.fill(prediction);
          }
        }

        pattern_type() : pattern_type(0) {}
        explicit pattern_type(uint64_t pattern_)
          : pattern(pattern_)
        {
        }
      };

      struct pattern_table_indexer {
        auto operator()(const pattern_type& entry) const { return entry.pattern; }
      };
      //champsim::msl::lru_table<pattern_type,pattern_table_indexer,pattern_table_indexer> pattern_table{PATTERN_TABLE_SETS,PATTERN_TABLE_WAYS};

      void add_to_pagemap(champsim::address addr, bool prefetch, champsim::address ip = champsim::address{}, uint64_t pf_trigger = 0, page prev_page = page{}, bool direction = true, uint8_t sample_type = 0);
      bool check_pagemap(champsim::address addr, bool prefetch);

      void add_to_llc_pagemap(champsim::address addr);
      bool check_llc_pagemap(champsim::address addr);
      void remove_from_llc_pagemap(champsim::address addr);

      void remove_from_pagemap(champsim::address addr, bool prefetch);

      int set_prefetch_degree(uint64_t pattern, int order, bool negative, int prev_usefulness);
      void decrease_usefulness_counter();
      void increase_usefulness_counter();
      bool use_pattern_confidence();

      void track_llc_missrate(uint32_t hit, bool prefetch);

      std::pair<page,bool> get_prev_page(uint32_t stream_id, page current_vpn);

      void modify_pattern_usefulness(champsim::address addr, bool useful);

      std::pair<uint64_t,uint64_t> get_patterns(page pn, block_in_page page_offset);

      std::pair<uint64_t, bool> get_prefetch_pattern(uint64_t access_map, int order, bool negative);
      void increment_access_pattern(uint64_t pattern, uint64_t prediction, int order, bool negative);

      int do_4_bit_mult(int op_1, int op_2);

      uint8_t get_sample_type(uint64_t set);

      uint8_t get_fairness_factor();

      void set_duel_tally(uint8_t sample_data, bool useful);

      bool use_ampm_or_sppam(int set, champsim::address addr);


      void scrape_region(champsim::address addr);

      int get_momentum(champsim::address addr);

      void print_patterns();

      template <typename T>
      static auto page_and_offset(T addr) -> std::pair<page, block_in_page>;
      void do_prefetch(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);

      std::vector<champsim::msl::lru_table<pattern_type,pattern_table_indexer,pattern_table_indexer>> pattern_tables;
      std::vector<champsim::msl::lru_table<pattern_type,pattern_table_indexer,pattern_table_indexer>> negative_pattern_tables;

      void initialize(CACHE* cache);

      uint64_t get_state_bits();
    };



  
  
  uint64_t prev_useless_prefetches = 0;
  Sppam_Module engine;
  using prefetcher::prefetcher;
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, bool prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_initialize() {
    engine.initialize(intern_);
  }
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
  
};

#endif