//=======================================================================================//
// File             : pythia/helper.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 20/AUG/2025
// Description      : Implements helper functionalities for Pythia (Bera+, MICRO'21)
//=======================================================================================//

#ifndef __PYTHIA_HELPER_H__
#define __PYTHIA_HELPER_H__

#include <bitset>
#include <cstdint>
#include <deque>
#include <unordered_set>
#include <vector>

#include "pythia_params.h"

#define Bitmap std::bitset<64UL>

typedef enum {
  none = 0,
  incorrect,
  correct_untimely,
  correct_timely,
  out_of_bounds,

  num_rewards
} RewardType;

const char* getRewardTypeString(RewardType type);
inline bool isRewardCorrect(RewardType type) { return (type == correct_timely || type == correct_untimely); }
inline bool isRewardIncorrect(RewardType type) { return type == incorrect; }

class State
{
public:
  uint64_t pc;
  uint64_t address;
  uint64_t page;
  uint32_t offset;
  int32_t delta;
  uint32_t local_delta_sig2;
  uint32_t local_pc_sig;
  uint32_t local_offset_sig;
  bool is_high_bw;

  /* Add more states here */

  void reset()
  {
    pc = 0xdeadbeef;
    address = 0xdeadbeef;
    page = 0xdeadbeef;
    offset = 0;
    delta = 0;
    local_delta_sig2 = 0;
    local_pc_sig = 0;
    local_offset_sig = 0;
    is_high_bw = false;
  }
  State() { reset(); }
  ~State() {}
  std::string to_string();
};

class ActionTracker
{
public:
  int32_t action;
  int32_t conf;
  ActionTracker(int32_t act, int32_t c) : action(act), conf(c) {}
  ~ActionTracker() {}
};

class Scooby_STEntry
{
public:
  uint64_t page;
  std::deque<uint64_t> pcs;
  std::deque<uint32_t> offsets;
  std::deque<int32_t> deltas;
  Bitmap bmp_pred;
  uint64_t trigger_pc;
  uint32_t trigger_offset;
  bool streaming;

  /* tracks last n actions on a page to determine degree */
  std::deque<ActionTracker*> action_tracker;
  std::unordered_set<int32_t> action_with_max_degree;
  std::unordered_set<int32_t> afterburning_actions;

  uint32_t total_prefetches;

public:
  Scooby_STEntry(uint64_t p, uint64_t pc, uint32_t offset) : page(p)
  {
    pcs.clear();
    offsets.clear();
    deltas.clear();
    bmp_pred.reset();
    trigger_pc = pc;
    trigger_offset = offset;
    streaming = false;

    pcs.push_back(pc);
    offsets.push_back(offset);
  }
  ~Scooby_STEntry() {}
  uint32_t get_delta_sig2();
  uint32_t get_pc_sig();
  uint32_t get_offset_sig();
  void update(uint64_t page, uint64_t pc, uint32_t offset, uint64_t address);
  void track_prefetch(uint32_t offset, int32_t pref_offset);
  void insert_action_tracker(int32_t pref_offset);
  bool search_action_tracker(int32_t action, int32_t& conf);
};

class Scooby_PTEntry
{
public:
  uint64_t address;
  State* state;
  uint32_t action_index;
  /* set when prefetched line is filled into cache
   * check during reward to measure timeliness */
  bool is_filled;
  /* set when prefetched line is alredy found in cache
   * donotes extreme untimely prefetch */
  bool pf_cache_hit;
  int32_t reward;
  RewardType reward_type;
  bool has_reward;
  std::vector<bool> consensus_vec; // only used in featurewise engine

  Scooby_PTEntry(uint64_t ad, State* st, uint32_t ac) : address(ad), state(st), action_index(ac)
  {
    is_filled = false;
    pf_cache_hit = false;
    reward = 0;
    reward_type = RewardType::none;
    has_reward = false;
  }
  ~Scooby_PTEntry() {}
};

typedef struct _stats {
  struct {
    uint64_t lookup;
    uint64_t hit;
    uint64_t evict;
    uint64_t insert;
    uint64_t streaming;
  } st;

  struct {
    uint64_t called;
    uint64_t out_of_bounds;
    uint64_t action_dist[PYTHIA::max_actions];
    uint64_t issue_dist[PYTHIA::max_actions];
    uint64_t pred_hit[PYTHIA::max_actions];
    uint64_t out_of_bounds_dist[PYTHIA::max_actions];
    uint64_t predicted;
    uint64_t multi_deg;
    uint64_t multi_deg_called;
    uint64_t multi_deg_histogram[PYTHIA::max_degree + 1];
    uint64_t deg_histogram[PYTHIA::max_degree + 1];
  } predict;

  struct {
    uint64_t called;
    uint64_t same_address;
    uint64_t evict;
  } track;

  struct {
    struct {
      uint64_t called;
      uint64_t pt_not_found;
      uint64_t pt_found;
      uint64_t pt_found_total;
      uint64_t has_reward;
    } demand;

    struct {
      uint64_t called;
    } train;

    struct {
      uint64_t called;
    } assign_reward;

    struct {
      uint64_t dist[PYTHIA::max_rewards][2];
    } compute_reward;

    uint64_t correct_timely;
    uint64_t correct_untimely;
    uint64_t no_pref;
    uint64_t incorrect;
    uint64_t out_of_bounds;
    uint64_t tracker_hit;
    uint64_t dist[PYTHIA::max_actions][PYTHIA::max_rewards];
  } reward;

  struct {
    uint64_t called;
    uint64_t compute_reward;
  } train;

  struct {
    uint64_t called;
    uint64_t set;
    uint64_t set_total;
  } register_fill;

  struct {
    uint64_t called;
    uint64_t set;
    uint64_t set_total;
  } register_prefetch_hit;

  struct {
    uint64_t scooby;
  } pref_issue;

  struct {
    uint64_t epochs;
    uint64_t histogram[PYTHIA::max_dram_bw_levels];
  } bandwidth;
} PythiaStats;

#endif /* __PYTHIA_HELPER_H__ */