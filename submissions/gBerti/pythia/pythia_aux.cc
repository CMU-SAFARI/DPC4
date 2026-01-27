//=======================================================================================//
// File             : pythia/pythia_aux.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 20/AUG/2025
// Description      : Implements Pythia prefectcher (Bera+, MICRO'21)
//=======================================================================================//

#include <cassert>
#include <iostream>

#include "dpc_api.h"
#include "pythia.h"
#include "pythia_params.h"

#define CHECK_ACTION_SANITY(ai) (assert((ai) < Actions.size()))

void pythia::init_knobs()
{
  for (uint32_t i = 0; i < PYTHIA::actions.size(); ++i)
    Actions.push_back(PYTHIA::actions[i]);
  assert(Actions.size() == PYTHIA::actions.size());
  assert(Actions.size() <= PYTHIA::max_actions);
  assert(PYTHIA::last_pref_offset_conf_thresholds.size() == PYTHIA::dyn_degrees_type2.size() - 1);
}

void pythia::update_global_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address) { /* @rbera TODO: implement */ }

Scooby_STEntry* pythia::update_local_state(uint64_t pc, uint64_t page, uint32_t offset, uint64_t address)
{
  stats.st.lookup++;
  Scooby_STEntry* stentry = NULL;
  auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry* _stentry) { return _stentry->page == page; });
  if (st_index != signature_table.end()) {
    stats.st.hit++;
    stentry = (*st_index);
    stentry->update(page, pc, offset, address);
    signature_table.erase(st_index);
    signature_table.push_back(stentry);
    return stentry;
  } else {
    if (signature_table.size() >= PYTHIA::st_size) {
      stats.st.evict++;
      stentry = signature_table.front();
      signature_table.pop_front();
      delete stentry;
    }

    stats.st.insert++;
    stentry = new Scooby_STEntry(page, pc, offset);
    signature_table.push_back(stentry);
    return stentry;
  }
}

//----------------------------------------------------//
// Main predict function. Does four broader tasks:
// 1. Asks the RL engine to select an action
// 2. Decides on prefetch degree
// 3. Generates corresponding prefetch addresses and
//    inserts into the prefetch tracker (PT).
// 4. Assigns rewards to PT entry in special cases
//    (i.e., no prefetch, or out-of-bounds prefetch)
//----------------------------------------------------//
uint32_t pythia::predict(uint64_t base_address, uint64_t page, uint32_t offset, State* state, std::vector<uint64_t>& pref_addr)
{
  MYLOG("addr@%lx page %lx off %u state %x", base_address, page, offset, state->value());

  stats.predict.called++;

  /* query learning engine to get the next prediction */
  uint32_t action_index = 0;
  uint32_t pref_degree = 1;
  std::vector<bool> consensus_vec; // only required for featurewise engine
  float max_to_avg_q_ratio = 1.0;

  // take an action
  action_index = brain_featurewise->chooseAction(state);
  CHECK_ACTION_SANITY(action_index);

  // select a prefetch degree
  if (PYTHIA::enable_dyn_degree) {
    pref_degree = get_dyn_pref_degree(max_to_avg_q_ratio, page, Actions[action_index]);
  }

  MYLOG("act_idx %u act %d", action_index, Actions[action_index]);

  uint64_t addr = 0xdeadbeef;
  Scooby_PTEntry* ptentry = NULL;
  int32_t predicted_offset = 0;
  if (Actions[action_index] != 0) {
    predicted_offset = (int32_t)offset + Actions[action_index];
    if (predicted_offset >= 0 && predicted_offset < 64) /* falls within the page */
    {
      addr = (page << LOG2_PAGE_SIZE) + (predicted_offset << LOG2_BLOCK_SIZE);
      MYLOG("pred_off %d pred_addr %lx", predicted_offset, addr);

      bool new_addr = track(addr, state, action_index, &ptentry); /* track prefetch */
      if (new_addr) {
        pref_addr.push_back(addr);
        track_in_st(page, predicted_offset, Actions[action_index]);
        stats.predict.issue_dist[action_index]++;
        if (pref_degree > 1) {
          gen_multi_degree_pref(page, offset, Actions[action_index], pref_degree, pref_addr);
        }
        stats.predict.deg_histogram[pref_degree]++;
        ptentry->consensus_vec = consensus_vec;
      } else {
        MYLOG("pred_off %d tracker_hit", predicted_offset);
        stats.predict.pred_hit[action_index]++;
      }
      stats.predict.action_dist[action_index]++;
    } else {
      MYLOG("pred_off %d out_of_bounds", predicted_offset);
      stats.predict.out_of_bounds++;
      stats.predict.out_of_bounds_dist[action_index]++;
      if (PYTHIA::enable_reward_out_of_bounds) {
        addr = 0xdeadbeef;
        track(addr, state, action_index, &ptentry);
        assert(ptentry);
        assign_reward(ptentry, RewardType::out_of_bounds);
        ptentry->consensus_vec = consensus_vec;
      }
    }
  } else {
    MYLOG("no prefecth");
    /* agent decided not to prefetch */
    addr = 0xdeadbeef;
    /* track no prefetch */
    track(addr, state, action_index, &ptentry);
    stats.predict.action_dist[action_index]++;
    ptentry->consensus_vec = consensus_vec;
  }

  stats.predict.predicted += pref_addr.size();
  MYLOG("end@%lx", base_address);

  return (uint32_t)pref_addr.size();
}

//----------------------------------------------------//
// Returns true if the address is
// not already present in prefetch_tracker.
// Otherwise, returns false.
//----------------------------------------------------//
bool pythia::track(uint64_t address, State* state, uint32_t action_index, Scooby_PTEntry** tracker)
{
  MYLOG("addr@%lx state %x act_idx %u act %d", address, state->value(), action_index, Actions[action_index]);
  stats.track.called++;

  bool new_addr = true;
  std::vector<Scooby_PTEntry*> ptentries = search_pt(address, false);
  if (ptentries.empty()) {
    new_addr = true;
  } else {
    new_addr = false;
  }

  if (!new_addr && address != 0xdeadbeef) {
    stats.track.same_address++;
    tracker = NULL;
    return new_addr;
  }

  /* new prefetched address that hasn't been seen before */
  Scooby_PTEntry* ptentry = NULL;

  if (prefetch_tracker.size() >= PYTHIA::pt_size) {
    stats.track.evict++;
    ptentry = prefetch_tracker.front();
    prefetch_tracker.pop_front();
    MYLOG("victim_state %x victim_act_idx %u victim_act %d", ptentry->state->value(), ptentry->action_index, Actions[ptentry->action_index]);
    if (last_evicted_tracker) {
      MYLOG("last_victim_state %x last_victim_act_idx %u last_victim_act %d", last_evicted_tracker->state->value(), last_evicted_tracker->action_index,
            Actions[last_evicted_tracker->action_index]);
      /* train the agent */
      train(ptentry, last_evicted_tracker);
      delete last_evicted_tracker->state;
      delete last_evicted_tracker;
    }
    last_evicted_tracker = ptentry;
  }

  ptentry = new Scooby_PTEntry(address, state, action_index);
  prefetch_tracker.push_back(ptentry);
  assert(prefetch_tracker.size() <= PYTHIA::pt_size);

  (*tracker) = ptentry;
  MYLOG("end@%lx", address);

  return new_addr;
}

//----------------------------------------------------//
// Computes the prefetch degree dynamically.
// Should be called when Pythia makes a prediction.
//----------------------------------------------------//
uint32_t pythia::get_dyn_pref_degree(float max_to_avg_q_ratio, uint64_t page, int32_t action)
{
  uint32_t counted = false;
  uint32_t degree = 1;
  bool high_bw = is_high_bw(get_dram_bw());

  auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry* stentry) { return stentry->page == page; });
  if (st_index != signature_table.end()) {
    int32_t conf = 0;
    bool found = (*st_index)->search_action_tracker(action, conf);
    std::vector<int32_t> conf_thresholds, deg_normal;

    conf_thresholds = high_bw ? PYTHIA::last_pref_offset_conf_thresholds_hbw : PYTHIA::last_pref_offset_conf_thresholds;
    deg_normal = high_bw ? PYTHIA::dyn_degrees_type2_hbw : PYTHIA::dyn_degrees_type2;

    if (found) {
      for (uint32_t index = 0; index < conf_thresholds.size(); ++index) {
        /* pythia_last_pref_offset_conf_thresholds is a sorted list in ascending order of values */
        if (conf <= conf_thresholds[index]) {
          degree = deg_normal[index];
          counted = true;
          break;
        }
      }
      if (!counted) {
        degree = deg_normal.back();
      }
    } else {
      degree = 1;
    }
  }

  return degree;
}

//----------------------------------------------------//
// Generates an array of prefetch candidates based on
// the given page, offset, and prefetch degree.
// Again, should be called from the predict function,
// after the prefetch degree has been decided.
//----------------------------------------------------//
void pythia::gen_multi_degree_pref(uint64_t page, uint32_t offset, int32_t action, uint32_t pref_degree, std::vector<uint64_t>& pref_addr)
{
  stats.predict.multi_deg_called++;
  uint64_t addr = 0xdeadbeef;
  int32_t predicted_offset = 0;
  if (action != 0) {
    for (uint32_t degree = 2; degree <= pref_degree; ++degree) {
      predicted_offset = (int32_t)offset + degree * action;
      if (predicted_offset >= 0 && predicted_offset < 64) {
        addr = (page << LOG2_PAGE_SIZE) + (predicted_offset << LOG2_BLOCK_SIZE);
        pref_addr.push_back(addr);
        MYLOG("degree %u pred_off %d pred_addr %lx", degree, predicted_offset, addr);
        stats.predict.multi_deg++;
        stats.predict.multi_deg_histogram[degree]++;
      }
    }
  }
}

//----------------------------------------------------//
// This reward fucntion is called after seeing
// a demand access to the address.
//
// TODO: what if multiple prefetch request generated the same address?
// Currently, it just rewards the oldest prefetch request to the address
// Should we reward all?
//----------------------------------------------------//
void pythia::reward(uint64_t address)
{
  MYLOG("addr @ %lx", address);

  stats.reward.demand.called++;
  std::vector<Scooby_PTEntry*> ptentries = search_pt(address, PYTHIA::enable_reward_all);

  if (ptentries.empty()) {
    MYLOG("PT miss");
    stats.reward.demand.pt_not_found++;
    return;
  } else {
    stats.reward.demand.pt_found++;
  }

  for (uint32_t index = 0; index < ptentries.size(); ++index) {
    Scooby_PTEntry* ptentry = ptentries[index];
    stats.reward.demand.pt_found_total++;

    MYLOG("PT hit. state %x act_idx %u act %d", ptentry->state->value(), ptentry->action_index, Actions[ptentry->action_index]);
    /* Do not compute reward if already has a reward.
     * This can happen when a prefetch access sees multiple demand reuse */
    if (ptentry->has_reward) {
      MYLOG("entry already has reward: %d", ptentry->reward);
      stats.reward.demand.has_reward++;
      return;
    }

    if (ptentry->is_filled) /* timely */
    {
      assign_reward(ptentry, RewardType::correct_timely);
      MYLOG("assigned reward correct_timely(%d)", ptentry->reward);
    } else {
      assign_reward(ptentry, RewardType::correct_untimely);
      MYLOG("assigned reward correct_untimely(%d)", ptentry->reward);
    }
    ptentry->has_reward = true;
  }
}

//----------------------------------------------------//
// This is another overloaded reward function.
// This variant is called during eviction from prefetch tracker.
//----------------------------------------------------//
void pythia::reward(Scooby_PTEntry* ptentry)
{
  MYLOG("reward PT evict %lx state %x act_idx %u act %d", ptentry->address, ptentry->state->value(), ptentry->action_index, Actions[ptentry->action_index]);

  stats.reward.train.called++;
  assert(!ptentry->has_reward);
  /* this is called during eviction from prefetch tracker
   * that means, this address doesn't see a demand reuse.
   * hence it either can be incorrect, or no prefetch */
  if (ptentry->address == 0xdeadbeef) /* no prefetch */
  {
    assign_reward(ptentry, RewardType::none);
    MYLOG("assigned reward no_pref(%d)", ptentry->reward);
  } else /* incorrect prefetch */
  {
    assign_reward(ptentry, RewardType::incorrect);
    MYLOG("assigned reward incorrect(%d)", ptentry->reward);
  }
  ptentry->has_reward = true;
}

//----------------------------------------------------//
// Asssigns reward to a given prefetch tracker entry.
// Can be called from one of the following four places:
// 1. prefetch goes out of page (inside predict())
// 2. no prefetch (inside predict())
// 3. PT entry sees a demand reuse (inside first variant of reward())
// 4. PT entry gets evicted without seeing a reuse (inside second variant of reward())
//----------------------------------------------------//
void pythia::assign_reward(Scooby_PTEntry* ptentry, RewardType type)
{
  MYLOG("assign_reward PT evict %lx state %x act_idx %u act %d", ptentry->address, ptentry->state->value(), ptentry->action_index,
        Actions[ptentry->action_index]);
  assert(!ptentry->has_reward);

  /* compute the reward */
  int32_t reward = compute_reward(ptentry, type);

  /* assign */
  ptentry->reward = reward;
  ptentry->reward_type = type;
  ptentry->has_reward = true;

  /* maintain stats */
  stats.reward.assign_reward.called++;
  switch (type) {
  case RewardType::correct_timely:
    stats.reward.correct_timely++;
    break;
  case RewardType::correct_untimely:
    stats.reward.correct_untimely++;
    break;
  case RewardType::incorrect:
    stats.reward.incorrect++;
    break;
  case RewardType::none:
    stats.reward.no_pref++;
    break;
  case RewardType::out_of_bounds:
    stats.reward.out_of_bounds++;
    break;
  default:
    assert(false);
  }
  stats.reward.dist[ptentry->action_index][type]++;
}

//----------------------------------------------------//
// Computes the reward depending on the six cases:
// 1. accurate AND timely
// 2. accurate but NOT timely
// 3. inaccurate
// 4. no prefetch
// 5. out-of-bounds prefetch
// 6. regenerated a previously-predicted addr
//----------------------------------------------------//
int32_t pythia::compute_reward(Scooby_PTEntry* ptentry, RewardType type)
{
  bool high_bw = (PYTHIA::enable_hbw_reward && is_high_bw(get_dram_bw())) ? true : false;
  int32_t reward = 0;

  stats.reward.compute_reward.dist[type][high_bw]++;

  if (type == RewardType::correct_timely) {
    reward = high_bw ? PYTHIA::reward_hbw_correct_timely : PYTHIA::reward_correct_timely;
  } else if (type == RewardType::correct_untimely) {
    reward = high_bw ? PYTHIA::reward_hbw_correct_untimely : PYTHIA::reward_correct_untimely;
  } else if (type == RewardType::incorrect) {
    reward = high_bw ? PYTHIA::reward_hbw_incorrect : PYTHIA::reward_incorrect;
  } else if (type == RewardType::none) {
    reward = high_bw ? PYTHIA::reward_hbw_none : PYTHIA::reward_none;
  } else if (type == RewardType::out_of_bounds) {
    reward = high_bw ? PYTHIA::reward_hbw_out_of_bounds : PYTHIA::reward_out_of_bounds;
  } else {
    std::cout << "Invalid reward type found " << type << std::endl;
    assert(false);
  }

  return reward;
}

//----------------------------------------------------//
// Main training functions.
// Invokes the RL engine training.
//----------------------------------------------------//
void pythia::train(Scooby_PTEntry* curr_evicted, Scooby_PTEntry* last_evicted)
{
  MYLOG("victim %s %u %d last_victim %s %u %d", curr_evicted->state->to_string().c_str(), curr_evicted->action_index, Actions[curr_evicted->action_index],
        last_evicted->state->to_string().c_str(), last_evicted->action_index, Actions[last_evicted->action_index]);

  stats.train.called++;
  if (!last_evicted->has_reward) {
    stats.train.compute_reward++;
    reward(last_evicted);
  }
  assert(last_evicted->has_reward);

  /* train */
  MYLOG("===SARSA=== S1: %s A1: %u R1: %d S2: %s A2: %u", last_evicted->state->to_string().c_str(), last_evicted->action_index, last_evicted->reward,
        curr_evicted->state->to_string().c_str(), curr_evicted->action_index);

  /* RL engine training */
  brain_featurewise->learn(last_evicted->state, last_evicted->action_index, last_evicted->reward, curr_evicted->state, curr_evicted->action_index,
                           last_evicted->reward_type);

  MYLOG("train done");
}

//----------------------------------------------------//
// Called when a prefetch request gets filled into the cache.
// Necessary to identify timely prefetches.
//
// TODO: what if multiple prefetch request generated the same address?
// Currently it just sets the fill bit of the oldest prefetch request.
// Do we need to set it for everyone?
//----------------------------------------------------//
void pythia::register_fill(uint64_t address)
{
  MYLOG("fill @ %lx", address);

  stats.register_fill.called++;
  std::vector<Scooby_PTEntry*> ptentries = search_pt(address, PYTHIA::enable_reward_all);
  if (!ptentries.empty()) {
    stats.register_fill.set++;
    for (uint32_t index = 0; index < ptentries.size(); ++index) {
      stats.register_fill.set_total++;
      ptentries[index]->is_filled = true;
      MYLOG("fill PT hit. pref with act_idx %u act %d", ptentries[index]->action_index, Actions[ptentries[index]->action_index]);
    }
  }
}

std::vector<Scooby_PTEntry*> pythia::search_pt(uint64_t address, bool search_all)
{
  std::vector<Scooby_PTEntry*> entries;
  for (uint32_t index = 0; index < prefetch_tracker.size(); ++index) {
    if (prefetch_tracker[index]->address == address) {
      entries.push_back(prefetch_tracker[index]);
      if (!search_all)
        break;
    }
  }
  return entries;
}

int32_t pythia::getAction(uint32_t action_index)
{
  assert(action_index < Actions.size());
  return Actions[action_index];
}

void pythia::track_in_st(uint64_t page, uint32_t pred_offset, int32_t pref_offset)
{
  auto st_index = find_if(signature_table.begin(), signature_table.end(), [page](Scooby_STEntry* stentry) { return stentry->page == page; });
  if (st_index != signature_table.end()) {
    (*st_index)->track_prefetch(pred_offset, pref_offset);
  }
}

bool pythia::is_high_bw(uint8_t bw_level) { return bw_level >= PYTHIA::high_bw_thresh ? true : false; }