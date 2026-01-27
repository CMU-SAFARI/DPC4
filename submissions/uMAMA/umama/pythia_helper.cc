//=======================================================================================//
// File             : pythia/pythia_helper.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 20/AUG/2025
// Description      : Implements helper functionalities for Pythia (Bera+, MICRO'21)
//=======================================================================================//

#include "pythia_helper.h"

#include <algorithm>
#include <cassert>
#include <sstream>
#include <string>

const char* MapRewardTypeString[] = {"none", "incorrect", "correct_untimely", "correct_timely", "out_of_bounds", "tracker_hit"};
const char* getRewardTypeString(RewardType type)
{
  assert(type < RewardType::num_rewards);
  return MapRewardTypeString[(uint32_t)type];
}

std::string State::to_string()
{
  std::stringstream ss;

  ss << std::hex << pc << std::dec << "|" << offset << "|" << delta;

  return ss.str();
}

void Scooby_STEntry::update(uint64_t _page, uint64_t pc, uint32_t offset, uint64_t address)
{
  assert(this->page == _page);

  /* insert PC */
  if (this->pcs.size() >= PYTHIA::max_pcs) {
    this->pcs.pop_front();
  }
  this->pcs.push_back(pc);

  /* insert deltas */
  if (!this->offsets.empty()) {
    int32_t delta = (offset > this->offsets.back()) ? (offset - this->offsets.back()) : (-1) * (this->offsets.back() - offset);
    if (this->deltas.size() >= PYTHIA::max_deltas) {
      this->deltas.pop_front();
    }
    this->deltas.push_back(delta);
  }

  /* insert offset */
  if (this->offsets.size() >= PYTHIA::max_offsets) {
    this->offsets.pop_front();
  }
  this->offsets.push_back(offset);
}

/* This is directly inspired by SPP's signature */
uint32_t Scooby_STEntry::get_delta_sig2()
{
  uint32_t curr_sig = 0;

  /* compute signature only using last 4 deltas */
  uint32_t n = (uint32_t)deltas.size();
  uint32_t ptr = (n >= 4) ? (n - 4) : 0;

  for (uint32_t index = ptr; index < deltas.size(); ++index) {
    int sig_delta = (deltas[index] < 0) ? (((-1) * deltas[index]) + (1 << (SIG_DELTA_BIT - 1))) : deltas[index];
    curr_sig = ((curr_sig << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
  }

  return curr_sig;
}

uint32_t Scooby_STEntry::get_pc_sig()
{
  uint32_t signature = 0;

  /* compute signature only using last 4 PCs */
  uint32_t n = (uint32_t)pcs.size();
  uint32_t ptr = (n >= 4) ? (n - 4) : 0;

  for (uint32_t index = ptr; index < pcs.size(); ++index) {
    signature = (signature << PC_SIG_SHIFT);
    signature = (signature ^ (uint32_t)pcs[index]);
  }
  signature = signature & ((1ull << PC_SIG_MAX_BITS) - 1);
  return signature;
}

uint32_t Scooby_STEntry::get_offset_sig()
{
  uint32_t signature = 0;

  /* compute signature only using last 4 offsets */
  uint32_t n = (uint32_t)offsets.size();
  uint32_t ptr = (n >= 4) ? (n - 4) : 0;

  for (uint32_t index = ptr; index < offsets.size(); ++index) {
    signature = (signature << OFFSET_SIG_SHIFT);
    signature = (signature ^ offsets[index]);
  }
  signature = signature & ((1ull << OFFSET_SIG_MAX_BITS) - 1);
  return signature;
}

void Scooby_STEntry::track_prefetch(uint32_t pred_offset, int32_t pref_offset)
{
  if (!bmp_pred[pred_offset]) {
    bmp_pred[pred_offset] = 1;
    total_prefetches++;

    insert_action_tracker(pref_offset);
  }
}

void Scooby_STEntry::insert_action_tracker(int32_t pref_offset)
{
  // bool found = false;
  auto it = find_if(action_tracker.begin(), action_tracker.end(), [pref_offset](ActionTracker* at) { return at->action == pref_offset; });
  if (it != action_tracker.end()) {
    (*it)->conf++;
    /* maintain the recency order */
    action_tracker.erase(it);
    action_tracker.push_back((*it));
  } else {
    if (action_tracker.size() >= PYTHIA::action_tracker_size) {
      ActionTracker* victim = action_tracker.front();
      action_tracker.pop_front();
      delete victim;
    }
    action_tracker.push_back(new ActionTracker(pref_offset, 0));
  }
}

bool Scooby_STEntry::search_action_tracker(int32_t action, int32_t& conf)
{
  conf = 0;
  auto it = find_if(action_tracker.begin(), action_tracker.end(), [action](ActionTracker* at) { return at->action == action; });
  if (it != action_tracker.end()) {
    conf = (*it)->conf;
    return true;
  } else {
    return false;
  }
}