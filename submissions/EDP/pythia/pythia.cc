//=======================================================================================//
// File             : pythia/pythia.cc
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 20/AUG/2025
// Description      : Implements Pythia prefectcher (Bera+, MICRO'21)
//=======================================================================================//

#include "pythia.h"

#include "cache.h"
#include "dpc_api.h"
#include "pythia_params.h"
#include <iostream>



void pythia::prefetcher_initialize()
{
  init_knobs();

#ifdef PYTHIA_SET_DUELING
  std::cout << "[Pythia] Set Dueling Filter Enabled" << std::endl;
  set_dueling_filter.initialize();
#else
  std::cout << "[Pythia] Set Dueling Filter Disabled" << std::endl;
#endif

  last_evicted_tracker = NULL;
  brain_featurewise = new LearningEngineFeaturewise(PYTHIA::alpha, PYTHIA::gamma, PYTHIA::epsilon, (uint32_t)Actions.size(), PYTHIA::seed, PYTHIA::policy,
                                                    PYTHIA::learning_type);
}

uint32_t pythia::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                          uint32_t metadata_in)
{
  uint64_t address = addr.to<uint64_t>();
  uint64_t pc = ip.to<uint64_t>();

#ifdef PYTHIA_SET_DUELING
  // Track access and miss for set dueling
  set_dueling_filter.on_access(address);
  if (!cache_hit) {
    set_dueling_filter.on_miss(address);
  }
#endif

  uint64_t page = address >> LOG2_PAGE_SIZE;
  uint32_t offset = (uint32_t)((address >> LOG2_BLOCK_SIZE) & ((1ull << (LOG2_PAGE_SIZE - LOG2_BLOCK_SIZE)) - 1));

  std::vector<uint64_t> pref_addr; // generated addresses to prefetch

  /* compute reward on demand */
  reward(address);

  /* global state tracking */
  update_global_state(pc, page, offset, address);
  /* per page state tracking */
  Scooby_STEntry* stentry = update_local_state(pc, page, offset, address);

  /* Measure state.
   * state can contain per page local information like delta signature, pc signature etc.
   * it can also contain global signatures like last three branch PCs etc.
   */
  State* state = new State();
  state->pc = pc;
  state->address = address;
  state->page = page;
  state->offset = offset;
  state->delta = !stentry->deltas.empty() ? stentry->deltas.back() : 0;
  state->local_delta_sig2 = stentry->get_delta_sig2();
  state->local_pc_sig = stentry->get_pc_sig();
  state->local_offset_sig = stentry->get_offset_sig();
  state->is_high_bw = is_high_bw(get_dram_bw());

#ifdef PYTHIA_SET_DUELING
  // SET DUELING: consensus_vec[i] = true means Feature i agreed to issue this prefetch.
  // Used to simulate: PC-only (F0), PC_Delta-only (F10), or PC AND PC_Delta strategies.
  uint32_t action_index = brain_featurewise->chooseAction(state);
  std::vector<bool> consensus_vec = brain_featurewise->getConsensusVec(state, action_index);
#endif

  // generate prefetch predictions
  predict(address, page, offset, state, pref_addr);

  /* issue prefetches */
  for (uint32_t addr_index = 0; addr_index < pref_addr.size(); ++addr_index) {
#ifdef PYTHIA_SET_DUELING
    // Filter based on set's assigned candidate strategy during tournament
    if (!set_dueling_filter.should_issue(pref_addr[addr_index], consensus_vec, intern_->cpu)) {
      continue;
    }
#endif
    champsim::address pf_addr{pref_addr[addr_index]};
    intern_->prefetch_line(pf_addr, true, 0);
  }

  return metadata_in;
}

uint32_t pythia::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
#ifdef PYTHIA_SET_DUELING
  set_dueling_filter.on_fill(addr.to<uint64_t>(), evicted_addr.to<uint64_t>(), prefetch);
#endif
  register_fill(addr.to<uint64_t>());
  return metadata_in;
}

void pythia::prefetcher_cycle_operate() {}

void pythia::prefetcher_final_stats() {
#ifdef PYTHIA_SET_DUELING
  set_dueling_filter.print_stats();
#endif
}