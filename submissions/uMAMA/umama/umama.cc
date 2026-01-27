#include "umama.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "cache.h"
#include "dpc_api.h"

#define CPU ((uint8_t)intern_->cpu)
#define CPU_INSTR(cpu) (get_retired_insts((uint8_t)cpu))
#define IS_ROOT (((CPU) == 0) && ((level) == 2))

// #define UMAMA_ASSERT(x) assert(x)
#define UMAMA_ASSERT(x)

#define EXPLORE()         \
  {                       \
    stats.mode.explore++; \
    mode = Mode::Explore; \
  }

#define NO_EXPLORE()      \
  {                       \
    stats.mode.exploit++; \
    mode = Mode::Exploit; \
  }

std::vector<bool> umama::phase_done;
bool umama::phase_override;
bool umama::global_phase;
uint64_t umama::last_phase_cycle;
uint64_t umama::phase_count;
uint64_t umama::next_gstep;
std::vector<umama::Config> umama::global_config;
bool umama::statics_initialized;
std::vector<double*> umama::IPC_norms;
std::vector<uint64_t> umama::access_counter;
std::vector<uint64_t> umama::demand_counters_l1;
std::vector<uint64_t> umama::demand_counters_l2;
std::vector<uint64_t> umama::reuse_counters_l1;
std::vector<uint64_t> umama::reuse_counters_l2;
std::vector<bool> umama::use_global_reward;

double umama::global_reward;

bool umama::activate_bloom_filters;

// JAV structure
std::vector<umama::config_record> umama::global_rewards;

// Arbiter
double umama::arbit_rewards[2];
double umama::arbit_n_sel[2];
double umama::arbit_IPC_norm;

// For stale data
bool umama::stale_global_phase;
std::vector<umama::Config> umama::stale_global_config;

// For tracking the progress of each core
std::vector<uint64_t> umama::core_insts_retired;
std::vector<uint64_t> umama::core_cycle_counts;
std::vector<uint64_t> umama::cycle_counts;
std::vector<uint64_t> umama::retired_counts;

// There has to be some additional logic here so that we can
// handle it when traces reset, which screws up our IPC
// calculation
uint64_t umama::instructions_processed_last_epoch(size_t i)
{
  if (retired_counts[i] > core_insts_retired[i]) {
    return retired_counts[i] - core_insts_retired[i] > 0
          ? retired_counts[i] - core_insts_retired[i]
          : 1;
  } else {
    // We should (in theory) only get here when the trace
    // resets, so we can get away with just returning the
    // number of instructions in retired_counts. This is
    // wrong from the viewpoint of the processor, but easy
    // to implement.
    return retired_counts[i] > 0 ? retired_counts[i] : 1;
  }
}

std::string umama::get_mode_string(Mode md)
{
  switch (md) {
  case Mode::Explore:
    return "Explore";
  case Mode::Exploit:
    return "Exploit";
  default:
    return "UNK";
  }
}

void umama::prefetcher_initialize()
{
  // Select allowable configs
  active_num_configs = Config::NumConfigs;

  // Initialize the D_UCB algorithm
  // local_rr_arm = active_num_configs;
  local_rr_arm = 0;
  cycle_stamp = 0;
  ret_inst_stamp = 0;
  config = (Config)0;
  phase_count = 0;
  stale_phase_count = 0;
  last_phase_cycle = 0;
  global_phase = false;
  stale_global_phase = false;

  // Generate a random order for initial arm exploration for each core.
  srand(CPU);
  for (size_t i = 0; i < active_num_configs; i++) {
    int j;
    do
    {
      j = rand() % (int) active_num_configs;
    } while (std::find(rr_randomizer.begin(), rr_randomizer.end(), j) != rr_randomizer.end());

    rr_randomizer.push_back(j);
  }

  for (uint64_t cfg_it = 0; cfg_it < active_num_configs; cfg_it++) {
    reward_vec.push_back(0.0);
    num_d_sel_vec.push_back(0.0);
  }

  if (!statics_initialized) {
    for (size_t i = 0; i < NUM_CPUS; i++) {
      cycle_counts.push_back(0);
      retired_counts.push_back(0);
      core_cycle_counts.push_back(0);
      core_insts_retired.push_back(0);

      phase_done.push_back(false);
      global_config.push_back((Config)0);
      stale_global_config.push_back((Config)0);
      reuse_counters_l1.push_back(0);
      reuse_counters_l2.push_back(0);
      demand_counters_l1.push_back(0);
      demand_counters_l2.push_back(0);
      access_counter.push_back(0);
      use_global_reward.push_back(false);

      IPC_norms.push_back(nullptr);
    }

    arbit_rewards[0] = 0;
    arbit_rewards[1] = 0;
    arbit_n_sel[0] = 0.001;
    arbit_n_sel[1] = 0.001;
    arbit_IPC_norm = 1;

    global_reward = 0;

    activate_bloom_filters = NUM_CPUS > 1;

    statics_initialized = true;
  }

  /* create component prefetchers */

  if (intern_->NAME.find("L1D") != std::string::npos) {
    level = 1;
  } else if (intern_->NAME.find("L2C") != std::string::npos) {
    level = 2;
  } else if (intern_->NAME.find("LLC") != std::string::npos) {
    level = 3;
  } else {
    UMAMA_ASSERT(false && ("Unknown cache level for umama prefetcher in cache" + intern_->NAME).c_str());
  }

  if (level == 1) {
    // L1 prefetchers
    l1_berti_pref = new berti(intern_);
    UMAMA_ASSERT(l1_berti_pref);
    l1_berti_pref->prefetcher_initialize();
    l1_streamer_pref = new Streamer<64, 48>(intern_);
    UMAMA_ASSERT(l1_streamer_pref);
    l1_streamer_pref->prefetcher_initialize();
  } else if (level == 2) {
    // L2 prefetchers
    streamer = new Streamer<256, 48>(intern_);
    UMAMA_ASSERT(streamer);
    streamer->prefetcher_initialize();
    stride_pref = new ip_stride2<128, 4>(intern_);
    UMAMA_ASSERT(stride_pref);
    sms_pref = new sms2(intern_);
    UMAMA_ASSERT(sms_pref);
    sms_pref->prefetcher_initialize();

    streamer->set_bloom(activate_bloom_filters ? &l2_bloom : nullptr);
    stride_pref->set_bloom(activate_bloom_filters ? &l2_bloom : nullptr);
    sms_pref->set_bloom(activate_bloom_filters ? &l2_bloom : nullptr);

    l2_bop_pref = new bandit_bop::bop(intern_);
    UMAMA_ASSERT(l2_bop_pref);
  } else if (level == 3) {
    // LLC prefetchers
    l3_pythia_pref = new pythia(intern_);
    UMAMA_ASSERT(l3_pythia_pref);
    l3_pythia_pref->prefetcher_initialize();
  }
}

uint32_t umama::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                         uint32_t metadata_in)
{
  stats.called++;

  champsim::block_number block{addr};
  champsim::address block_address{block};
  l2_bloom.insert((uint64_t)block_address.to<uint64_t>());

  if (cache_hit && !useful_prefetch && type != access_type::PREFETCH) {
    if (level == 1)
      reuse_counters_l1[CPU]++;
    else if (level == 2)
      reuse_counters_l2[CPU]++;
  }

  if (type != access_type::PREFETCH) {
    if (level == 1)
      demand_counters_l1[CPU]++;
    else if (level == 2)
      demand_counters_l2[CPU]++;
  }

  if (!intern_->warmup) {
    update_prefetcher();
  }

  config = global_config[CPU];

  /* invoke each individual prefetcher */
  if (level == 1) {

    l1_streamer_pref->set_pref_degree(get_l1_streamer_degree(config));
    
    if (get_l1_berti_degree(config) > 0)
    {
      l1_berti_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    }
    l1_streamer_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    if (get_l1_next_line_degree(config) > 0)
    {
      invoke_prefetcher_next_line(addr);
    }

  } else if (level == 2) {

    streamer->set_pref_degree(get_streamer_degree(config));
    stride_pref->PREFETCH_DEGREE = get_stride_degree(config);

    streamer->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    stride_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);

    sms_pref->PREF_DEGREE = get_sms_degree(config);
    sms_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    if (get_l2_bop_degree(config) > 0)
    {
      l2_bop_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    }
    if (get_next_line_degree(config) > 0)
    {
      invoke_prefetcher_next_line(addr);
    }

  } else if (level == 3) {

    if (get_l3_pythia_degree(config) > 0)
    {
      l3_pythia_pref->prefetcher_cache_operate(addr, ip, cache_hit, useful_prefetch, type, metadata_in);
    }

  } else {
      UMAMA_ASSERT(false && "Invalid cache level in bandit_power7 prefetcher_cache_operate");
  }

  return 0;
}

void umama::update_prefetcher()
{
  if (level != 2) {
    return;
  }

  access_counter[CPU]++;

  if (umama_config::power7_explore_epoch_instructions <= instructions_processed_last_epoch(CPU)) {
    if (access_counter[CPU] == umama_config::power7_explore_epoch) {
      phase_done[CPU] = true;
    }

    if (access_counter[CPU] == 5 * umama_config::power7_explore_epoch) {
      phase_override = true;
    }
  }

  //stats.config.histogram[config][mode]++;
}

umama::Config umama::get_winner_config()
{
  // If we're enforcing a single arm choice, then choose that.
  if constexpr (umama_config::force_arm != -1) {
    return (Config)(umama_config::force_arm >= 0 ? umama_config::force_arm : 0);
  }

  if (global_phase) {
    return global_config[CPU];
  }

  Config best_cfg = (Config)0;

  // Start by scanning all of the arms
  if (local_rr_arm < active_num_configs) {
    best_cfg = (Config)rr_randomizer[local_rr_arm]; // Random per core
    local_rr_arm++;

  } else {
    double max_util = -0.0001;
    double curr_policy_util = -0.0001;

    for (uint32_t index = 0; index < (uint32_t)active_num_configs; ++index) {
      double times_tried = (num_d_sel_vec[index] > 0.00001) ? num_d_sel_vec[index] : 0.00001;
      double cur_util = reward_vec[index] + umama_config::exploration_bonus * std::sqrt(std::log(d_time_steps) / times_tried);

      if(std::isnan(cur_util)) {
        std::cerr << "reward_vec[" << index << "] = " << reward_vec[index] << std::endl
                  << "expl_bonus = " << umama_config::exploration_bonus << std::endl
                  << "d_time_steps = " << d_time_steps << std::endl
                  << "times_tried = " << times_tried << std::endl;
      }
      UMAMA_ASSERT(!std::isnan(cur_util));

      if (index == config) {
        curr_policy_util = cur_util;
      }

      if (cur_util > max_util) {
        max_util = cur_util;
        best_cfg = (Config)index;
      }
    }

    if(!(max_util > 0 && curr_policy_util > 0)) {
      std::cerr << "CPU: " << (int)(CPU) << "max_util = " << max_util << ", curr_policy_util = " << curr_policy_util << std::endl;
    }
    UMAMA_ASSERT(max_util > 0 && curr_policy_util > 0);
  }

  // We may want to scan again, to ensure that the supervisor gets properly normalized results
  if (rr_count > 0 && local_rr_arm == active_num_configs) {
    rr_count--;

    if (rr_count > 0) {
      local_rr_arm = 0;
    } else {
      next_gstep = phase_count + umama_config::lsteps_per_gstep;
    }
  }

  return best_cfg;
}

void umama::invoke_prefetcher_next_line(champsim::address address)
{
  stats.pred.total++;
  champsim::block_number pf_addr{address};
  champsim::address to_fetch{pf_addr + 1};
  if(level == 2) {
    if (activate_bloom_filters && l2_bloom.test((uint64_t) to_fetch.to<uint64_t>())) {
      return;
    }

    l2_bloom.insert((uint64_t) to_fetch.to<uint64_t>());
  }

  prefetch_line(to_fetch, true, 0);
}

uint32_t umama::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  if (level == 1) {
    l1_berti_pref->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
    l1_streamer_pref->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
  } else if (level == 2) {
    streamer->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
    stride_pref->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
    sms_pref->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
  } else if (level == 3) {
    l3_pythia_pref->prefetcher_cache_fill(addr, set, way, prefetch, evicted_addr, metadata_in);
  } else {
    UMAMA_ASSERT(false && "Invalid cache level in umama prefetcher_cache_fill");
  }

  return CPU;
}

void umama::prefetcher_cycle_operate()
{
  if (level == 2 && IPC_norms[CPU] == nullptr) {
    IPC_norms[CPU] = &IPC_norm;
  }

  for(auto p : IPC_norms) {
    if (p == nullptr) {
      return;
    }
  }

  if (intern_->warmup) {
    // In this warmup phase, we don't use the bandit learner at all, but we still call out to
    // the sub-prefetchers, in case any of them have things to do.
    cpu_cycle++;
    if (level == 1) {
      l1_berti_pref->prefetcher_cycle_operate();
      l1_streamer_pref->prefetcher_cycle_operate();
    } else if (level == 2) {
      streamer->prefetcher_cycle_operate();
      stride_pref->prefetcher_cycle_operate();
      sms_pref->prefetcher_cycle_operate();
    } else if (level == 3) {
      l3_pythia_pref->prefetcher_cycle_operate();
    }

    return;

  }

  // If more than half of our CPUs are done with their memory accesses for this phase,
  // then we want to move on.
  auto num_done = std::count(phase_done.begin(), phase_done.end(), true);

  // bool phase_complete = num_done > (int)(NUM_CPUS / 2);
  bool phase_complete = (num_done >= (int)(NUM_CPUS / 2)) && (num_done >= 1);

  if (phase_override) {
    phase_complete = true;
  }

  // These actions are really taken by the central uMama controller, but we use CPU 0
  // here as a stand-in.
  if (IS_ROOT) {

    if (cpu_cycle - last_phase_cycle < umama_config::phase_latency) {
      for (size_t i = 0; i < NUM_CPUS; i++) {
        cycle_counts[i] = cpu_cycle;
        retired_counts[i] = CPU_INSTR(i);
      }
    }

    if (!phase_complete) {
      // Ensure that we keep operating in the current phase if we aren't ready to progress
      last_phase_cycle = cpu_cycle;

    } else if (cpu_cycle - last_phase_cycle >= umama_config::phase_latency + umama_config::update_latency) {
      // Global updates

      // Reset the flags indicating that the phase has completed
      std::fill(phase_done.begin(), phase_done.end(), false);
      phase_override = false;

      vector<Config> best_config = global_config;

      // Compute the system-level reward from the core IPCs
      double system_reward = 0;
      if(phase_count != 0) {
        // Normalize the components that we need to compute the alpha values
        std::vector<double> normed_l1_accesses_per_insts;
        std::vector<double> normed_l1_insts_per_access;
        std::vector<double> normed_l1_insts_per_hits;
        for (size_t i = 0; i < NUM_CPUS; i++) {
          // The first three of these are scaled by the number of instructions executed in the phase
          double l1_accesses_per_inst = (double) demand_counters_l1[i]
                                    / (double) (instructions_processed_last_epoch(i));
          double l1_insts_per_access  = (double) (instructions_processed_last_epoch(i))
                                    / (double) (demand_counters_l1[i] + 1);
          double l1_insts_per_hit     = (double) (instructions_processed_last_epoch(i))
                                    / (double) (reuse_counters_l1[i] + 1);

          normed_l1_accesses_per_insts.push_back(l1_accesses_per_inst);
          normed_l1_insts_per_access.push_back(l1_insts_per_access);
          normed_l1_insts_per_hits.push_back(l1_insts_per_hit);
        }

        // Normalize each vector
        auto normalize_vector = [](std::vector<double>& vec) {
          double sum = std::reduce(vec.begin(), vec.end()) + 0.0001;
          for (size_t i = 0; i < vec.size(); i++) {
            vec[i] /= sum;
          }
        };
        normalize_vector(normed_l1_accesses_per_insts);
        normalize_vector(normed_l1_insts_per_access);
        normalize_vector(normed_l1_insts_per_hits);

        if constexpr (umama_config::system_reward_type == umama_config::OG) {
          double total_accesses = 0;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double count = (double) (demand_counters_l2[i] - reuse_counters_l2[i]) / (double) instructions_processed_last_epoch(i);
            total_accesses += count;
          }

          std::vector<double> ipcs;
          std::vector<double> inverse_baselines;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double core_ipc = (double)(instructions_processed_last_epoch(i)) / (double)(cycle_counts[i] - core_cycle_counts[i]);
            if(umama_config::send_normed_ipc) {
              core_ipc /= *IPC_norms[i];
            }

            // Compare the misses/instr to the other cores
            double rate = (double) (demand_counters_l2[i] - reuse_counters_l2[i]) / (double) instructions_processed_last_epoch(i);
            double inverse_baseline_approx = 1.0;
            if (total_accesses > 0) {
              inverse_baseline_approx = 1.0 - rate / total_accesses;
            }

            ipcs.push_back(core_ipc);
            inverse_baselines.push_back(inverse_baseline_approx);

            system_reward += 1.0 / (core_ipc * inverse_baseline_approx + 0.0001);
          }

          system_reward = (double) NUM_CPUS / system_reward;

          // Compute the weight as the derivative of the harmonic mean with respect to each core's performance
          double common_term = 0;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            common_term += 1.0 / (ipcs[i] * inverse_baselines[i] + 0.0001);
          }
          common_term = ((double) NUM_CPUS) / common_term;

          std::vector<double> weights;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double weight = std::pow(common_term / (ipcs[i] * inverse_baselines[i] + 0.0001), 2) * inverse_baselines[i];
            weights.push_back(weight);

            // If the weight is below a certain point, enable global reward for a given core
            if((weight < umama_config::conditional_global_threshold) && (NUM_CPUS > 1)) {
              use_global_reward[i] = true;
            } else {
              use_global_reward[i] = false;
            }
          }

          weights_over_time.push_back(weights);

        } else if constexpr (umama_config::system_reward_type == umama_config::HARMONIC_SPEEDUP) {
          // Compute the alpha values for each core
          std::vector<double> alphas;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double alpha = umama_config::b1 * normed_l1_accesses_per_insts[i]
                        + umama_config::b2 * normed_l1_insts_per_access[i]
                        + umama_config::b3 * normed_l1_insts_per_hits[i]
                        + umama_config::b0;
            if (alpha < 0.05) { // Some sanity checking to prevent extreme values
              alpha = 0.05;
            } else if (alpha > 0.5) {
              alpha = 0.5;
            }
            alphas.push_back(alpha);
          }

          alphas_over_time.push_back(alphas);

          std::vector<double> ipcs;
          std::vector<double> contribs;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double core_ipc = (double) instructions_processed_last_epoch(i) / (double)(cycle_counts[i] - core_cycle_counts[i]);

            // We use the normalized IPC for uMama
            if constexpr (umama_config::send_normed_ipc) {
              core_ipc /= *IPC_norms[i];
            }

            UMAMA_ASSERT(!std::isnan(core_ipc));
            UMAMA_ASSERT(!std::isnan(alphas[i]));

            // Save this info for use later
            ipcs.push_back(core_ipc);
            system_reward += 1.0 / (core_ipc * alphas[i] + 0.0001);
            contribs.push_back(core_ipc * alphas[i]);
          }

          contrib_over_time.push_back(contribs);

          system_reward = (double) NUM_CPUS / system_reward; // Complete the harmonic mean calculation
          UMAMA_ASSERT(!std::isnan(system_reward));

          // Compute the "usefulness factor" for each core, to determine whether it should use its
          // local reward or the system-level reward.
          double total_weight = 0;
          std::vector<double> weights;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double weight = std::pow(1 / (ipcs[i] + 0.0001), 2) / alphas[i];
            weights.push_back(weight);
            total_weight += weight;
          }

          weights_over_time.push_back(weights);

          for (size_t i = 0; i < NUM_CPUS; i++) {
            double weight = weights[i] / total_weight;

            // If the weight falls below a certain point, enable the global reward for a given core
            use_global_reward[i] = (NUM_CPUS > 1)
                                && (rr_count <= 1)
                                && (weight < umama_config::conditional_global_threshold)
                                && (umama_config::system_reward_type != umama_config::OFF);
          }

        } else if constexpr (umama_config::system_reward_type == umama_config::GEOMEAN_IPC) {
          system_reward = 1.0;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double core_ipc = (double) instructions_processed_last_epoch(i) / (double)(cycle_counts[i] - core_cycle_counts[i]);

            // We use the normalized IPC for uMama
            if constexpr (umama_config::send_normed_ipc) {
              core_ipc /= *IPC_norms[i];
            }

            UMAMA_ASSERT(!std::isnan(core_ipc));

            if(core_ipc < 0.005) {
              core_ipc = 0.005;
            }

            system_reward *= core_ipc;
          }
          system_reward = std::pow(system_reward, 1.0 / (double)NUM_CPUS);

          UMAMA_ASSERT(!std::isnan(system_reward));

          use_global_reward = std::vector<bool>(NUM_CPUS, false);

        } else if constexpr (umama_config::system_reward_type == umama_config::MIN_IPC) {
          system_reward = 1e9;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double core_ipc = (double) instructions_processed_last_epoch(i) / (double)(cycle_counts[i] - core_cycle_counts[i]);

            // We use the normalized IPC for uMama
            if constexpr (umama_config::send_normed_ipc) {
              core_ipc /= *IPC_norms[i];
            }

            UMAMA_ASSERT(!std::isnan(core_ipc));

            if (core_ipc < system_reward) {
              system_reward = core_ipc;
            }
          }
          UMAMA_ASSERT(!std::isnan(system_reward));

          use_global_reward = std::vector<bool>(NUM_CPUS, false);
        } else if constexpr (umama_config::system_reward_type == umama_config::WEIGHTED_GEOMEAN_IPC) {
          system_reward = 1.0;
          std::vector<double> ipcs;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double core_ipc = (double) instructions_processed_last_epoch(i) / (double)(cycle_counts[i] - core_cycle_counts[i]);

            // We use the normalized IPC for uMama
            if constexpr (umama_config::send_normed_ipc) {
              core_ipc /= *IPC_norms[i];
            }

            UMAMA_ASSERT(!std::isnan(core_ipc));

            ipcs.push_back(core_ipc);
          }

          // Figure out the rankings
          std::vector<size_t> rankings(NUM_CPUS, 0);
          for (size_t i = 0; i < NUM_CPUS; i++) {
            for (size_t j = 0; j < NUM_CPUS; j++) {
              if (ipcs[j] > ipcs[i]) {
                rankings[i]++;
              }
            }
          }

          double total_weight = 0;
          for (size_t i = 0; i < NUM_CPUS; i++) {
            double weight = 1 + 0.1 * rankings[i];;
            system_reward *= std::pow(ipcs[i], weight);
            total_weight += weight;
          }
          system_reward = std::pow(system_reward, 1.0 / total_weight);
          UMAMA_ASSERT(!std::isnan(system_reward));

          use_global_reward = std::vector<bool>(NUM_CPUS, false);
        }

        // "Global reward" is a normalized version of the system reward.
        // We will use this for most things, other than the JAV cache.
        UMAMA_ASSERT(!std::isnan(system_reward));
        UMAMA_ASSERT(!std::isnan(arbit_IPC_norm));
        UMAMA_ASSERT(arbit_IPC_norm != 0);
        global_reward = system_reward / arbit_IPC_norm;

        if (rr_count <= 1) {

          // Decay all entries in the JAV
          for (auto& i : global_rewards) {
            i.n = i.n * umama_config::jav_gamma;
          }

          // Determine whether the current configuartion exists in the tracker
          bool found_entry = false;
          for (auto& i : global_rewards) {
            if (i.conf == global_config) {
              found_entry = true;

              // Update the entry
              // i.ipc = (i.ipc * i.n + system_reward) / (i.n + 1);
              i.ipc = (i.n + 1) / (i.n / (i.ipc + 0.00001) + 1 / (system_reward + 0.00001));
              i.n += 1.0;

              break;
            }
          }

          // Insert a new entry if it did not exist
          if (!found_entry) {
            if (global_rewards.size() < umama_config::jav_size) {
              global_rewards.push_back(config_record{1, system_reward, global_config});
            } else {
              // Find the least useful entry
              auto min_elem = std::min_element(global_rewards.begin(), global_rewards.end(), [](const auto& lhs, const auto& rhs) { return lhs.ipc < rhs.ipc; });

              // Only insert if this is better than the worst entry already in the JAV
              if (min_elem->ipc < system_reward) {
                *min_elem = config_record{1, system_reward, global_config};
              }
            }
          }
        }

        auto best_entry = std::max_element(global_rewards.begin(), global_rewards.end(), [](auto a, auto b) { return a.ipc < b.ipc; });
        if (best_entry != global_rewards.end()) {
          best_config = best_entry->conf;
        } else {
          best_config.resize(NUM_CPUS);
          std::fill(best_config.begin(), best_config.end(), (Config) 0);
        }
      }

      hs_over_time.push_back(system_reward);

      phase_count++;
      global_phase = stale_global_phase;

      // Update the global arbiter
      if (rr_count == 0) {
        // arbit_rewards[global_phase] = (arbit_rewards[global_phase] * arbit_n_sel[global_phase] + global_reward) / (arbit_n_sel[global_phase] + 1);
        arbit_rewards[global_phase] = (arbit_n_sel[global_phase] + 1)
                                    / (
                                        arbit_n_sel[global_phase] / (arbit_rewards[global_phase] + 0.00001)
                                        + 1 / (global_reward + 0.00001)
                                      );

        arbit_n_sel[global_phase] += (1.0 / (double)umama_config::lsteps_per_gstep);
        if(std::isnan(arbit_rewards[global_phase])) {
          std::cerr << "arbit_n_sel[" << global_phase << "] = " << arbit_n_sel[global_phase] << std::endl
                    << "global_reward = " << global_reward << std::endl;
        }
        UMAMA_ASSERT(!std::isnan(arbit_rewards[global_phase]));
      }

      arbit_rewards_over_time.push_back(std::pair<double, double>(arbit_rewards[0], arbit_rewards[1]));

      // Normalize the rewards
      if (arbit_rewards[0] > 0.00001 && arbit_rewards[1] > 0.00001) {
        double avg_arbit_rew = (arbit_rewards[0] + arbit_rewards[1]) / 2;
        arbit_IPC_norm *= avg_arbit_rew;
        for (auto& v : arbit_rewards) {
          v /= avg_arbit_rew;
          UMAMA_ASSERT(!std::isnan(v));
        }
      }

      // Handle progression to the next gstep
      if (phase_count >= next_gstep) {
        if ((NUM_CPUS > 1) && (rr_count == 0) && (umama_config::system_reward_type != umama_config::OFF)) {
          // Determine local/global for next timestep
          if constexpr (umama_config::egreedy_arbiter) {
            stale_global_phase = arbit_rewards[1] > arbit_rewards[0];

            if ((rand() % 100) < umama_config::egreedy_chance) {
              stale_global_phase = !stale_global_phase;
            }

          } else {
            double t = arbit_n_sel[0] + arbit_n_sel[1];
            double util_local = arbit_rewards[0] + umama_config::arbit_exploration_bonus * std::sqrt(std::log(t) / arbit_n_sel[0]);
            double util_global = arbit_rewards[1] + umama_config::arbit_exploration_bonus * std::sqrt(std::log(t) / arbit_n_sel[1]);

            stale_global_phase = util_global > util_local;
          }

          // Discount the entries in the global arbiter
          for (auto& n : arbit_n_sel) {
            n *= umama_config::arbit_gamma;
          }

        } else { // rr_count != 0
          stale_global_phase = false;
        }

        next_gstep = phase_count + umama_config::lsteps_per_gstep;
      }

      global_config = best_config;

      // Account for staleness
      std::swap(global_config, stale_global_config);
    }
  }

  // Update local IPC tracker when phase changes
  if (level == 2 && phase_count > stale_phase_count) {

    // We don't do anything on the first phase (since ChampSim is a bit noisy right after warmup)
    if(stale_phase_count != 0) {
      // Update the rewards/counts
      for (auto& n : num_d_sel_vec) {
        n *= umama_config::d_ucb_gamma;
      }

      double ipc_update = stale_ipc_update;
      // Config config_to_update = config;
      Config config_to_update = stale_config;

      // Update using stale information
      if (ipc_update > 0) {
        reward_vec[config_to_update] =
            (reward_vec[config_to_update] * num_d_sel_vec[config_to_update] + ipc_update) / (num_d_sel_vec[config_to_update] + 1);
        num_d_sel_vec[config_to_update] += 1.0;

        UMAMA_ASSERT(!std::isnan(reward_vec[config_to_update]));
      }

      d_time_steps = 0;
      for (const auto& n : num_d_sel_vec) {
        d_time_steps += n;
      }
      local_timesteps++;

      double raw_ipc = (double) instructions_processed_last_epoch(CPU) / (double(cycle_counts[CPU] - core_cycle_counts[CPU]));

      // Normalize with the running IPC
      if (ipc_update > 0 && stale_phase_count == active_num_configs + 2)
      {
        IPC_norm = std::reduce(reward_vec.begin(), reward_vec.end()) / (double) active_num_configs;
        for (auto& x : reward_vec) {
          x /= IPC_norm;
        }
      }

      // Store the information about the most recent epoch for update after the next one
      double local_ipc = (double) instructions_processed_last_epoch(CPU) / ((double) (cycle_counts[CPU] - core_cycle_counts[CPU] + 1)) / IPC_norm;
      if (use_global_reward[CPU]) {
        local_ipc = global_reward;
      }
      if (umama_config::system_reward_type == umama_config::GEOMEAN_IPC) {
        // For geomean IPC, we don't have a great way to deduce which CPUs should receive
        // the global reward. So instead, we mix it in with the reward function of each.
        local_ipc = local_ipc * (1 - umama_config::global_reward_portion) + global_reward * umama_config::global_reward_portion;
      }
      UMAMA_ASSERT(!std::isnan(local_ipc));

      // Store the IPC update for the next round (accounting for stale info)
      stale_ipc_update = local_ipc;
      stale_config = config;

      // Stats
      vec_cfg_stats.push_back(std::pair<uint64_t, uint16_t>(cpu_cycle, (uint16_t)config));
      rewards_over_time.push_back(reward_vec);
      global_rewards_time.push_back(global_rewards);
      ipc_over_time.push_back(raw_ipc);
      global_opt_over_time.push_back(global_phase);
      global_reward_over_time.push_back(use_global_reward[CPU]);

      // Determine the new configuration
      Config old_config = config;
      global_config[CPU] = config = get_winner_config();

      if (old_config != config) {
        EXPLORE();
      } else {
        NO_EXPLORE();
      }
    }

    // Reset a bunch of the trackers
    access_counter[CPU] = 0;
    reuse_counters_l1[CPU] = 0;
    reuse_counters_l2[CPU] = 0;
    demand_counters_l1[CPU] = 0;
    demand_counters_l2[CPU] = 0;
    core_cycle_counts[CPU] = cycle_counts[CPU];
    core_insts_retired[CPU] = retired_counts[CPU];

    stale_phase_count = phase_count;
  }

  // Operate all of the prefetchers
  cpu_cycle++;
  if (level == 1) {
    l1_berti_pref->prefetcher_cycle_operate();
    l1_streamer_pref->prefetcher_cycle_operate();
  } else if (level == 2) {
    streamer->prefetcher_cycle_operate();
    stride_pref->prefetcher_cycle_operate();
    sms_pref->prefetcher_cycle_operate();
  } else if (level == 3) {
    l3_pythia_pref->prefetcher_cycle_operate();
  } else {
    UMAMA_ASSERT(false && "Invalid cache level in umama prefetcher_cycle_operate");
  }
}

void umama::print_config()
{
  std::cout << "power7 config:" << std::endl
            << "umama_config::d_ucb_gamma " << umama_config::d_ucb_gamma << std::endl
            << "umama_config::power7_explore_epoch " << umama_config::power7_explore_epoch << std::endl
            << "umama_config::update_latency " << umama_config::update_latency << std::endl
            << "umama_config::exploration_bonus " << umama_config::exploration_bonus << std::endl
            << "umama_config::force_arm " << umama_config::force_arm << std::endl
            << std::endl;
}

void umama::prefetcher_final_stats()
{
  std::cout << "CPU = " << CPU << std::endl;

  print_config();

  std::cout << "power7.called " << stats.called << std::endl
            << "power7.mode.explore " << stats.mode.explore << std::endl
            << "power7.mode.exploit " << stats.mode.exploit << std::endl
            << "power7.mode.explore_to_exploit " << stats.mode.explore_to_exploit << std::endl
            << "power7.mode.exploit_to_explore " << stats.mode.exploit_to_explore << std::endl
            << std::endl;

  //for (uint32_t cfg = 0; cfg < (uint32_t)active_num_configs; ++cfg) {
  //  for (uint32_t md = (uint32_t)Mode::Explore; md < (uint32_t)Mode::NumModes; ++md) {
  //    std::cout << "power7.config." << get_config_string((Config)cfg) << "." << get_mode_string((Mode)md) << " " << stats.config.histogram[cfg][md]
  //              << std::endl;
  //  }
  //}
  //std::cout << std::endl;

  std::cout << "power7.pred.total " << stats.pred.total << std::endl
            << "power7.pred.streamer " << stats.pred.streamer << std::endl
            << "power7.pred.stride " << stats.pred.stride << std::endl
            << std::endl;

  std::cout << std::endl;
  std::cout << "Vector of configurations in time:" << std::endl;
  for (uint64_t i = 0; i < vec_cfg_stats.size(); i++) {
    std::cout << vec_cfg_stats[i].first << "," << vec_cfg_stats[i].second;

    std::cout << "; [ ";
    for (uint64_t j = 0; j < active_num_configs; j++) {
      std::cout << j << ":" << rewards_over_time[i][j] << " ";
    }
    std::cout << "]";

    std::cout << " " << ipc_over_time[i];
    std::cout << " " << global_opt_over_time[i];

    if (hs_over_time.size() > i) {
      std::cout << " sys:" << hs_over_time[i];
    }

    if (arbit_rewards_over_time.size() > i) {
      std::cout << " ar{" << arbit_rewards_over_time[i].first << "," << arbit_rewards_over_time[i].second << "}";
    }

    if (alphas_over_time.size() > i-1) {
      std::cout << " a{ ";
      for (uint64_t j = 0; j < alphas_over_time[i-1].size(); j++) {
        std::cout << alphas_over_time[i-1][j] << " ";
      }
      std::cout << " }";
    }

    if (contrib_over_time.size() > i-1) {
      std::cout << " c{ ";
      for (uint64_t j = 0; j < contrib_over_time[i-1].size(); j++) {
        std::cout << contrib_over_time[i-1][j] << " ";
      }
      std::cout << " }";
    }

    if (weights_over_time.size() > i-1) {
      std::cout << " w{ ";
      for (uint64_t j = 0; j < weights_over_time[i-1].size(); j++) {
        std::cout << weights_over_time[i-1][j] << " ";
      }
      std::cout << " }";
    }

    std::cout << " { ";
    for(uint64_t j = 0; j < global_rewards_time[i].size(); j++) {
        std::cout << "(";
        for(uint64_t k = 0; k < global_rewards_time[i][j].conf.size(); k++) {
            std::cout << global_rewards_time[i][j].conf[k] << ",";
        }
        std::cout << "):";
        std::cout << global_rewards_time[i][j].ipc << " ";
    }
    std::cout << " }";

    std::cout << " " << global_reward_over_time[i];

    std::cout << std::endl;
  }

  if (level == 1) {
    l1_streamer_pref->prefetcher_final_stats();
  } else if (level == 2) {
    streamer->prefetcher_final_stats();
  } else if (level == 3) {
    // Nothing
  } else {
    UMAMA_ASSERT(false && "Invalid cache level in umama prefetcher_final_stats");
  }
}
