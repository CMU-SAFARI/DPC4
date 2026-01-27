#pragma once

#include "address.h"
#include "champsim.h"
#include "modules.h"
#include "streamer.h"
#include "ip_stride2.h"
#include "umama_params.h"
#include "sms2/sms2.h"
#include "berti.h"
#include "pythia.h"
#include "bloom.h"
#include "bop.h"

struct umama : public champsim::modules::prefetcher {

public:
    using champsim::modules::prefetcher::prefetcher;

    using Config = umama_config::Config;

    enum Variant
    {
        D_UCB = 0,

        NumVariants
    };

    enum Mode
    {
        Explore = 0,
        Exploit,

        NumModes
    };

    // Utility functions
    std::string get_config_string(Config cfg);
    std::string get_mode_string(Mode mode);

    // L1 prefetchers
    uint32_t get_l1_berti_degree(Config config);
    uint32_t get_l1_next_line_degree(Config config);
    uint32_t get_l1_streamer_degree(Config config);

    // L2 prefetchers
    uint32_t get_streamer_degree(Config config);
    uint32_t get_stride_degree(Config config);
    uint32_t get_sms_degree(Config config);
    uint32_t get_l2_bop_degree(Config config);
    uint32_t get_next_line_degree(Config config);

    // L3 prefetchers
    uint32_t get_l3_pythia_degree(Config config);

    uint32_t get_demand(Config config);

    // Necessary functions for prefetchers to have
    void prefetcher_initialize();
    uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in);
    uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
    void prefetcher_cycle_operate();
    void prefetcher_final_stats();
    void print_config();

    uint64_t instructions_processed_last_epoch(size_t i);

private:
    Config config;
    Mode mode;

    uint64_t time_steps = 0;
    uint64_t time_since_restart = 1;
    uint64_t cpu_cycle = 0;

    uint64_t local_rr_arm = 0;
    uint64_t rr_count = 1;
    uint64_t active_num_configs = umama_config::Config::NumConfigs;
    double d_time_steps = 0.0;
    double IPC_norm = 1.0;

    uint64_t local_timesteps = 0;

    uint64_t cycle_stamp = 0;
    uint64_t ret_inst_stamp = 0;
    uint64_t stale_cycle_stamp = 0;
    uint64_t stale_ret_inst_stamp = 0;
    uint64_t stale_phase_count = 0;

    double stale_ipc_update = -1;
    Config stale_config = (Config) 0;

    // L1 prefetchers
    berti *l1_berti_pref = nullptr;
    Streamer<64, 48> *l1_streamer_pref = nullptr;

    // L2 prefetchers
    ip_stride2<128, 4> *stride_pref = nullptr;
    Streamer<256, 48> *streamer = nullptr;
    sms2 *sms_pref;
    bandit_bop::bop *l2_bop_pref;

    // Tracks the most recent 150 accesses to the cache,
    // to attempt to prevent duplicates
    DoubleBloomFilter<L2_BLOOM_N, L2_BLOOM_M> l2_bloom;

    // LLC prefetchers
    pythia *l3_pythia_pref;

    void update_prefetcher();

    Config get_winner_config();
    void invoke_prefetcher_next_line(champsim::address address);

    // umama-related statics

    struct config_record {
        double n;
        double ipc;
        std::vector<Config> conf;
    };

    static std::vector<bool> phase_done;
    static bool phase_override;
    static bool global_phase;
    static uint64_t last_phase_cycle;
    static uint64_t phase_count;
    static uint64_t next_gstep;
    static std::vector<Config> global_config;
    static bool statics_initialized;
    static std::vector<double*> IPC_norms;
    static std::vector<uint64_t> reuse_counters_l1;
    static std::vector<uint64_t> reuse_counters_l2;
    static std::vector<uint64_t> demand_counters_l1;
    static std::vector<uint64_t> demand_counters_l2;
    static std::vector<uint64_t> access_counter;
    static std::vector<bool> use_global_reward;

    static bool activate_bloom_filters;

    static double global_reward;

    // JAV structure
    static std::vector<config_record> global_rewards;
    
    // Arbiter
    static double arbit_rewards[2];
    static double arbit_n_sel[2];
    static double arbit_IPC_norm;

    // For stale data
    static bool stale_global_phase;
    static std::vector<Config> stale_global_config;

    // For tracking the progress of each core
    static std::vector<uint64_t> core_insts_retired;
    static std::vector<uint64_t> core_cycle_counts;
    static std::vector<uint64_t> cycle_counts;
    static std::vector<uint64_t> retired_counts;

    // For round robin randomization
    std::vector<size_t> rr_randomizer;

    // Meta stuff
    uint32_t level = -1;
    struct Stats
    {
        Stats() : called(0), mode(), config(), pred() {}

        uint64_t called = 0;
        struct
        {
            uint64_t explore = 0;
            uint64_t exploit = 0;
            uint64_t explore_to_exploit = 0;
            uint64_t exploit_to_explore = 0;
        } mode;

        struct
        {
            uint64_t histogram[Config::NumConfigs][Mode::NumModes] = {0};
        } config;

        struct
        {
            uint64_t total = 0;
            uint64_t streamer = 0;
            uint64_t stride = 0;
        } pred;
    };

    Stats stats;

    // Stats
    std::vector<double> num_d_sel_vec;
    std::vector<double> reward_vec;
    std::vector<std::vector<double>> rewards_over_time;
    std::vector<std::pair<uint64_t,uint16_t>> vec_cfg_stats;
    std::vector<double> ipc_over_time;
    std::vector<std::vector<config_record>> global_rewards_time;
    std::vector<bool> global_opt_over_time;
    std::vector<double> global_reward_over_time;
    std::vector<std::pair<double, double>> arbit_rewards_over_time;
    std::vector<double> hs_over_time;
    std::vector<std::vector<double>> weights_over_time;
    std::vector<std::vector<double>> alphas_over_time;
    std::vector<std::vector<double>> contrib_over_time; // Somewhat different from IPC
};
