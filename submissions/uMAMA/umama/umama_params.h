// Auto-generated Bandit Configuration Header
#pragma once

namespace umama_config {
    enum Config
    {
        Off,
        ARM0,
        ARM1,
        ARM2,
        ARM3,
        ARM4,
        ARM5,
        ARM6,
        ARM7,
        ARM8,
        ARM9,
        ARM10,
        ARM11,
        ARM12,
        ARM13,
        ARM14,
        ARM15,
        ARM16,
        ARM17,
        NumConfigs
    };

    constexpr double d_ucb_gamma = 0.9995;
    constexpr double exploration_bonus = 0.01;
    constexpr uint64_t power7_explore_epoch = 800;
    constexpr uint64_t update_latency = 50;
    constexpr bool normalize_reward_rr = true;
    constexpr int32_t force_arm = -1;
    constexpr uint64_t power7_explore_epoch_instructions = 150;

    constexpr double arbit_gamma = 0.995;
    constexpr double jav_gamma = 0.999;
    constexpr int32_t lsteps_per_gstep = 5;
    constexpr int32_t phase_latency = 200;
    constexpr double conditional_global_threshold = 0.4;
    constexpr int32_t jav_size = 2;
    constexpr double arbit_exploration_bonus = 0.1;
    constexpr size_t bloom_n = 50;

    enum SystemRewardType
    {
        OFF,
        OG,                     // Original reward type, that we used in the original paper
        HARMONIC_SPEEDUP,       // Makes the most sense, but doesn't work well with our approximations
        GEOMEAN_IPC,            // Simple geometric mean of IPCs
        MIN_IPC,                // Minimum IPC across all cores
        WEIGHTED_GEOMEAN_IPC,   // Weight each core's IPC by its rank (so the lowest-IPC core matters most, and the highest-IPC core matters least)
    };

    //constexpr SystemRewardType system_reward_type = OG;
    constexpr SystemRewardType system_reward_type = GEOMEAN_IPC;
    // constexpr SystemRewardType system_reward_type = OFF;
    constexpr bool send_normed_ipc = false;

    constexpr bool egreedy_arbiter = false;
    constexpr int egreedy_chance = 4; // This is a percentage (5%)

    constexpr double global_reward_portion = 0.15;
    
    // Only used for the harmonic speedup reward type
    constexpr double b0 = 0.26553;
    constexpr double b1 = 0.20182;
    constexpr double b2 = -0.18863;
    constexpr double b3 = -0.0753;
    constexpr double alpha_bias = 0.1;
} // namespace umama_config
