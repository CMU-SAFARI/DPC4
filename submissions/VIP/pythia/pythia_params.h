//=======================================================================================//
// File             : pythia/params.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 20/AUG/2025
// Description      : Defines all parameters of Pythia (Bera+, MICRO'21)
//=======================================================================================//

#ifndef __PYTHIA_PARAMS_H__
#define __PYTHIA_PARAMS_H__

#include <cstdint>
#include <string>
#include <vector>

#if 0
#define LOCKED(...) \
  {                 \
    fflush(stdout); \
    __VA_ARGS__;    \
    fflush(stdout); \
  }
#define LOGID() fprintf(stdout, "[%25s@%3u] ", __FUNCTION__, __LINE__);
#define MYLOG(...) LOCKED(LOGID(); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n");)
#else
#define MYLOG(...) \
  {                \
  }
#endif

#define NOT_PORTED \
  do {             \
    assert(false); \
  } while (false)

#define FABS(x) ((x) < 0.0f ? -(x) : (x))

#define DELTA_BITS 7
#define FK_MAX_TILINGS 32

#define DELTA_SIG_MAX_BITS 12
#define DELTA_SIG_SHIFT 3
#define PC_SIG_MAX_BITS 32
#define PC_SIG_SHIFT 4
#define OFFSET_SIG_MAX_BITS 24
#define OFFSET_SIG_SHIFT 4

#define SIG_SHIFT 3
#define SIG_BIT 12
#define SIG_MASK ((1 << SIG_BIT) - 1)
#define SIG_DELTA_BIT 7

namespace PYTHIA
{

//----------------------------//
// General parameters
//----------------------------//
static const float alpha = (float)0.006508802942367162;
static const float gamma = (float)0.556300959940946;
static const float epsilon = (float)0.0018228444309622588;
static const uint32_t seed = 200;
static const std::string policy = std::string("EGreedy");
static const std::string learning_type = std::string("SARSA");
static const std::vector<int32_t> actions = {1, 3, 4, 5, 10, 11, 12, 22, 23, 30, 32, -1, -3, -6, 0};
static const uint32_t pt_size = 256;
static const uint32_t st_size = 64;
static const uint32_t max_pcs = 5;
static const uint32_t max_offsets = 5;
static const uint32_t max_deltas = 5;
static const uint32_t max_actions = 64;
static const uint32_t max_rewards = 16;
static const uint32_t max_degree = 16;
static const uint32_t max_dram_bw_levels = 16;

//----------------------------//
// Reward framework
//----------------------------//
static const bool enable_hbw_reward = true;
static const uint32_t high_bw_thresh = 12;
static const bool enable_reward_out_of_bounds = true;
static const bool enable_reward_all = false; // can be deprecated

static const int32_t reward_correct_timely = 20;
static const int32_t reward_hbw_correct_timely = 20;
static const int32_t reward_correct_untimely = 12;
static const int32_t reward_hbw_correct_untimely = 12;
static const int32_t reward_incorrect = -8;
static const int32_t reward_hbw_incorrect = -14;
static const int32_t reward_none = -4;
static const int32_t reward_hbw_none = -2;
static const int32_t reward_out_of_bounds = -12;
static const int32_t reward_hbw_out_of_bounds = -12;

//----------------------------//
// Degree selection logic
//----------------------------//
static const bool enable_dyn_degree = true;
static const uint32_t action_tracker_size = 2;
static const std::vector<int32_t> last_pref_offset_conf_thresholds = {1, 3, 8};
static const std::vector<int32_t> dyn_degrees_type2 = {1, 2, 4, 6};
static const std::vector<int32_t> last_pref_offset_conf_thresholds_hbw = {1, 3, 8};
static const std::vector<int32_t> dyn_degrees_type2_hbw = {1, 2, 4, 6};

//------------------------------------//
// Knobs for Featurewise learning engine
//------------------------------------//
static const std::vector<int32_t> le_featurewise_active_features = {0, 10};
static const std::vector<int32_t> le_featurewise_num_tilings = {3, 3};
static const std::vector<int32_t> le_featurewise_num_tiles = {128, 128};
static const std::vector<int32_t> le_featurewise_hash_types = {2, 2};
static const std::vector<int32_t> le_featurewise_enable_tiling_offset = {1, 1};
static const uint32_t le_featurewise_pooling_type = 2;

} // namespace PYTHIA

#endif /* __PYTHIA_PARAMS_H__ */
