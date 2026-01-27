#ifndef LEARNING_ENGINE_FEATUREWISE_H
#define LEARNING_ENGINE_FEATUREWISE_H

#include <random>

#include "feature_knowledge.h"
#include "learning_engine_base.h"

class LearningEngineFeaturewise : public LearningEngineBase
{
private:
  FeatureKnowledge* m_feature_knowledges[NumFeatureTypes];
  double m_max_q_value;

  std::default_random_engine m_generator;
  std::bernoulli_distribution* m_explore;
  std::uniform_int_distribution<int>* m_actiongen;

  std::vector<double> m_q_value_buckets;
  std::vector<uint64_t> m_q_value_histogram;

  /* tracing related knobs */
  uint32_t trace_interval;
  uint64_t trace_timestamp;
  FILE* trace;

  /* stats */
  struct {
    struct {
      uint64_t called;
      uint64_t explore;
      uint64_t exploit;
      uint64_t dist[PYTHIA::max_actions][2]; /* 0:explored, 1:exploited */
      uint64_t fallback;
      uint64_t dyn_fallback_saved_bw;
      uint64_t dyn_fallback_saved_bw_acc;
    } action;

    struct {
      uint64_t called;
      uint64_t su_skip[NumFeatureTypes];
    } learn;

    struct {
      uint64_t total;
      uint64_t feature_align_dist[NumFeatureTypes];
      uint64_t feature_align_all;
    } consensus;

  } stats;

private:
  void init_knobs();
  void init_stats();
  uint32_t getMaxAction(State* state, float& max_q);
  float consultQ(State* state, uint32_t action);
  bool do_fallback(State* state);

public:
  LearningEngineFeaturewise(float alpha, float gamma, float epsilon, uint32_t actions, uint64_t seed, std::string policy, std::string type);
  ~LearningEngineFeaturewise();
  uint32_t chooseAction(State* state);
  void learn(State* state1, uint32_t action1, int32_t reward, State* state2, uint32_t action2, RewardType reward_type);
  void dump_stats();
};

#endif /* LEARNING_ENGINE_FEATUREWISE_H */
