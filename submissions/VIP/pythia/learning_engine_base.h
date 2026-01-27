//=======================================================================================//
// File             : pythia/learning_engine_base.h
// Author           : Rahul Bera, SAFARI Research Group (write2bera@gmail.com)
// Date             : 21/AUG/2025
// Description      : Implements a skeleton of an RL engine. This needs to be extended by
//                    a derived class to implement a spefiic type of RL learning.
//=======================================================================================//

#ifndef LEARNING_ENGINE_BASE_H
#define LEARNING_ENGINE_BASE_H

#include <cstdint>
#include <string>

enum Policy {
  InvalidPolicy = 0,
  EGreedy,

  NumPolicies
};

enum LearningType {
  InvalidLearningType = 0,
  QLearning,
  SARSA,

  NumLearningTypes
};

const char* MapPolicyString(Policy policy);
const char* MapLearningTypeString(LearningType type);

class LearningEngineBase
{
protected:
  float m_alpha;
  float m_gamma;
  float m_epsilon;
  uint32_t m_actions;
  uint32_t m_states;
  uint64_t m_seed;
  Policy m_policy;
  LearningType m_type;

protected:
  LearningType parseLearningType(std::string str);
  Policy parsePolicy(std::string str);

public:
  LearningEngineBase(float alpha, float gamma, float epsilon, uint32_t actions, uint32_t states, uint64_t seed, std::string policy, std::string type);
  virtual ~LearningEngineBase() {};
  virtual void dump_stats() = 0;

  inline void setAlpha(float alpha) { m_alpha = alpha; }
  inline float getAlpha() { return m_alpha; }
  inline void setGamma(float gamma) { m_gamma = gamma; }
  inline float getGamma() { return m_gamma; }
  inline void setEpsilon(float epsilon) { m_epsilon = epsilon; }
  inline float getEpsilon() { return m_epsilon; }
  inline void setStates(uint32_t states) { m_states = states; }
  inline uint32_t getStates() { return m_states; }
  inline void setActions(uint32_t actions) { m_actions = actions; }
  inline uint32_t getActions() { return m_actions; }
};

#endif /* LEARNING_ENGINE_BASE_H */
