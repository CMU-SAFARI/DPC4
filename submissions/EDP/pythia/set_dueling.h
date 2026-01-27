#ifndef SET_DUELING_FILTER_H
#define SET_DUELING_FILTER_H

#include <cstdint>
#include <iostream>
#include <vector>

#include "dpc_api.h"

class SetDuelingFilter
{
public:
  bool initialized_ = false;

  // 5 candidates: NoPref, PC, PC_Delta, PC_OR_Delta, PC_AND_Delta
  static constexpr int NUM_CANDIDATES = 5;
  static constexpr int TOTAL_SETS = 2048;

  // Set index extraction (assumes 2048 sets, 64B blocks)
  static constexpr int BLOCK_OFFSET_BITS = 6;
  static constexpr uint64_t SET_MASK = TOTAL_SETS - 1;

  static uint64_t get_set(uint64_t addr) { return (addr >> BLOCK_OFFSET_BITS) & SET_MASK; }

  // Thresholds
  static constexpr double SIGNIFICANCE_THRESHOLD = 0.04; // 4% better than NoPref

  static constexpr uint64_t TOURNAMENT_INSTRS = 10000000; // 10M

  enum Candidate { NoPrefetch = 0, PC = 1, PC_Delta = 2, PC_OR_Delta = 3, PC_AND_Delta = 4 };

  struct CandidateStats {
    uint64_t misses = 0;
    uint64_t accesses = 0;
  };

  CandidateStats cand[NUM_CANDIDATES];

  int winner = PC_OR_Delta;
  int phase = 0;             // 0=tournament, 1=final
  uint64_t phase_start_insts = 0;

  static int get_candidate_for_set(uint64_t set) {
    return set % NUM_CANDIDATES;
  }

  void initialize()
  {
    if (initialized_)
      return;

    for (int c = 0; c < NUM_CANDIDATES; c++) {
      cand[c].misses = 0;
      cand[c].accesses = 0;
    }

    phase = 0;
    phase_start_insts = 0;
    winner = PC_AND_Delta;
    initialized_ = true;

    std::cout << "[SetDueling] Static assignment (set % 5)" << std::endl;
    std::cout << "[SetDueling] Starting tournament immediately" << std::endl;
  }

  void on_access(uint64_t addr)
  {
    if (!initialized_ || phase != 0)
      return;
    uint64_t set = get_set(addr);
    int c = get_candidate_for_set(set);
    cand[c].accesses++;
  }

  void on_miss(uint64_t addr)
  {
    if (!initialized_ || phase != 0)
      return;
    uint64_t set = get_set(addr);
    int c = get_candidate_for_set(set);
    cand[c].misses++;
  }

  void pick_winner()
  {
    double best_ratio = 1.0;
    int best = PC_OR_Delta;

    double nopref_ratio = (cand[NoPrefetch].accesses > 0) ? (double)cand[NoPrefetch].misses / cand[NoPrefetch].accesses : 1.0;

    std::cout << "[SetDueling] Tournament results:" << std::endl;
    const char* names[] = {"NoPref", "PC", "PC_Delta", "PC|Delta", "PC&Delta"};

    for (int c = 0; c < NUM_CANDIDATES; c++) {
      double ratio = (cand[c].accesses > 0) ? (double)cand[c].misses / cand[c].accesses : 1.0;
      std::cout << "  " << names[c] << ": " << cand[c].misses << "/" << cand[c].accesses << " = " << ratio << std::endl;
      if (ratio < best_ratio) {
        best_ratio = ratio;
        best = c;
      }
    }

    // Significance check
    if (best != NoPrefetch && nopref_ratio > 0) {
      double improvement = (nopref_ratio - best_ratio) / nopref_ratio;
      if (improvement < SIGNIFICANCE_THRESHOLD) {
        std::cout << "[SetDueling] Winner " << names[best] << " not significant (" << improvement * 100 << "%), using NoPrefetch" << std::endl;
        best = NoPrefetch;
      }
    }

    winner = best;
    std::cout << "[SetDueling] Winner: " << names[winner] << std::endl;
  }

  bool issue_for_candidate(int c, const std::vector<bool>& consensus_vec)
  {
    switch (c) {
    case NoPrefetch:
      return false;
    case PC:
      return consensus_vec[0];
    case PC_Delta:
      return consensus_vec[10];
    case PC_OR_Delta:
      return consensus_vec[0] || consensus_vec[10];
    case PC_AND_Delta:
      return consensus_vec[0] && consensus_vec[10];
    default:
      return false;
    }
  }

  bool should_issue(uint64_t addr, const std::vector<bool>& consensus_vec, size_t cpu = 0)
  {
    if (!initialized_)
      return true;

    // Phase 0: Tournament - each set uses its assigned candidate
    if (phase == 0) {
      uint64_t current_insts = get_retired_insts(cpu);
      if (current_insts - phase_start_insts >= TOURNAMENT_INSTRS) {
        pick_winner();
        phase = 1;
        std::cout << "[SetDueling] Phase 1: Final" << std::endl;
      }

      uint64_t set = get_set(addr);
      int c = get_candidate_for_set(set);
      return issue_for_candidate(c, consensus_vec);
    }

    // Phase 1: All sets use winner
    return issue_for_candidate(winner, consensus_vec);
  }

  int get_winner() const { return winner; }

  void on_fill(uint64_t addr, uint64_t evicted_addr, uint8_t prefetch) {}

  void print_stats()
  {
    const char* names[] = {"NoPref", "PC", "PC_Delta", "PC|Delta", "PC&Delta"};

    std::cout << "[SetDueling] === Final Statistics ===" << std::endl;
    std::cout << "  Winner: " << names[winner] << std::endl;

    for (int c = 0; c < NUM_CANDIDATES; c++) {
      double ratio = (cand[c].accesses > 0) ? (double)cand[c].misses / cand[c].accesses : 0;
      std::cout << "  " << names[c] << ": miss_ratio=" << ratio << " (acc=" << cand[c].accesses << ", miss=" << cand[c].misses << ")" << std::endl;
    }
  }
};

#endif // SET_DUELING_FILTER_H