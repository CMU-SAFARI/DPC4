#ifndef GAZE_H
#define GAZE_H

/*
 * Gaze into the Pattern: Characterizing Spatial Patterns with Internal Temporal Correlations for Hardware Prefetching
 *
 * To appear in 31st IEEE International Symposium on High-Performance Computer Architecture (HPCA 2025),
 * 3/1/2025-3/5/2025, Las Vegas, NV, USA
 *
 * @Authors: Zixiao Chen, Chentao Wu, Yunfei Gu, Ranhao Jia, Jie Li, and Minyi Guo
 * @Manteiners: Zixiao Chen
 * @Email: chen_zx@sjtu.edu.cn
 * @Date: 12/02/2024
 */

#include "custom_util.h"
#include "cache.h"
#include "champsim.h"
#include "modules.h"

#include <stdint.h>
#include <random>
#include <deque>

namespace gaze_impl {

#define __region_offset(block_num) (block_num & REGION_OFFSET_MASK)
#define __add_pf_dest_level(pf_metadata, level) (pf_metadata | (level << 30))
#define __add_pf_sour_level(pf_metadata, level) (pf_metadata | (level << 28))

#define FT_TYPE custom_util::SRRIPSetAssociativeCache
#define AT_TYPE custom_util::LRUSetAssociativeCache
#define PT_TYPE custom_util::LRUSetAssociativeCache
#define PB_TYPE custom_util::LRUSetAssociativeCache

const uint64_t REGION_SIZE = 4 * 1024; // '4KB', '8KB', '16KB, '32KB', '64KB, '128KB', '512KB', '1024KB', '2048KB'
const uint64_t LOG2_REGION_SIZE = champsim::lg2(REGION_SIZE);
const uint64_t REGION_OFFSET_MASK = (1ULL << (LOG2_REGION_SIZE - LOG2_BLOCK_SIZE)) - 1;

const int NUM_BLOCKS = static_cast<int>(REGION_SIZE / BLOCK_SIZE);

const int FT_SIZE = 64, FT_WAY = 8;
const int AT_SIZE = 64, AT_WAY = 8;
const int PT_WAY = 4;
const int PT_SIZE = PT_WAY * NUM_BLOCKS;
const int PB_SIZE = 32, PB_WAY = 8;

const int STRIDE_PF_LOOKAHEAD = 2;
const int PF_FILL_L1 = 1;
const int PF_FILL_L2 = 2;

// ------------------------- Util Functions ------------------------- //
std::pair<float, float> calculate_acc_and_cov(std::vector<int> pattern_1, std::vector<int> pattern_2);
bool different_patterns(std::vector<int> pattern_1, std::vector<int> pattern_2);
std::vector<int> pattern_bool2int(std::vector<bool> pattern);
bool pattern_all_set(std::vector<bool> pattern);
bool pattern_all_set(std::vector<int> pattern);

// ------------------------- Filter Table ------------------------- //
struct FilterTableData {
    uint64_t trigger_offset;
    uint64_t pc;
};

class FilterTable : public FT_TYPE<FilterTableData> {
    typedef FT_TYPE<FilterTableData> Super;

private:
    uint64_t build_key(uint64_t region_num);
    void write_data(Entry& entry, custom_util::Table& table, int row);

public:
    FilterTable(int size, int num_ways);

    Entry* find(uint64_t region_num);
    void insert(uint64_t region_num, uint64_t trigger_offset, uint64_t pc);
    Entry* erase(uint64_t region_num);

    std::string log();
};

// ------------------------- Accumulate Table ------------------------- //
struct AccumulateTableData {
    uint64_t trigger_offset;
    uint64_t second_offset;
    uint64_t pc;
    bool missed_in_pt;
    std::vector<bool> pattern;
    std::vector<int> order;

    int last_stride;
    uint64_t last_offset;

    bool con = false;

    int timestamp = 2;
};

class AccumulateTable : public AT_TYPE<AccumulateTableData> {
    typedef AT_TYPE<AccumulateTableData> Super;

private:
    bool __stride_prefetch = false;

    void write_data(Entry& entry, custom_util::Table& table, int row);
    uint64_t build_key(uint64_t region_num);

public:
    bool get_stride_prefetch();
    void turn_off_stride_prefetch();

public:
    AccumulateTable(int size, int num_ways);

    Entry* set_pattern(uint64_t region_num, uint64_t offset);

    Entry insert(uint64_t region_num, uint64_t trigger_offset, uint64_t second_offset, uint64_t pc, bool missed_in_pt, bool con);
    Entry* erase(uint64_t region_num);

    std::string log();
};

// ------------------------- Pattern Table ------------------------- //
struct PatternTableData {
    std::vector<int> pattern;
    uint64_t pc;
    bool con = false;

    PatternTableData() {
        pattern = std::vector<int>(NUM_BLOCKS, 0);
        pc = 0;
    }
    PatternTableData(std::vector<int> pattern_val, uint64_t pc_val) :
        pattern(pattern_val), pc(pc_val) {}
    PatternTableData(std::vector<int> pattern_val, uint64_t pc_val, bool con_val) :
        pattern(pattern_val), pc(pc_val), con(con_val) {}
};

class PatternTable : public PT_TYPE<PatternTableData> {
    typedef PT_TYPE<PatternTableData> Super;

private:
    void write_data(Entry& entry, custom_util::Table& table, int row);
    uint64_t build_key(uint64_t trigger, uint64_t second, uint64_t pc, uint64_t region_num);

public:
    std::deque<uint64_t> con_pc;
    int con_counter = 0;

    PatternTable(int size, int num_ways);

    void insert(uint64_t trigger, uint64_t second, uint64_t pc, uint64_t region_num, std::vector<bool> pattern);
    Entry* find(uint64_t trigger, uint64_t second, uint64_t pc, uint64_t region_num);

    std::string log();
};

// ------------------------- Prefetch Buffer ------------------------- //
struct PrefetchBufferData {
public:
    std::vector<int> pattern;
    uint64_t trigger;
    uint64_t second;
    std::vector<int> pf_metadata;
};

class PrefetchBuffer : public PB_TYPE<PrefetchBufferData> {
    typedef PB_TYPE<PrefetchBufferData> Super;

private:
    int pattern_len;

    void write_data(Entry& entry, custom_util::Table& table, int row);
    uint64_t build_key(uint64_t region_num);

public:
    bool warmup;

    PrefetchBuffer(int size, int pattern_len, int debug_level, int num_ways);

    void insert(uint64_t region_num, std::vector<int> pattern, uint64_t trigger, uint64_t second, uint32_t pf_metadata);
    void prefetch(CACHE* cache, uint64_t block_num);

    std::string log();
};

// ------------------------- Gaze Prefetcher ------------------------- //
class Gaze {
private:
    int cpu;
    int stride_pf_degree = 4;

    FilterTable ft;
    AccumulateTable at;
    PatternTable pt;
    PrefetchBuffer pb;

    PatternTable::Entry* find_in_pt(uint64_t trigger, uint64_t second, uint64_t pc, uint64_t region_num);
    void insert_in_pt(const AccumulateTable::Entry& entry, uint64_t region_num);

public:
    int global_level = 0;
    bool warmup;

    Gaze(int ft_size, int ft_ways, int at_size, int at_ways, int pt_size, int pt_ways, int pb_size, int pb_ways, int cpu);
    Gaze();
    void set_warmup(bool warmup);

    void access(uint64_t block_num, uint64_t ip, CACHE* cache);
    void eviction(uint64_t block_num);
    void prefetch(CACHE* cache, uint64_t block_num);

    void stride_prefetch(uint64_t block_num);
    void pattern_prefetch(uint64_t trigger, uint64_t second);

    void tune_stride_degree(CACHE* cache);

    void log();
};

} // namespace gaze_impl

// Wrapper struct for ChampSim interface
struct gaze : public champsim::modules::prefetcher {
    gaze_impl::Gaze gaze_impl;

    using champsim::modules::prefetcher::prefetcher;

    void prefetcher_initialize();
    uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
    uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
    void prefetcher_cycle_operate();
    void prefetcher_final_stats();
};

#endif