#ifndef PREFETCHER_BINGO_PLUS_H
#define PREFETCHER_BINGO_PLUS_H

#include "cache.h"
#include "modules.h"

#include <bits/stdc++.h>


class bingo_plus_module;

namespace Bingo_plus{

class Table {
  public:
    Table(int width_, int height_);
    void set_row(int row, const std::vector<std::string> &data, int start_col = 0);
    void set_col(int col, const std::vector<std::string> &data, int start_row = 0);
    void set_cell(int row, int col, std::string data);
    void set_cell(int row, int col, double data);
    void set_cell(int row, int col, int64_t data);
    void set_cell(int row, int col, int data);
    void set_cell(int row, int col, uint64_t data);

    /**
     * @return The entire table as a string
     */
    std::string to_string();
    std::string data_row(int row, const std::vector<int> &widths);
    static std::string top_line(const std::vector<int> &widths);
    static std::string mid_line(const std::vector<int> &widths);
    static std::string bot_line(const std::vector<int> &widths);
    static std::string line(const std::vector<int> &widths, std::string left, std::string mid, std::string right);

  private:
    unsigned width;
    unsigned height;
    std::vector<std::vector<std::string>> cells;
};

template <class T>
class Entry {
    public:
    uint64_t key;
    uint64_t index;
    uint64_t tag;
    bool valid;
    T data;
};

template <class T> class SetAssociativeCache {
  public:
    SetAssociativeCache(int size_, int num_ways_, int debug_level_ = 0);

    /**
     * Invalidates the entry corresponding to the given key.
     * @return A pointer to the invalidated entry
     */
    Entry<T> *erase(uint64_t key);
    /**
     * @return The old state of the entry that was updated
     */
    Entry<T> insert(uint64_t key, const T &data);
    Entry<T> *find(uint64_t key);
    /**
     * Creates a table with the given headers and populates the rows by calling `write_data` on all
     * valid entries contained in the cache. This function makes it easy to visualize the contents
     * of a cache.
     * @return The constructed table as a string
     */
    std::string log(std::vector<std::string> headers);
    int get_index_len();
    void set_debug_level(int debug_level_);

  protected:
    /* should be overriden in children */
    virtual void write_data(Entry<T> &entry, Table &table, int row);
    /**
     * @return The way of the selected victim
     */
    virtual int select_victim(uint64_t index);
    std::vector<Entry<T>> get_valid_entries();

    int size;
    int num_ways;
    int num_sets;
    int index_len = 0; /* in bits */
    std::vector<std::vector<Entry<T>>> entries;
    std::vector<std::unordered_map<uint64_t, int>> cams;
    int debug_level = 0;
};

template <class T> class LRUSetAssociativeCache : public SetAssociativeCache<T> {
    typedef SetAssociativeCache<T> Super;

  public:
    LRUSetAssociativeCache(int size_, int num_ways_, int debug_level_ = 0);
    void set_mru(uint64_t key);
    void set_lru(uint64_t key);

  protected:
    /* @override */
    int select_victim(uint64_t index);
    uint64_t *get_lru(uint64_t key);
    std::vector<std::vector<uint64_t>> lru;
    uint64_t t = 1;
};

class FilterTableData {
  public:
    uint64_t pc;
    int offset;
};

class FilterTable : public LRUSetAssociativeCache<FilterTableData> {
    typedef LRUSetAssociativeCache<FilterTableData> Super;

  public:
    FilterTable(int size_, int debug_level_ = 0, int num_ways_ = 16);
    Entry<FilterTableData> *find(uint64_t region_number);
    void insert(uint64_t region_number, uint64_t pc, int offset);
    Entry<FilterTableData> *erase(uint64_t region_number);
    std::string log();

  private:
    /* @override */
    void write_data(Entry<FilterTableData> &entry, Table &table, int row);

    uint64_t build_key(uint64_t region_number);

    /*==========================================================*/
    /* Entry   = [tag, offset, PC, valid, LRU]                  */
    /* Storage = size * (37 - lg(sets) + 5 + 16 + 1 + lg(ways)) */
    /* 64 * (37 - lg(4) + 5 + 16 + 1 + lg(16)) = 488 Bytes      */
    /*==========================================================*/
};

class AccumulationTableData {
  public:
    uint64_t pc;
    int offset;
    std::vector<bool> pattern;
};

class AccumulationTable : public LRUSetAssociativeCache<AccumulationTableData> {
    typedef LRUSetAssociativeCache<AccumulationTableData> Super;

  public:
    AccumulationTable(int size_, int pattern_len_, int debug_level_ = 0, int num_ways_ = 16);
    /**
     * @return False if the tag wasn't found and true if the pattern bit was successfully set
     */
    bool set_pattern(uint64_t region_number, int offset);
    /* NOTE: `region_number` is probably truncated since it comes from the filter table */
    Entry<AccumulationTableData> insert(uint64_t region_number, uint64_t pc, int offset);
    Entry<AccumulationTableData> *erase(uint64_t region_number);
    std::string log();

  private:
    /* @override */
    void write_data(Entry<AccumulationTableData> &entry, Table &table, int row);
    uint64_t build_key(uint64_t region_number);
    int pattern_len;

    /*===============================================================*/
    /* Entry   = [tag, map, offset, PC, valid, LRU]                  */
    /* Storage = size * (37 - lg(sets) + 32 + 5 + 16 + 1 + lg(ways)) */
    /* 128 * (37 - lg(8) + 32 + 5 + 16 + 1 + lg(16)) = 1472 Bytes    */
    /*===============================================================*/
};

/**
 * There are 3 possible outcomes (here called `Event`) for a PHT lookup:
 * PC+Address hit, PC+Offset hit(s), or Miss.
 * NOTE: `Event` is only used for gathering stats.
 */
enum Event { PC_ADDRESS = 0, PC_OFFSET = 1, MISS = 2 };

class PatternHistoryTableData {
  public:
    std::vector<bool> pattern;
};

class PatternHistoryTable : public LRUSetAssociativeCache<PatternHistoryTableData> {
    typedef LRUSetAssociativeCache<PatternHistoryTableData> Super;

  public:
    PatternHistoryTable(int size_, int pattern_len_, int min_addr_width_, int max_addr_width_, int pc_width_,
        int debug_level_ = 0, int num_ways_ = 15);

    /* NOTE: In BINGO, address is actually block number. */
    void insert(uint64_t pc, uint64_t address, std::vector<bool> pattern);

    /**
     * First searches for a PC+Address match. If no match is found, returns all PC+Offset matches.
     * @return All un-rotated patterns if matches were found, returns an empty vector otherwise
     */
    std::vector<std::vector<bool>> find(uint64_t pc, uint64_t address);
    Event get_last_event();
    std::string log();
  private:
    /* @override */
    void write_data(Entry<PatternHistoryTableData> &entry, Table &table, int row);
    uint64_t build_key(uint64_t pc, uint64_t address);
    int pattern_len;
    int min_addr_width, max_addr_width, pc_width;
    Event last_event;

    /*======================================================*/
    /* Entry   = [tag, map, valid, LRU]                     */
    /* Storage = size * (32 - lg(sets) + 32 + 1 + lg(ways)) */
    /* 8K * (32 - lg(512) + 32 + 1 + lg(16)) = 60K Bytes    */
    /*======================================================*/

    //FOR DPC4 we cut this down to 15 ways to save state
};

class PrefetchStreamerData {
  public:
    /* contains the prefetch fill level for each block of spatial region */
    std::vector<bool> pattern;
};

class PrefetchStreamer : public LRUSetAssociativeCache<PrefetchStreamerData> {
    typedef LRUSetAssociativeCache<PrefetchStreamerData> Super;
  public:
    PrefetchStreamer(int size_, int pattern_len_, int debug_level_ = 0, int num_ways_ = 16);
    void insert(uint64_t region_number, std::vector<bool> pattern);
    int prefetch(bingo_plus_module *b, uint64_t block_address);
    std::string log();

  private:
    /* @override */
    void write_data(Entry<PrefetchStreamerData> &entry, Table &table, int row);
    uint64_t build_key(uint64_t region_number);
    int pattern_len;

    /*======================================================*/
    /* Entry   = [tag, map, valid, LRU]                     */
    /* Storage = size * (53 - lg(sets) + 64 + 1 + lg(ways)) */
    /* 128 * (53 - lg(8) + 64 + 1 + lg(16)) = 1904 Bytes    */
    /*======================================================*/
};
}

class bingo_plus_module {
  public:
    /*=== Bingo Settings ===*/
    static constexpr int DEBUG_LEVEL = 1;
    static constexpr int REGION_SIZE = 2 * 1024;  /* size of spatial region = 2KB */
    static constexpr int PC_WIDTH = 16;           /* number of PC bits used in PHT */
    static constexpr int MIN_ADDR_WIDTH = 5;      /* number of Address bits used for PC+Offset matching */
    static constexpr int MAX_ADDR_WIDTH = 16;     /* number of Address bits used for PC+Address matching */
    static constexpr int FT_SIZE = 64;            /* size of filter table */
    static constexpr int AT_SIZE = 128;           /* size of accumulation table */
    static constexpr int PHT_SIZE = 512 * 15;     /* size of pattern history table (PHT) */
    static constexpr int PHT_WAYS = 15;           /* associativity of PHT */
    static constexpr int PF_STREAMER_SIZE = 128;  /* size of prefetch streamer */
    /**
     * Updates BINGO's state based on the most recent LOAD access.
     * @param block_number The block address of the most recent LOAD access
     * @param pc           The PC of the most recent LOAD access
     */

     //Added for DPC4
     static constexpr uint32_t BINGO_PROPORTION = 1;
     static constexpr uint32_t BINGO_LLC_EVICT = 2;
     static constexpr bool DO_PREFETCH_TRAIN = false;
     static constexpr bool DO_PREFETCH_PREFETCH = false;


    void access(uint64_t block_number, uint64_t pc);
    void eviction(uint64_t block_number);
    int prefetch(uint64_t block_number);
    void set_debug_level(int debug_level_);
    void log();

    CACHE* intern_;
    champsim::modules::prefetcher* module_;
    int cpu;


    /*========== stats ==========*/
    /* NOTE: the BINGO code submitted for DPC3 (this code) does not call any of these methods. */

    Bingo_plus::Event get_event(uint64_t block_number);
    void add_prefetch(uint64_t block_number);
    void add_useful(uint64_t block_number, Bingo_plus::Event ev);
    void add_useless(uint64_t block_number, Bingo_plus::Event ev);
    void reset_stats();
    void print_stats();

  private:
    /**
     * Performs a PHT lookup and computes a prefetching pattern from the result.
     * @return The appropriate prefetch level for all blocks based on PHT output or an empty vector
     *         if no blocks should be prefetched
     */
    std::vector<bool> find_in_pht(uint64_t pc, uint64_t address);

    void insert_in_pht(const Bingo_plus::Entry<Bingo_plus::AccumulationTableData> &entry);

    /**
     * Uses a voting mechanism to produce a prefetching pattern from a set of footprints.
     * @param x The patterns obtained from all PC+Offset matches
     * @return  The appropriate prefetch level for all blocks based on BINGO's voting thresholds or
     *          an empty vector if no blocks should be prefetched
     */
    std::vector<bool> vote(const std::vector<std::vector<bool>> &x);

    /*=== Bingo Settings ===*/
    /* voting thresholds */
    const double L1D_THRESH = 0.75;
    const double L2C_THRESH = 0.25;
    const double LLC_THRESH = 0.25; /* off */
    
    /* PC+Address matches are filled into L1 */
    //const int PC_ADDRESS_FILL_LEVEL = FILL_L1;
    /*======================*/

    int pattern_len = REGION_SIZE >> LOG2_BLOCK_SIZE;
    Bingo_plus::FilterTable filter_table{FT_SIZE,DEBUG_LEVEL};
    Bingo_plus::AccumulationTable accumulation_table{AT_SIZE, REGION_SIZE >> LOG2_BLOCK_SIZE, DEBUG_LEVEL};
    Bingo_plus::PatternHistoryTable pht{PHT_SIZE,REGION_SIZE >> LOG2_BLOCK_SIZE, MIN_ADDR_WIDTH, MAX_ADDR_WIDTH, PC_WIDTH, DEBUG_LEVEL, PHT_WAYS};
    Bingo_plus::PrefetchStreamer pf_streamer{PF_STREAMER_SIZE, REGION_SIZE >> LOG2_BLOCK_SIZE, DEBUG_LEVEL};
    int debug_level = DEBUG_LEVEL;

    /* stats */
    std::unordered_map<uint64_t, Bingo_plus::Event> pht_events;

    uint64_t pht_access_cnt = 0;
    uint64_t pht_pc_address_cnt = 0;
    uint64_t pht_pc_offset_cnt = 0;
    uint64_t pht_miss_cnt = 0;

    uint64_t prefetch_cnt[2] = {0};
    uint64_t useful_cnt[2] = {0};
    uint64_t useless_cnt[2] = {0};

    std::unordered_map<int, uint64_t> pref_level_cnt;
    uint64_t region_pref_cnt = 0;

    uint64_t vote_cnt = 0;
    uint64_t voter_sum = 0;
    uint64_t voter_sqr_sum = 0;

    public:
      bingo_plus_module(CACHE* intern, champsim::modules::prefetcher* module__, int cpu_) : intern_(intern), module_(module__), cpu(cpu_) {}
};

class bingo_plus : public champsim::modules::prefetcher {
    std::vector<bingo_plus_module> prefetchers;
    int pattern_len = bingo_plus_module::REGION_SIZE >> LOG2_BLOCK_SIZE;

    static constexpr uint32_t PORP_SAMPLE = 2048;
    std::vector<uint32_t> core_proportion_counter;
    std::vector<uint32_t> core_proportion_index;

    public:
        using prefetcher::prefetcher;
        uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
        uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, bool prefetch, champsim::address evicted_addr, uint32_t metadata_in);

        void prefetcher_initialize();
        void prefetcher_cycle_operate();
        void prefetcher_final_stats();

};
#endif