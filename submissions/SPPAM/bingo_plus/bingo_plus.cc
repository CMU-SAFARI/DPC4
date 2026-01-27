/* Bingo [https://mshakerinava.github.io/papers/bingo-hpca19.pdf] */
#include "bingo_plus.h"
#include "../berti_plus/berti_plus_parameters.h"

using namespace Bingo_plus;

/**
 * A very simple and efficient hash function that:
 * 1) Splits key into blocks of length `index_len` bits and computes the XOR of all blocks.
 * 2) Replaces the least significant block of key with computed block.
 * With this hash function, the index will depend on all bits in the key. As a consequence, entries
 * will be more randomly distributed among the sets.
 * NOTE: Applying this hash function twice with the same `index_len` acts as the identity function.
 */ 
uint64_t hash_index_plus(uint64_t key, int index_len) {
    if (index_len == 0)
        return key;
    for (uint64_t tag = (key >> index_len); tag > 0; tag >>= index_len)
        key ^= tag & ((1 << index_len) - 1);
    return key;
}

/**
 * A class for printing beautiful data tables.
 * It's useful for logging the information contained in tabular structures.
 */
Table::Table(int width_, int height_) : width(width_), height(height_), cells(height_, std::vector<std::string>(width_)) {}

void Table::set_row(int row, const std::vector<std::string> &data, int start_col) {
    // assert(data.size() + start_col == this->width);
    for (unsigned col = start_col; col < this->width; col += 1)
        this->set_cell(row, col, data[col]);
}

void Table::set_col(int col, const std::vector<std::string> &data, int start_row) {
    // assert(data.size() + start_row == this->height);
    for (unsigned row = start_row; row < this->height; row += 1)
        this->set_cell(row, col, data[row]);
}

void Table::set_cell(int row, int col, std::string data) {
    // assert(0 <= row && row < (int)this->height);
    // assert(0 <= col && col < (int)this->width);
    this->cells[row][col] = data;
}

void Table::set_cell(int row, int col, double data) {
    std::ostringstream oss;
    oss << std::setw(11) << std::fixed << std::setprecision(8) << data;
    this->set_cell(row, col, oss.str());
}

void Table::set_cell(int row, int col, int64_t data) {
    std::ostringstream oss;
    oss << std::setw(11) << std::left << data;
    this->set_cell(row, col, oss.str());
}

void Table::set_cell(int row, int col, int data) { this->set_cell(row, col, (int64_t)data); }

void Table::set_cell(int row, int col, uint64_t data) {
    std::ostringstream oss;
    oss << "0x" << std::setfill('0') << std::setw(16) << std::hex << data;
    this->set_cell(row, col, oss.str());
}
std::string Table::to_string() {
    std::vector<int> widths;
    for (unsigned i = 0; i < this->width; i += 1) {
        int max_width = 0;
        for (unsigned j = 0; j < this->height; j += 1)
            max_width = std::max(max_width, (int)this->cells[j][i].size());
        widths.push_back(max_width + 2);
    }
    std::string out;
    out += Table::top_line(widths);
    out += this->data_row(0, widths);
    for (unsigned i = 1; i < this->height; i += 1) {
        out += Table::mid_line(widths);
        out += this->data_row(i, widths);
    }
    out += Table::bot_line(widths);
    return out;
}

std::string Table::data_row(int row, const std::vector<int> &widths) {
    std::string out;
    for (unsigned i = 0; i < this->width; i += 1) {
        std::string data = this->cells[row][i];
        data.resize(widths[i] - 2, ' ');
        out += " | " + data;
    }
    out += " |\n";
    return out;
}

std::string Table::top_line(const std::vector<int> &widths) { return Table::line(widths, "┌", "┬", "┐"); }

std::string Table::mid_line(const std::vector<int> &widths) { return Table::line(widths, "├", "┼", "┤"); }

std::string Table::bot_line(const std::vector<int> &widths) { return Table::line(widths, "└", "┴", "┘"); }

std::string Table::line(const std::vector<int> &widths, std::string left, std::string mid, std::string right) {
    std::string out = " " + left;
    for (unsigned i = 0; i < widths.size(); i += 1) {
        int w = widths[i];
        for (int j = 0; j < w; j += 1)
            out += "─";
        if (i != widths.size() - 1)
            out += mid;
        else
            out += right;
    }
    return out + "\n";
}

template <class T>
SetAssociativeCache<T>::SetAssociativeCache(int size_, int num_ways_, int debug_level_) : size(size_), num_ways(num_ways_), num_sets(size_ / num_ways_), entries(num_sets, std::vector<Entry<T>>(num_ways_)),
    cams(num_sets), debug_level(debug_level_) {
    // assert(size % num_ways == 0);
    for (int i = 0; i < num_sets; i += 1)
        for (int j = 0; j < num_ways; j += 1)
            entries[i][j].valid = false;
    /* calculate `index_len` (number of bits required to store the index) */
    for (int max_index = num_sets - 1; max_index > 0; max_index >>= 1)
        this->index_len += 1;
}

    /**
     * Invalidates the entry corresponding to the given key.
     * @return A pointer to the invalidated entry
     */
template <class T>
Entry<T>* SetAssociativeCache<T>::erase(uint64_t key) {
    Entry<T> *entry = this->find(key);
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    auto &cam = cams[index];
    //int num_erased = cam.erase(tag);
    if (entry)
        entry->valid = false;
    // assert(entry ? num_erased == 1 : num_erased == 0);
    return entry;
}

    /**
     * @return The old state of the entry that was updated
     */
template <class T>
Entry<T> SetAssociativeCache<T>::insert(uint64_t key, const T &data) {
    Entry<T> *entry = this->find(key);
    if (entry != nullptr) {
        Entry<T> old_entry = *entry;
        entry->data = data;
        return old_entry;
    }
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    std::vector<Entry<T>> &set = this->entries[index];
    int victim_way = -1;
    for (int i = 0; i < this->num_ways; i += 1)
        if (!set[i].valid) {
            victim_way = i;
            break;
        }
    if (victim_way == -1) {
        victim_way = this->select_victim(index);
    }
    Entry<T> &victim = set[victim_way];
    Entry<T> old_entry = victim;
    victim = {key, index, tag, true, data};
    auto &cam = cams[index];
    if (old_entry.valid) {
        //int num_erased = cam.erase(old_entry.tag);
        // assert(num_erased == 1);
    }
    cam[tag] = victim_way;
    return old_entry;
}
template <class T>
Entry<T> *SetAssociativeCache<T>::find(uint64_t key) {
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    auto &cam = cams[index];
    if (cam.find(tag) == cam.end())
        return nullptr;
    int way = cam[tag];
    Entry<T> &entry = this->entries[index][way];
    // assert(entry.tag == tag && entry.valid);
    return &entry;
}

/**
 * Creates a table with the given headers and populates the rows by calling `write_data` on all
 * valid entries contained in the cache. This function makes it easy to visualize the contents
 * of a cache.
 * @return The constructed table as a string
 */
template <class T>
std::string SetAssociativeCache<T>::log(std::vector<std::string> headers) {
    std::vector<Entry<T>> valid_entries = this->get_valid_entries();
    Table table(headers.size(), valid_entries.size() + 1);
    table.set_row(0, headers);
    for (unsigned i = 0; i < valid_entries.size(); i += 1)
        this->write_data(valid_entries[i], table, i + 1);
    return table.to_string();
}

template <class T>
int SetAssociativeCache<T>::get_index_len() { return this->index_len; }

template <class T>
void SetAssociativeCache<T>::set_debug_level(int debug_level_) { this->debug_level = debug_level_; }

    /* should be overriden in children */
template <class T>
void SetAssociativeCache<T>::write_data(Entry<T> &entry, Table &table, int row) {}

/**
 * @return The way of the selected victim
 */
template <class T>
int SetAssociativeCache<T>::select_victim(uint64_t index) {
    /* random eviction policy if not overriden */
    return rand() % this->num_ways;
}

template <class T>
std::vector<Entry<T>> SetAssociativeCache<T>::get_valid_entries() {
    std::vector<Entry<T>> valid_entries;
    for (int i = 0; i < num_sets; i += 1)
        for (int j = 0; j < num_ways; j += 1)
            if (entries[i][j].valid)
                valid_entries.push_back(entries[i][j]);
    return valid_entries;
}

template <class T>
LRUSetAssociativeCache<T>::LRUSetAssociativeCache(int size_, int num_ways_, int debug_level_)
    : Super(size_, num_ways_, debug_level_), lru(this->num_sets, std::vector<uint64_t>(num_ways_)) {}

template <class T>
void LRUSetAssociativeCache<T>::set_mru(uint64_t key) { *this->get_lru(key) = this->t++; }

template <class T>
void LRUSetAssociativeCache<T>::set_lru(uint64_t key) { *this->get_lru(key) = 0; }

template <class T>
int LRUSetAssociativeCache<T>::select_victim(uint64_t index) {
    std::vector<uint64_t> &lru_set = this->lru[index];
    return std::min_element(lru_set.begin(), lru_set.end()) - lru_set.begin();
}

template <class T>
uint64_t *LRUSetAssociativeCache<T>::get_lru(uint64_t key) {
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    // assert(this->cams[index].count(tag) == 1);
    int way = this->cams[index][tag];
    return &this->lru[index][way];
}
/*=== End Of Cache Framework ===*/




FilterTable::FilterTable(int size_, int debug_level_, int num_ways_) : Super(size_, num_ways_, debug_level_) {
    // assert(__builtin_popcount(size) == 1);
    if (this->debug_level >= 1)
        std::cerr << "FilterTable::FilterTable(size=" << size << ", debug_level=" << debug_level
                << ", num_ways=" << num_ways << ")" << std::dec << std::endl;
}

Entry<FilterTableData> *FilterTable::find(uint64_t region_number) {
    if (this->debug_level >= 2)
        std::cerr << "FilterTable::find(region_number=0x" << std::hex << region_number << ")" << std::dec << std::endl;
    uint64_t key = this->build_key(region_number);
    Entry<FilterTableData> *entry = Super::find(key);
    if (!entry) {
        if (this->debug_level >= 2)
            std::cerr << "[FilterTable::find] Miss!" << std::dec << std::endl;
        return nullptr;
    }
    if (this->debug_level >= 2)
        std::cerr << "[FilterTable::find] Hit!" << std::dec << std::endl;
    Super::set_mru(key);
    return entry;
}

void FilterTable::insert(uint64_t region_number, uint64_t pc, int offset) {
    if (this->debug_level >= 2)
        std::cerr << "FilterTable::insert(region_number=0x" << std::hex << region_number << ", pc=0x" << pc
                << ", offset=" << std::dec << offset << ")" << std::dec << std::endl;
    uint64_t key = this->build_key(region_number);
    // assert(!Super::find(key));
    Super::insert(key, {pc, offset});
    Super::set_mru(key);
}

Entry<FilterTableData> *FilterTable::erase(uint64_t region_number) {
    uint64_t key = this->build_key(region_number);
    return Super::erase(key);
}

std::string FilterTable::log() {
    std::vector<std::string> headers({"Region", "PC", "Offset"});
    return Super::log(headers);
}

/* @override */
void FilterTable::write_data(Entry<FilterTableData> &entry, Table &table, int row) {
    uint64_t key = hash_index_plus(entry.key, this->index_len);
    table.set_cell(row, 0, key);
    table.set_cell(row, 1, entry.data.pc);
    table.set_cell(row, 2, entry.data.offset);
}

uint64_t FilterTable::build_key(uint64_t region_number) {
    uint64_t key = region_number & ((1ULL << 37) - 1);
    return hash_index_plus(key, this->index_len);
}

template <class T> std::string pattern_to_string(const std::vector<T> &pattern) {
    std::ostringstream oss;
    for (unsigned i = 0; i < pattern.size(); i += 1)
        oss << int(pattern[i]);
    return oss.str();
}

AccumulationTable::AccumulationTable(int size_, int pattern_len_, int debug_level_, int num_ways_)
    : Super(size_, num_ways_, debug_level_), pattern_len(pattern_len_) {
    // assert(__builtin_popcount(size) == 1);
    // assert(__builtin_popcount(pattern_len) == 1);
    if (this->debug_level >= 1)
        std::cerr << "AccumulationTable::AccumulationTable(size=" << size << ", pattern_len=" << pattern_len
                << ", debug_level=" << debug_level << ", num_ways=" << num_ways << ")" << std::dec << std::endl;
}

/**
 * @return False if the tag wasn't found and true if the pattern bit was successfully set
 */
bool AccumulationTable::set_pattern(uint64_t region_number, int offset) {
    if (this->debug_level >= 2)
        std::cerr << "AccumulationTable::set_pattern(region_number=0x" << std::hex << region_number << ", offset=" << std::dec
                << offset << ")" << std::dec << std::endl;
    uint64_t key = this->build_key(region_number);
    Entry<AccumulationTableData> *entry = Super::find(key);
    if (!entry) {
        if (this->debug_level >= 2)
            std::cerr << "[AccumulationTable::set_pattern] Not found!" << std::dec << std::endl;
        return false;
    }
    entry->data.pattern[offset] = true;
    Super::set_mru(key);
    if (this->debug_level >= 2)
        std::cerr << "[AccumulationTable::set_pattern] OK!" << std::dec << std::endl;
    return true;
}

/* NOTE: `region_number` is probably truncated since it comes from the filter table */
Entry<AccumulationTableData> AccumulationTable::insert(uint64_t region_number, uint64_t pc, int offset) {
    if (this->debug_level >= 2)
        std::cerr << "AccumulationTable::insert(region_number=0x" << std::hex << region_number << ", pc=0x" << pc
                << ", offset=" << std::dec << offset << std::dec << std::endl;
    uint64_t key = this->build_key(region_number);
    // assert(!Super::find(key));
    std::vector<bool> pattern(this->pattern_len, false);
    pattern[offset] = true;
    Entry<AccumulationTableData> old_entry = Super::insert(key, {pc, offset, pattern});
    Super::set_mru(key);
    return old_entry;
}

Entry<AccumulationTableData> *AccumulationTable::erase(uint64_t region_number) {
    uint64_t key = this->build_key(region_number);
    return Super::erase(key);
}

std::string AccumulationTable::log() {
    std::vector<std::string> headers({"Region", "PC", "Offset", "Pattern"});
    return Super::log(headers);
}

    /* @override */
void AccumulationTable::write_data(Entry<AccumulationTableData> &entry, Table &table, int row) {
    uint64_t key = hash_index_plus(entry.key, this->index_len);
    table.set_cell(row, 0, key);
    table.set_cell(row, 1, entry.data.pc);
    table.set_cell(row, 2, entry.data.offset);
    table.set_cell(row, 3, pattern_to_string(entry.data.pattern));
}

uint64_t AccumulationTable::build_key(uint64_t region_number) {
    uint64_t key = region_number & ((1ULL << 37) - 1);
    return hash_index_plus(key, this->index_len);
}

template <class T> std::vector<T> my_rotate(const std::vector<T> &x, int n) {
    std::vector<T> y;
    int len = x.size();
    n = n % len;
    for (int i = 0; i < len; i += 1)
        y.push_back(x[(i - n + len) % len]);
    return y;
}


PatternHistoryTable::PatternHistoryTable(int size_, int pattern_len_, int min_addr_width_, int max_addr_width_, int pc_width_,
    int debug_level_, int num_ways_)
    : Super(size_, num_ways_, debug_level_), pattern_len(pattern_len_), min_addr_width(min_addr_width_),
        max_addr_width(max_addr_width_), pc_width(pc_width_) {
    // assert(this->pc_width >= 0);
    // assert(this->min_addr_width >= 0);
    // assert(this->max_addr_width >= 0);
    // assert(this->max_addr_width >= this->min_addr_width);
    // assert(this->pc_width + this->min_addr_width > 0);
    // assert(__builtin_popcount(pattern_len) == 1);
    if (this->debug_level >= 1)
        std::cerr << "PatternHistoryTable::PatternHistoryTable(size=" << size << ", pattern_len=" << pattern_len
                << ", min_addr_width=" << min_addr_width << ", max_addr_width=" << max_addr_width
                << ", pc_width=" << pc_width << ", debug_level=" << debug_level << ", num_ways=" << num_ways << ")"
                << std::dec << std::endl;
}

    /* NOTE: In BINGO, address is actually block number. */
void PatternHistoryTable::insert(uint64_t pc, uint64_t address, std::vector<bool> pattern) {
    if (this->debug_level >= 2)
        std::cerr << "PatternHistoryTable::insert(pc=0x" << std::hex << pc << ", address=0x" << address
                << ", pattern=" << pattern_to_string(pattern) << ")" << std::dec << std::endl;
    // assert((int)pattern.size() == this->pattern_len);
    int offset = (int)(address % this->pattern_len);
    pattern = my_rotate(pattern, -offset);
    uint64_t key = this->build_key(pc, address);
    Super::insert(key, {pattern});
    Super::set_mru(key);
}

/**
 * First searches for a PC+Address match. If no match is found, returns all PC+Offset matches.
 * @return All un-rotated patterns if matches were found, returns an empty vector otherwise
 */
std::vector<std::vector<bool>> PatternHistoryTable::find(uint64_t pc, uint64_t address) {
    if (this->debug_level >= 2)
        std::cerr << "PatternHistoryTable::find(pc=0x" << std::hex << pc << ", address=0x" << address << ")" << std::dec << std::endl;
    uint64_t key = this->build_key(pc, address);
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    auto &set = this->entries[index];
    uint64_t min_tag_mask = (1 << (this->pc_width + this->min_addr_width - this->index_len)) - 1;
    uint64_t max_tag_mask = (1 << (this->pc_width + this->max_addr_width - this->index_len)) - 1;
    std::vector<std::vector<bool>> matches;
    this->last_event = MISS;
    for (int i = 0; i < this->num_ways; i += 1) {
        if (!set[i].valid)
            continue;
        bool min_match = ((set[i].tag & min_tag_mask) == (tag & min_tag_mask));
        bool max_match = ((set[i].tag & max_tag_mask) == (tag & max_tag_mask));
        std::vector<bool> &cur_pattern = set[i].data.pattern;
        if (max_match) {
            this->last_event = PC_ADDRESS;
            Super::set_mru(set[i].key);
            matches.clear();
            matches.push_back(cur_pattern);
            break;
        }
        if (min_match) {
            this->last_event = PC_OFFSET;
            matches.push_back(cur_pattern);
        }
    }
    int offset = (int)(address % this->pattern_len);
    for (int i = 0; i < (int)matches.size(); i += 1)
        matches[i] = my_rotate(matches[i], +offset);
    return matches;
}

Event PatternHistoryTable::get_last_event() { return this->last_event; }

std::string PatternHistoryTable::log() {
    std::vector<std::string> headers({"PC", "Offset", "Address", "Pattern"});
    return Super::log(headers);
}


    /* @override */
void PatternHistoryTable::write_data(Entry<PatternHistoryTableData> &entry, Table &table, int row) {
    uint64_t base_key = entry.key >> (this->pc_width + this->min_addr_width);
    uint64_t index_key = entry.key & ((1 << (this->pc_width + this->min_addr_width)) - 1);
    index_key = hash_index_plus(index_key, this->index_len); /* unhash */
    uint64_t key = (base_key << (this->pc_width + this->min_addr_width)) | index_key;

    /* extract PC, offset, and address */
    uint64_t offset = key & ((1 << this->min_addr_width) - 1);
    key >>= this->min_addr_width;
    uint64_t pc = key & ((1 << this->pc_width) - 1);
    key >>= this->pc_width;
    uint64_t address = (key << this->min_addr_width) + offset;

    table.set_cell(row, 0, pc);
    table.set_cell(row, 1, offset);
    table.set_cell(row, 2, address);
    table.set_cell(row, 3, pattern_to_string(entry.data.pattern));
}

uint64_t PatternHistoryTable::build_key(uint64_t pc, uint64_t address) {
    pc &= (1 << this->pc_width) - 1;            /* use `pc_width` bits from pc */
    address &= (1 << this->max_addr_width) - 1; /* use `addr_width` bits from address */
    uint64_t offset = address & ((1 << this->min_addr_width) - 1);
    uint64_t base = (address >> this->min_addr_width);
    /* key = base + hash_index( pc + offset )
        * The index must be computed from only PC+Offset to ensure that all entries with the same
        * PC+Offset end up in the same set */
    uint64_t index_key = hash_index_plus((pc << this->min_addr_width) | offset, this->index_len);
    uint64_t key = (base << (this->pc_width + this->min_addr_width)) | index_key;
    return key;
}

PrefetchStreamer::PrefetchStreamer(int size_, int pattern_len_, int debug_level_, int num_ways_)
    : Super(size_, num_ways_, debug_level_), pattern_len(pattern_len_) {
    if (this->debug_level >= 1)
        std::cerr << "PrefetchStreamer::PrefetchStreamer(size=" << size << ", pattern_len=" << pattern_len
                << ", debug_level=" << debug_level << ", num_ways=" << num_ways << ")" << std::dec << std::endl;
}

void PrefetchStreamer::insert(uint64_t region_number, std::vector<bool> pattern) {
    if (this->debug_level >= 2)
        std::cerr << "PrefetchStreamer::insert(region_number=0x" << std::hex << region_number
                << ", pattern=" << pattern_to_string(pattern) << ")" << std::dec << std::endl;
    uint64_t key = this->build_key(region_number);
    Super::insert(key, {pattern});
    Super::set_mru(key);
}

int PrefetchStreamer::prefetch(bingo_plus_module *b, uint64_t block_address) {
    if (this->debug_level >= 2) {
        std::cerr << "PrefetchStreamer::prefetch(cache=" << b->intern_->NAME << ", block_address=0x" << std::hex << block_address
                << ")" << std::dec << std::endl;
        std::cerr << "[PrefetchStreamer::prefetch] " << b->intern_->get_pq_occupancy().back() << "/" << b->intern_->get_pq_size().back()
                << " PQ entries occupied." << std::dec << std::endl;
        std::cerr << "[PrefetchStreamer::prefetch] " << b->intern_->get_mshr_occupancy() << "/" << b->intern_->get_mshr_size()
                << " MSHR entries occupied." << std::dec << std::endl;
    }
    uint64_t base_addr = block_address << LOG2_BLOCK_SIZE;
    int region_offset = (int)(block_address % this->pattern_len);
    uint64_t region_number = block_address / this->pattern_len;
    uint64_t key = this->build_key(region_number);
    Entry<PrefetchStreamerData> *entry = Super::find(key);
    if (!entry) {
        if (this->debug_level >= 2)
            std::cerr << "[PrefetchStreamer::prefetch] No entry found." << std::dec << std::endl;
        return 0;
    }
    Super::set_mru(key);
    int pf_issued = 0;
    auto &pattern = entry->data.pattern;
    pattern[region_offset] = 0; /* accessed block will be automatically fetched if necessary (miss) */
    int pf_offset;
    /* prefetch blocks that are close to the recent access first (locality!) */
    for (int d = 1; d < this->pattern_len; d += 1) {
        /* prefer positive strides */
        for (int sgn = +1; sgn >= -1; sgn -= 2) {
            pf_offset = region_offset + sgn * d;
            if (0 <= pf_offset && pf_offset < this->pattern_len && pattern[pf_offset] > 0) {
                uint64_t pf_address = (region_number * this->pattern_len + pf_offset) << LOG2_BLOCK_SIZE;
                if (b->intern_->get_pq_occupancy().back() + b->intern_->get_mshr_occupancy() < b->intern_->get_mshr_size() - 1 &&
                    b->intern_->get_pq_occupancy().back() < b->intern_->get_pq_size().back()) {
                    int ok = b->module_->prefetch_line(pf_address, pattern[pf_offset], 0xffffffff);
                    // assert(ok == 1);
                    pf_issued += 1;
                    pattern[pf_offset] = 0;
                } else {
                    /* prefetching limit is reached */
                    return pf_issued;
                }
            }
        }
    }
    /* all prefetches done for this spatial region */
    Super::erase(key);
    return pf_issued;
}

std::string PrefetchStreamer::log() {
    std::vector<std::string> headers({"Region", "Pattern"});
    return Super::log(headers);
}

/* @override */
void PrefetchStreamer::write_data(Entry<PrefetchStreamerData> &entry, Table &table, int row) {
    uint64_t key = hash_index_plus(entry.key, this->index_len);
    table.set_cell(row, 0, key);
    table.set_cell(row, 1, pattern_to_string(entry.data.pattern));
}

uint64_t PrefetchStreamer::build_key(uint64_t region_number) { return hash_index_plus(region_number, this->index_len); }


template <class T> inline T square(T x) { return x * x; }

void bingo_plus_module::access(uint64_t block_number, uint64_t pc) {
    if (this->debug_level >= 2)
        std::cerr << "[" << cpu << " Bingo] access(block_number=0x" << std::hex << block_number << ", pc=0x" << pc << ")" << std::dec << std::endl;
    uint64_t region_number = block_number / this->pattern_len;
    int region_offset = (int)(block_number % this->pattern_len);
    bool success = this->accumulation_table.set_pattern(region_number, region_offset);
    if (success)
        return;
    Entry<FilterTableData> *entry = this->filter_table.find(region_number);
    if (!entry) {
        /* trigger access */
        this->filter_table.insert(region_number, pc, region_offset);
        auto pattern = this->find_in_pht(pc, block_number);
        if (pattern.empty()) {
            /* nothing to prefetch */
            return;
        }
        /* give pattern to `pf_streamer` */
        // assert((int)pattern.size() == this->pattern_len);
        this->pf_streamer.insert(region_number, pattern);
        return;
    }
    if (entry->data.offset != region_offset) {
        /* move from filter table to accumulation table */
        region_number = hash_index_plus(entry->key, this->filter_table.get_index_len());
        Entry<AccumulationTableData> victim =
            this->accumulation_table.insert(region_number, entry->data.pc, entry->data.offset);
        this->accumulation_table.set_pattern(region_number, region_offset);
        this->filter_table.erase(region_number);
        if (victim.valid) {
            /* move from accumulation table to PHT */
            this->insert_in_pht(victim);
        }
    }
}

void bingo_plus_module::eviction(uint64_t block_number) {
    if (this->debug_level >= 2)
        std::cerr << "[" << cpu << " Bingo] eviction(block_number=" << block_number << ")" << std::dec << std::endl;
    /* end of generation: footprint must now be stored in PHT */
    uint64_t region_number = block_number / this->pattern_len;
    this->filter_table.erase(region_number);
    Entry<AccumulationTableData> *entry = this->accumulation_table.erase(region_number);
    if (entry) {
        /* move from accumulation table to PHT */
        this->insert_in_pht(*entry);
    }
}

int bingo_plus_module::prefetch(uint64_t block_number) {
    if (this->debug_level >= 2)
        std::cerr << "Bingo::prefetch(cache=" << intern_->NAME << ", block_number=" << std::hex << block_number << ")" << std::dec
                << std::endl;
    int pf_issued = this->pf_streamer.prefetch(this, block_number);
    if (this->debug_level >= 2)
        std::cerr << "[Bingo::prefetch] pf_issued=" << pf_issued << std::dec << std::endl;
    return pf_issued;
}

void bingo_plus_module::set_debug_level(int debug_level_) {
    this->filter_table.set_debug_level(debug_level_);
    this->accumulation_table.set_debug_level(debug_level_);
    this->pht.set_debug_level(debug_level_);
    this->pf_streamer.set_debug_level(debug_level_);
    this->debug_level = debug_level_;
}

void bingo_plus_module::log() {
    std::cerr << "Filter Table:" << std::dec << std::endl;
    std::cerr << this->filter_table.log();

    std::cerr << "Accumulation Table:" << std::dec << std::endl;
    std::cerr << this->accumulation_table.log();

    std::cerr << "Pattern History Table:" << std::dec << std::endl;
    std::cerr << this->pht.log();

    std::cerr << "Prefetch Streamer:" << std::dec << std::endl;
    std::cerr << this->pf_streamer.log();
}

/*========== stats ==========*/
/* NOTE: the BINGO code submitted for DPC3 (this code) does not call any of these methods. */

Event bingo_plus_module::get_event(uint64_t block_number) {
    uint64_t region_number = block_number / this->pattern_len;
    // assert(this->pht_events.count(region_number) == 1);
    return this->pht_events[region_number];
}

void bingo_plus_module::add_prefetch(uint64_t block_number) {
    Event ev = this->get_event(block_number);
    // assert(ev != MISS);
    this->prefetch_cnt[ev] += 1;
}

void bingo_plus_module::add_useful(uint64_t block_number, Event ev) {
    // assert(ev != MISS);
    this->useful_cnt[ev] += 1;
}

void bingo_plus_module::add_useless(uint64_t block_number, Event ev) {
    // assert(ev != MISS);
    this->useless_cnt[ev] += 1;
}

void bingo_plus_module::reset_stats() {
    this->pht_access_cnt = 0;
    this->pht_pc_address_cnt = 0;
    this->pht_pc_offset_cnt = 0;
    this->pht_miss_cnt = 0;

    for (int i = 0; i < 2; i += 1) {
        this->prefetch_cnt[i] = 0;
        this->useful_cnt[i] = 0;
        this->useless_cnt[i] = 0;
    }

    this->pref_level_cnt.clear();
    this->region_pref_cnt = 0;

    this->voter_sum = 0;
    this->vote_cnt = 0;
}

void bingo_plus_module::print_stats() {
    std::cout << "[" << cpu << " Bingo] PHT Access: " << this->pht_access_cnt << std::endl;
    std::cout << "[" << cpu << " Bingo] PHT Hit PC+Addr: " << this->pht_pc_address_cnt << std::endl;
    std::cout << "[" << cpu << " Bingo] PHT Hit PC+Offs: " << this->pht_pc_offset_cnt << std::endl;
    std::cout << "[" << cpu << " Bingo] PHT Miss: " << this->pht_miss_cnt << std::endl;

    std::cout << "[" << cpu << " Bingo] Prefetch PC+Addr: " << this->prefetch_cnt[PC_ADDRESS] << std::endl;
    std::cout << "[" << cpu << " Bingo] Prefetch PC+Offs: " << this->prefetch_cnt[PC_OFFSET] << std::endl;
    
    std::cout << "[" << cpu << " Bingo] Useful PC+Addr: " << this->useful_cnt[PC_ADDRESS] << std::endl;
    std::cout << "[" << cpu << " Bingo] Useful PC+Offs: " << this->useful_cnt[PC_OFFSET] << std::endl;
    
    std::cout << "[" << cpu << " Bingo] Useless PC+Addr: " << this->useless_cnt[PC_ADDRESS] << std::endl;
    std::cout << "[" << cpu << " Bingo] Useless PC+Offs: " << this->useless_cnt[PC_OFFSET] << std::endl;

    double l1_pref_per_region = 1.0 * this->pref_level_cnt[true] / this->region_pref_cnt;
    double l2_pref_per_region = 1.0 * this->pref_level_cnt[false] / this->region_pref_cnt;
    //double l3_pref_per_region = 1.0 * this->pref_level_cnt[FILL_LLC] / this->region_pref_cnt;
    double no_pref_per_region = (double)this->pattern_len - (l1_pref_per_region +
                                                                l2_pref_per_region);// +
                                                                //l3_pref_per_region);

    std::cout << "[" << cpu << " Bingo] L1 Prefetch per Region: " << l1_pref_per_region << std::endl;
    std::cout << "[" << cpu << " Bingo] L2 Prefetch per Region: " << l2_pref_per_region << std::endl;
    //std::cout << "[" << cpu << " Bingo] L3 Prefetch per Region: " << l3_pref_per_region << endl;
    std::cout << "[" << cpu << " Bingo] No Prefetch per Region: " << no_pref_per_region << std::endl;

    double voter_mean = 1.0 * this->voter_sum / this->vote_cnt;
    double voter_sqr_mean = 1.0 * this->voter_sqr_sum / this->vote_cnt;
    double voter_sd = sqrt(voter_sqr_mean - square(voter_mean));
    std::cout << "[" << cpu << " Bingo] Number of Voters Mean: " << voter_mean << std::endl;
    std::cout << "[" << cpu << " Bingo] Number of Voters SD: " << voter_sd << std::endl;
}

/**
 * Performs a PHT lookup and computes a prefetching pattern from the result.
 * @return The appropriate prefetch level for all blocks based on PHT output or an empty vector
 *         if no blocks should be prefetched
 */
std::vector<bool> bingo_plus_module::find_in_pht(uint64_t pc, uint64_t address) {
    if (this->debug_level >= 2) {
        std::cerr << "[" << cpu << " Bingo] find_in_pht(pc=0x" << std::hex << pc << ", address=0x" << address << ")" << std::dec << std::endl;
    }
    std::vector<std::vector<bool>> matches = this->pht.find(pc, address);
    this->pht_access_cnt += 1;
    Event pht_last_event = this->pht.get_last_event();
    uint64_t region_number = address / this->pattern_len;
    if (pht_last_event != MISS)
        this->pht_events[region_number] = pht_last_event;
    std::vector<bool> pattern;
    if (pht_last_event == PC_ADDRESS) {
        this->pht_pc_address_cnt += 1;
        // assert(matches.size() == 1); /* there can only be 1 PC+Address match */
        // assert(matches[0].size() == (unsigned)this->pattern_len);
        pattern.resize(this->pattern_len, 0);
        for (int i = 0; i < this->pattern_len; i += 1)
            if (matches[0][i])
                pattern[i] = true;
    } else if (pht_last_event == PC_OFFSET) {
        this->pht_pc_offset_cnt += 1;
        pattern = this->vote(matches);
    } else if (pht_last_event == MISS) {
        this->pht_miss_cnt += 1;
    } else {
        /* error: unknown event! */
        // assert(0);
    }
    /* stats */
    if (pht_last_event != MISS) {
        this->region_pref_cnt += 1;
        for (int i = 0; i < (int)pattern.size(); i += 1)
            if (pattern[i] != 0)
                this->pref_level_cnt[pattern[i]] += 1;
        // assert(this->pref_level_cnt.size() <= 3); /* L1, L2, L3 */
    }
    /* ===== */
    return pattern;
}

void bingo_plus_module::insert_in_pht(const Entry<AccumulationTableData> &entry) {
    uint64_t pc = entry.data.pc;
    uint64_t region_number = hash_index_plus(entry.key, this->accumulation_table.get_index_len());
    uint64_t address = region_number * this->pattern_len + entry.data.offset;
    if (this->debug_level >= 2) {
        std::cerr << "[" << cpu << " Bingo] insert_in_pht(pc=0x" << std::hex << pc << ", address=0x" << address << ")" << std::dec << std::endl;
    }
    const std::vector<bool> &pattern = entry.data.pattern;
    this->pht.insert(pc, address, pattern);
}

/**
 * Uses a voting mechanism to produce a prefetching pattern from a set of footprints.
 * @param x The patterns obtained from all PC+Offset matches
 * @return  The appropriate prefetch level for all blocks based on BINGO's voting thresholds or
 *          an empty vector if no blocks should be prefetched
 */
std::vector<bool> bingo_plus_module::vote(const std::vector<std::vector<bool>> &x) {
    if (this->debug_level >= 2)
        std::cerr << "Bingo::vote(...)" << std::endl;
    int n = x.size();
    if (n == 0) {
        if (this->debug_level >= 2)
            std::cerr << "[Bingo::vote] There are no voters." << std::endl;
        return {};
    }
    /* stats */
    this->vote_cnt += 1;
    this->voter_sum += n;
    this->voter_sqr_sum += square(n);
    /* ===== */
    if (this->debug_level >= 2) {
        std::cerr << "[Bingo::vote] Taking a vote among:" << std::endl;
        for (int i = 0; i < n; i += 1)
            std::cerr << "<" << std::setw(3) << i + 1 << "> " << pattern_to_string(x[i]) << std::endl;
    }
    bool pf_flag = false;
    std::vector<bool> res(this->pattern_len, false);
    //for (int i = 0; i < n; i += 1)
        // assert((int)x[i].size() == this->pattern_len);
    for (int i = 0; i < this->pattern_len; i += 1) {
        int cnt = 0;
        for (int j = 0; j < n; j += 1)
            if (x[j][i])
                cnt += 1;
        double p = 1.0 * cnt / n;
        if (p >= L1D_THRESH)
            res[i] = true;
        //else if (p >= L2C_THRESH)
            //res[i] = false;
        //else if (p >= LLC_THRESH)
            //res[i] = FILL_LLC;
        else
            res[i] = false;
        if (res[i] != false)
            pf_flag = true;
    }
    if (this->debug_level >= 2) {
        std::cerr << "<res> " << pattern_to_string(res) << std::endl;
    }
    if (!pf_flag)
        return {};
    return res;
}

void bingo_plus::prefetcher_initialize() {
    core_proportion_counter = std::vector<uint32_t>(NUM_CPUS,0);
    core_proportion_index = std::vector<uint32_t>(NUM_CPUS,0);
    uint64_t bingo_state = 58560 + 1472 + 488 + 1904;
    auto bingo_bytes = champsim::data::bytes{bingo_state*NUM_CPUS};
    for(int i = 0; i < NUM_CPUS; i++)
        prefetchers.emplace_back(intern_,this,i);
    
    fmt::print("[{}] Bingo+ Initialized. Total State: {}\n",intern_->NAME,champsim::data::kibibytes{bingo_bytes});
    /*if (bingo_plus_module::DEBUG_LEVEL >= 1)
        std::cerr << "Bingo::Bingo(pattern_len=" << pattern_len << ", min_addr_width=" << bingo_plus_module::MIN_ADDR_WIDTH
                << ", max_addr_width=" << bingo_plus_module::MAX_ADDR_WIDTH << ", pc_width=" << bingo_plus_module::PC_WIDTH
                << ", filter_table_size=" << bingo_plus_module::FT_SIZE
                << ", accumulation_table_size=" << bingo_plus_module::AT_SIZE << ", pht_size=" << bingo_plus_module::PHT_SIZE
                << ", pht_ways=" << bingo_plus_module::PHT_WAYS << ", pf_streamer_size=" << bingo_plus_module::PF_STREAMER_SIZE
                << ", debug_level=" << bingo_plus_module::DEBUG_LEVEL << ")" << std::endl;*/
}

uint32_t bingo_plus::prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in)
{

    if(!cache_hit && type == access_type::PREFETCH) {
        core_proportion_counter.at(intern_->cpu)++;
        uint64_t total = 0;
        for(int i = 0; i < NUM_CPUS; i++) {
            total += core_proportion_counter.at(i);
        }
        if(total >= PORP_SAMPLE - 1) {
            for(int i = 0; i < NUM_CPUS; i++) {
                core_proportion_index.at(i) = core_proportion_counter.at(i) >> (champsim::lg2(PORP_SAMPLE) - 4);
                core_proportion_counter.at(i) = 0;
            }
        }
    }
    
    if (bingo_plus_module::DEBUG_LEVEL >= 2) {
        std::cerr << "CACHE::l1d_prefetcher_operate(addr=0x" << std::hex << addr << ", ip=0x" << ip << ", cache_hit=" << std::dec
             << (int)cache_hit << ", type=" << (int)type << ")" << std::dec << std::endl;
        std::cerr << "[CACHE::l1d_prefetcher_operate] CACHE{core=" << intern_->cpu << ", NAME=" << intern_->NAME << "}" << std::dec
             << std::endl;
    }

    //if (type != access_type::LOAD && !bingo_plus_module::DO_PREFETCH_PREFETCH) {
    //    return cache_hit ? bingo_plus_module::BINGO_WAS_LLC_HIT : metadata_in;
    //}

    
    champsim::block_number block_number{addr};
    uint32_t ip_data = (metadata_in >> BERTI_IP_ENCODING_OFFSET) & BERTI_IP_MASK;
    /* update BINGO with most recent LOAD access */
    if(type == access_type::PREFETCH && bingo_plus_module::DO_PREFETCH_TRAIN) {
        if(ip_data != 0)
            prefetchers[intern_->cpu].access(block_number.to<uint64_t>(), ip_data);
    }
    else if(type != access_type::PREFETCH) {
        if(bingo_plus_module::DO_PREFETCH_TRAIN)
            prefetchers[intern_->cpu].access(block_number.to<uint64_t>(), ip.to<uint64_t>() & BERTI_IP_MASK);
        else if(ip.to<uint64_t>() != 0)
            prefetchers[intern_->cpu].access(block_number.to<uint64_t>(), ip.to<uint64_t>());
    }

    /* issue prefetches */
    if(bingo_plus_module::DO_PREFETCH_PREFETCH || type == access_type::LOAD)
        prefetchers[intern_->cpu].prefetch(block_number.to<uint64_t>());

    if (bingo_plus_module::DEBUG_LEVEL >= 3) {
        prefetchers[intern_->cpu].log();
        std::cerr << "=======================================" << std::dec << std::endl;
    }

    //if hit return proportion
    if(cache_hit) {
        uint32_t metadata = 0;
        //fmt::print("Sending core proportion: {}\n",core_proportion_index.at(intern_->cpu));
        metadata |= bingo_plus_module::BINGO_PROPORTION;
        metadata |= core_proportion_index.at(intern_->cpu) << 2;
        //fmt::print("Metadata: {:b}\n",metadata);
        return metadata;
    }

    //if(cache_hit)
    //    return bingo_plus_module::BINGO_WAS_LLC_HIT;
    //else
        return 0;
}

uint32_t bingo_plus::prefetcher_cache_fill(champsim::address addr, long set, long way, bool prefetch, champsim::address evicted_addr, uint32_t metadata_in) {
    champsim::block_number evicted_block_number{evicted_addr};
    if(evicted_addr == champsim::address{})
        return 0;
    //if (this->block[set][way].valid == 0)
        //return metadata_in; /* no eviction */

    /* inform all sms modules of the eviction */
    for (std::size_t i = 0; i < NUM_CPUS; i++)
        prefetchers[i].eviction(evicted_block_number.to<uint64_t>());

    uint32_t metadata = 0; 
    if(evicted_addr != champsim::address{}) {
        metadata |= bingo_plus_module::BINGO_LLC_EVICT;
        metadata |= champsim::block_number{evicted_addr}.to<uint64_t>() << 2;
        //fmt::print("Informing L2C that: {} was evicted\n",champsim::address{champsim::block_number{evicted_addr}});
    }


    return metadata;
}

void bingo_plus::prefetcher_final_stats() {
    for(int i = 0; i < NUM_CPUS; i++)
        prefetchers[i].print_stats();
}
void bingo_plus::prefetcher_cycle_operate() {

}