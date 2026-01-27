#ifndef CUSTOM_UTIL_H
#define CUSTOM_UTIL_H

#include "champsim.h"

#include <stdint.h>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <type_traits>

namespace custom_util {

// Macros used by SRRIP cache
#define ADD(x, MAX) (x = (x >= static_cast<std::remove_reference<decltype(x)>::type>(MAX)) ? x : x + 1)

// Hash functions (inline)
inline uint64_t hash_index(uint64_t key, int index_len) {
    if (index_len == 0)
        return key;
    for (uint64_t tag = (key >> index_len); tag > 0; tag >>= index_len)
        key ^= tag & ((1 << index_len) - 1);
    return key;
}

inline uint64_t my_hash_index(uint64_t key, int index_len, int discard_lsb_len) {
    key >>= discard_lsb_len;
    key &= ((1ULL << index_len) - 1);
    return key;
}

// Pattern to string template
template <class T>
std::string pattern_to_string(const std::vector<T>& pattern) {
    std::ostringstream oss;
    for (unsigned i = 0; i < pattern.size(); i += 1)
        oss << int(pattern[i]) << " ";
    return oss.str();
}

// Table class for logging
class Table {
public:
    Table(int width_val, int height_val) :
        width(width_val), height(height_val), cells(height_val, std::vector<std::string>(width_val)) {}
    
    void set_row(int row, const std::vector<std::string>& data, int start_col = 0) {
        for (unsigned col = start_col; col < this->width && col - start_col < data.size(); col += 1)
            this->set_cell(row, col, data[col - start_col]);
    }

    void set_col(int col, const std::vector<std::string>& data, int start_row = 0) {
        for (unsigned row = start_row; row < this->height && row - start_row < data.size(); row += 1)
            this->set_cell(row, col, data[row - start_row]);
    }

    void set_cell(int row, int col, std::string data) {
        if (row >= 0 && row < (int)this->height && col >= 0 && col < (int)this->width)
            this->cells[row][col] = data;
    }

    void set_cell(int row, int col, double data) {
        std::ostringstream oss;
        oss << std::setw(11) << std::fixed << std::setprecision(8) << data;
        this->set_cell(row, col, oss.str());
    }

    void set_cell(int row, int col, int64_t data) {
        std::ostringstream oss;
        oss << std::setw(11) << std::left << data;
        this->set_cell(row, col, oss.str());
    }

    void set_cell(int row, int col, int data) { 
        this->set_cell(row, col, (int64_t)data); 
    }

    void set_cell(int row, int col, uint64_t data) {
        std::ostringstream oss;
        oss << "0x" << std::setfill('0') << std::setw(16) << std::hex << data;
        this->set_cell(row, col, oss.str());
    }
    
    std::string to_string() {
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

    std::string data_row(int row, const std::vector<int>& widths) {
        std::string out;
        for (unsigned i = 0; i < this->width; i += 1) {
            std::string data = this->cells[row][i];
            data.resize(widths[i] - 2, ' ');
            out += " | " + data;
        }
        out += " |\n";
        return out;
    }

    static std::string top_line(const std::vector<int>& widths) { 
        return Table::line(widths, "┌", "┬", "┐"); 
    }

    static std::string mid_line(const std::vector<int>& widths) { 
        return Table::line(widths, "├", "┼", "┤"); 
    }

    static std::string bot_line(const std::vector<int>& widths) { 
        return Table::line(widths, "└", "┴", "┘"); 
    }

    static std::string line(const std::vector<int>& widths, std::string left, std::string mid, std::string right) {
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

private:
    unsigned width;
    unsigned height;
    std::vector<std::vector<std::string>> cells;
};

// Set-associative cache base class
template <class T>
class SetAssociativeCache {
public:
    class Entry {
    public:
        uint64_t key;
        uint64_t index;
        uint64_t tag;
        bool valid;
        T data;
    };

    SetAssociativeCache(int size_val, int num_ways_val, int debug_level_val = 0) :
        size(size_val), num_ways(num_ways_val), num_sets(size_val / num_ways_val), entries(num_sets, std::vector<Entry>(num_ways_val)),
        cams(num_sets, std::unordered_map<uint64_t, int>(num_ways_val)), debug_level(debug_level_val) {
        for (int i = 0; i < num_sets; i += 1)
            for (int j = 0; j < num_ways; j += 1)
                entries[i][j].valid = false;
        
        for (int max_index = num_sets - 1; max_index > 0; max_index >>= 1)
            this->index_len += 1;
    }

    Entry* erase(uint64_t key) {
        Entry* entry = this->find(key);
        uint64_t index = key & ((1 << this->index_len) - 1);
        uint64_t tag = key >> this->index_len;
        auto& cam = cams[index];
        cam.erase(tag);
        if (entry)
            entry->valid = false;
        return entry;
    }

    Entry insert(uint64_t key, const T& data) {
        Entry* entry = this->find(key);
        if (entry != nullptr) {
            Entry old_entry = *entry;
            entry->data = data;
            return old_entry;
        }
        uint64_t index = key & ((1 << this->index_len) - 1);
        uint64_t tag = key >> this->index_len;
        std::vector<Entry>& set = this->entries[index];
        int victim_way = -1;
        for (int i = 0; i < this->num_ways; i += 1)
            if (!set[i].valid) {
                victim_way = i;
                break;
            }
        if (victim_way == -1) {
            victim_way = this->select_victim(index);
        }
        Entry& victim = set[victim_way];
        Entry old_entry = victim;
        victim = {key, index, tag, true, data};
        auto& cam = cams[index];
        if (old_entry.valid) {
            cam.erase(old_entry.tag);
        }
        cam[tag] = victim_way;
        return old_entry;
    }

    Entry* find(uint64_t key) {
        uint64_t index = key & ((1 << this->index_len) - 1);
        uint64_t tag = key >> this->index_len;
        auto& cam = cams[index];
        if (cam.find(tag) == cam.end())
            return nullptr;
        int way = cam[tag];
        Entry& entry = this->entries[index][way];
        if (!entry.valid)
            return nullptr;
        return &entry;
    }

    std::string log(std::vector<std::string> headers) {
        std::vector<Entry> valid_entries = this->get_valid_entries();
        Table table(static_cast<int>(headers.size()), static_cast<int>(valid_entries.size() + 1));
        table.set_row(0, headers);
        for (unsigned i = 0; i < valid_entries.size(); i += 1)
            this->write_data(valid_entries[i], table, i + 1);
        return table.to_string();
    }

    int get_index_len() { return this->index_len; }

protected:
    virtual void write_data(Entry& entry, Table& table, int row) {
        (void)entry; (void)table; (void)row;
    }

    virtual int select_victim(uint64_t index) {
        (void)index;
        return rand() % this->num_ways;
    }

    std::vector<Entry> get_valid_entries() {
        std::vector<Entry> valid_entries;
        for (int i = 0; i < num_sets; i += 1)
            for (int j = 0; j < num_ways; j += 1)
                if (entries[i][j].valid)
                    valid_entries.push_back(entries[i][j]);
        return valid_entries;
    }

    int size;
    int num_ways;
    int num_sets;
    int index_len = 0;
    std::vector<std::vector<Entry>> entries;
    std::vector<std::unordered_map<uint64_t, int>> cams;
    int debug_level = 0;
};

// LRU Set Associative Cache
template <class T>
class LRUSetAssociativeCache : public SetAssociativeCache<T> {
    typedef SetAssociativeCache<T> Super;

public:
    LRUSetAssociativeCache(int size_val, int num_ways_val, int debug_level_val = 0) :
        Super(size_val, num_ways_val, debug_level_val), lru(this->num_sets, std::vector<uint64_t>(num_ways_val)) {}

    void set_mru(uint64_t key) { *this->get_lru(key) = this->t++; }
    void set_lru(uint64_t key) { *this->get_lru(key) = 0; }
    void rp_promote(uint64_t key) { set_mru(key); }
    void rp_insert(uint64_t key) { set_mru(key); }

protected:
    int select_victim(uint64_t index) override {
        std::vector<uint64_t>& lru_set = this->lru[index];
        return static_cast<int>(std::min_element(lru_set.begin(), lru_set.end()) - lru_set.begin());
    }

    uint64_t* get_lru(uint64_t key) {
        uint64_t index = key % this->num_sets;
        uint64_t tag = key / this->num_sets;
        int way = this->cams[index][tag];
        return &this->lru[index][way];
    }

    std::vector<std::vector<uint64_t>> lru;
    uint64_t t = 1;
};

// SRRIP Set Associative Cache
template <class T>
class SRRIPSetAssociativeCache : public SetAssociativeCache<T> {
    typedef SetAssociativeCache<T> Super;

public:
    SRRIPSetAssociativeCache(int size_val, int num_ways_val, int debug_level_val = 0, int max_rrpv_val = 3) :
        Super(size_val, num_ways_val, debug_level_val), rrpv(this->num_sets, std::vector<uint64_t>(num_ways_val)),
        max_rrpv(max_rrpv_val) {}

    void rp_promote(uint64_t key) { *this->get_rrpv(key) = 0; }
    void rp_insert(uint64_t key) { *this->get_rrpv(key) = 2; }

protected:
    int select_victim(uint64_t index) override {
        std::vector<uint64_t>& rrpv_set = this->rrpv[index];
        for (;;) {
            for (int i = 0; i < this->num_ways; i++) {
                if (rrpv_set[i] >= static_cast<uint64_t>(max_rrpv)) {
                    return i;
                }
            }
            aging(index);
        }
    }

    uint64_t* get_rrpv(uint64_t key) {
        uint64_t index = key % this->num_sets;
        uint64_t tag = key / this->num_sets;
        int way = this->cams[index][tag];
        return &this->rrpv[index][way];
    }

private:
    void aging(uint64_t index) {
        std::vector<uint64_t>& rrpv_set = this->rrpv[index];
        for (auto& r : rrpv_set) {
            ADD(r, max_rrpv);
        }
    }

    std::vector<std::vector<uint64_t>> rrpv;
    int max_rrpv;
};

} // namespace custom_util

#endif
