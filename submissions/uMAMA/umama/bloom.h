#pragma once
// A simple implementation of a bloom filter.

#include <cmath>
#include "umama_params.h"

// Fairly large, but we have the budget for it
constexpr size_t L2_BLOOM_N = umama_config::bloom_n;
constexpr size_t L2_BLOOM_M = 1000;

// Parameterized by the number of items to target (n),
// and the number of bits to use in the filter (m).
template<size_t n, size_t m>
class BloomFilter {
public:

    BloomFilter() : k((int)((double) m / (double) n * log(2)) + 1), elems(0) {
        reset();
    }

    virtual void reset() {
        memset(bits, 0, sizeof(bits));
        elems = 0;
    }

    // Get the ith hash function
    uint64_t get_hash(uint64_t input, uint64_t i) {
        return (h1(input) + i * h2(input)) % m;
    }

    size_t num_entries() {
        return elems;
    }

    size_t capacity() {
        return n;
    }

    virtual void insert(uint64_t input) {
        if(test(input)) {
            return;
        }

        for (size_t i = 0; i < (size_t)this->k; i++) {
            uint64_t x = get_hash(input, i);
            bits[x / 8] |= (uint8_t)(1 << (x % 8));
        }

        elems++;
    }

    virtual bool test(uint64_t input) {
        for (size_t i = 0; i < (size_t)this->k; i++) {
            uint64_t x = get_hash(input, i);
            if(!(bits[x / 8] & (uint8_t)(1 << (x % 8)))) {
                return false;
            }
        }

        return true;
    }

protected:
    const int k; // Number of hash functions to use
    uint8_t bits[m / 8 + 1];
    size_t elems;

    // Two hash functions that combine to create the above
    uint64_t h1(uint64_t input) {
        return ((input * 1103515245 + 12345) % ((uint64_t) 1 << 31)) % m;
    }

    uint64_t h2(uint64_t input) {
        return ((input * 48271) % (((uint64_t) 1 << 31) - 1)) % m; 
    }
};

// This class uses two bloom filters (so total of 2*m bits)
// with an offset in reset timing from one another to guarantee
// that the latest n / 2 insertions will always be reflected.
template<size_t n, size_t m>
class DoubleBloomFilter : public BloomFilter<n, m> {
public:

    DoubleBloomFilter() {
        reset();
    }
    
    virtual void reset() {
        memset(this->bits, 0, sizeof(this->bits));
        memset(this->bits2, 0, sizeof(this->bits2));
        this->elems = 0;
        this->bits2_elems = n / 2;
    }

    virtual void insert(uint64_t input) {
        if(test(input)) {
            return;
        }

        for (size_t i = 0; i < (size_t)this->k; i++) {
            uint64_t x = this->get_hash(input, i);
            this->bits[x / 8]  |= (uint8_t)(1 << (x % 8));
            this->bits2[x / 8] |= (uint8_t)(1 << (x % 8));
        }

        this->elems++;
        this->bits2_elems++;

        if(this->elems > n) {
            memset(this->bits, 0, sizeof(this->bits));
            this->elems = 0;
        }

        if(this->bits2_elems > n) {
            memset(this->bits2, 0, sizeof(this->bits));
            this->bits2_elems = 0;
        }
    }

    virtual bool test(uint64_t input) {
        bool found_in_1 = true;
        bool found_in_2 = true;
        for (size_t i = 0; i < (size_t)this->k; i++) {
            uint64_t x = this->get_hash(input, i);
            if(!(this->bits[x / 8] & (uint8_t)(1 << (x % 8)))) {
                found_in_1 = false;
            }

            if(!(this->bits2[x / 8] & (uint8_t)(1 << (x % 8)))) {
                found_in_2 = false;
            }
        }

        return found_in_1 || found_in_2;
    }

private:
    uint8_t bits2[m / 8 + 1];
    size_t bits2_elems = n / 2;
};
