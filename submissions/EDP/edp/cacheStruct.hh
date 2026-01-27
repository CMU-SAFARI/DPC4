#ifndef CACHE_STRUCT_ENTANGLING_BERTI_H_
#define CACHE_STRUCT_ENTANGLING_BERTI_H_
/*
 * The Entangling Data Prefetcher (EDP)
 *
 * Submission #49
 *
 * 4 Data Prefetching Championship
 */

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <ostream>

template <typename T, size_t SETS, size_t WAYS> class Cache_Berti {
public:
  std::array<std::array<T *, WAYS>, SETS> cache;
  std::array<std::array<bool, WAYS>, SETS> nru;

  Cache_Berti() = default;

  std::optional<T *> get(uint64_t key) {
    uint64_t set = key % SETS;

    auto dx = std::find_if(std::begin(cache[set]), std::end(cache[set]),
                           [key](auto &i) { return i && i->tag == key; });

    if (dx == std::end(cache[set]))
      return std::nullopt;

    uint64_t way = std::distance(std::begin(cache[set]), dx);
    nru[set][way] = true;

    return *dx;
  }

  T *get(uint64_t set, uint64_t way) {
    nru[set][way] = true;
    return cache[set][way];
  }

  std::pair<uint64_t, uint64_t> getSetWay(uint64_t key) {
    uint64_t set = key % SETS;

    auto dx = std::find_if(std::begin(cache[set]), std::end(cache[set]),
                           [key](auto &i) { return i && i->tag == key; });

    if (dx == std::end(cache[set]))
      return std::pair<uint64_t, uint64_t>(SETS, WAYS);

    uint64_t way = std::distance(std::begin(cache[set]), dx);

    return std::pair<uint64_t, uint64_t>(set, way);
  }

  void insert(uint64_t key, T *e) {
    uint64_t set = key % SETS;

    auto dx = std::find_if(std::begin(cache[set]), std::end(cache[set]),
                           [key](auto &i) { return i && i->tag == key; });

    if (dx == std::end(cache[set])) {
      // NRU replacement
      auto way = std::find_if(std::begin(nru[set]), std::end(nru[set]),
                              [](auto &i) { return !i; });
      uint64_t waydx = 0;
      if (way == std::end(nru[set]))
        std::for_each(std::begin(nru[set]), std::end(nru[set]),
                      [](auto &i) { i = false; });
      else
        waydx = std::distance(std::begin(nru[set]), way);

      nru[set][waydx] = true;
      delete cache[set][waydx];
      cache[set][waydx] = e;

      return;
    }
    *dx = e;
  }
};

#endif
