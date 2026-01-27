/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONF_TABLE_H
#define CONF_TABLE_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "extent.h"
#include "msl/bits.h"
#include "util/detect.h"
#include "util/span.h"
#include "util/type_traits.h"

namespace champsim
{
template <typename Extent>
class address_slice;
}

namespace sppam_util::msl
{
namespace detail
{
template <typename T>
struct table_indexer {
  auto operator()(const T& t) const { return t.index(); }
};

template <typename T>
struct table_tagger {
  auto operator()(const T& t) const { return t.tag(); }
};

template <class T, class U>
constexpr bool cmp_equal(T t, U u) noexcept
{
  using UT = std::make_unsigned_t<T>;
  using UU = std::make_unsigned_t<U>;
  if constexpr (std::is_signed_v<T> == std::is_signed_v<U>)
    return t == u;
  else if constexpr (std::is_signed_v<T>)
    return t < 0 ? false : UT(t) == u;
  else
    return u < 0 ? false : t == UU(u);
}
} // namespace detail

template <typename T, typename SetProj = detail::table_indexer<T>, typename TagProj = detail::table_tagger<T>>
class conf_table
{
public:
  using value_type = T;

private:
  struct block_t {
    bool valid = false;
    uint64_t conf = 0;
    value_type data;
  };
  using block_vec_type = std::vector<block_t>;
  using diff_type = typename block_vec_type::difference_type;

  SetProj set_projection;
  TagProj tag_projection;

  diff_type NUM_SET;
  diff_type NUM_WAY;
  uint64_t access_count = 0;
  block_vec_type block;

  auto get_set_span(const value_type& elem)
  {
    diff_type set_idx;
    if constexpr (champsim::is_specialization_v<std::invoke_result_t<SetProj, decltype(elem)>, champsim::address_slice>) {
      set_idx = set_projection(elem).template to<decltype(set_idx)>();
    } else {
      set_idx = static_cast<diff_type>(set_projection(elem));
    }
    if (set_idx < 0)
      throw std::range_error{"Set projection produced negative set index: " + std::to_string(set_idx)};
    diff_type raw_idx{(set_idx % NUM_SET) * NUM_WAY};
    auto begin = std::next(std::begin(block), raw_idx);
    auto end = std::next(begin, NUM_WAY);
    return std::pair{begin, end};
  }

  auto match_func(const value_type& elem)
  {
    return [tag = tag_projection(elem), proj = this->tag_projection](const block_t& x) {
      return x.valid && proj(x.data) == tag;
    };
  }

  template <typename U>
  auto match_and_check(U tag)
  {
    return [tag, proj = this->tag_projection](const auto& x, const auto& y) {
      auto x_valid = x.valid;
      auto y_valid = y.valid;
      auto x_match = proj(x.data) == tag;
      auto y_match = proj(y.data) == tag;
      auto cmp_conf = x.conf < y.conf;
      return !x_valid || (y_valid && ((!x_match && y_match) || ((x_match == y_match) && cmp_conf)));
    };
  }

public:
  std::optional<value_type> incr_conf(const value_type& elem)
  {
    access_count++;
    auto [set_begin, set_end] = get_set_span(elem);
    auto hit = std::find_if(set_begin, set_end, match_func(elem));


    if (hit == set_end) {
      return std::nullopt;
    }

    //increment confidence of this entry by 1
    //fmt::print("Incrementing confidence of entry with tag {} from {} to {}\n", tag_projection(hit->data), hit->conf, hit->conf + 1);
    if(hit->conf < 100)
      hit->conf += 1;

    //all others are halved
    if(hit->conf >= 100)
    for(auto it = set_begin; it != set_end; ++it) {
      it->conf = it->conf >> 1;
    }
    return hit->data;
  }

  void print_stats() const
  {
    for(auto it = block.begin(); it != block.end(); ++it) {
      if(it->valid) {
        fmt::print("Prediction: {:b} Confidence: {}\n", it->data, it->conf);
      }
    }
  }

  std::optional<value_type> get_highest_conf(const value_type& elem, uint64_t min_conf = 0)
  {
    access_count++;
    auto [set_begin, set_end] = get_set_span(elem);

    uint64_t highest_conf = min_conf;
    auto highest_conf_entry = set_end;
    for(auto it = set_begin; it != set_end; ++it) {
      auto temp_conf = it->conf;
      if(temp_conf > highest_conf) {
        highest_conf_entry = it;
        highest_conf = temp_conf;
      }
    }
    if (highest_conf_entry == set_end) {
      return std::nullopt;
    }
    return highest_conf_entry->data;
  }

  void fill(const value_type& elem)
  {
    auto tag = tag_projection(elem);
    auto [set_begin, set_end] = get_set_span(elem);
    if (set_begin != set_end) {
      auto [miss, hit] = std::minmax_element(set_begin, set_end, match_and_check(tag));

      if (tag_projection(hit->data) == tag) {
        *hit = {true,25, elem};
      } else {
        *miss = {true,25, elem};
      }
    }
  }



  conf_table(std::size_t sets, std::size_t ways, SetProj set_proj, TagProj tag_proj)
      : set_projection(set_proj), tag_projection(tag_proj), NUM_SET(static_cast<diff_type>(sets)), NUM_WAY(static_cast<diff_type>(ways)), block(sets * ways)
  {
    if (!detail::cmp_equal(sets, static_cast<diff_type>(sets)))
      throw std::overflow_error{"Sets is out of bounds"};
    if (!detail::cmp_equal(ways, static_cast<diff_type>(ways)))
      throw std::overflow_error{"Ways is out of bounds"};
    if (sets <= 0)
      throw std::range_error{"Sets is not positive"};
    if ((sets & (sets - 1)) != 0)
      throw std::range_error{"Sets is not a power of 2"};
  }

  conf_table(std::size_t sets, std::size_t ways, SetProj set_proj) : conf_table(sets, ways, set_proj, {}) {}
  conf_table(std::size_t sets, std::size_t ways) : conf_table(sets, ways, {}, {}) {}
};
} // namespace sppam::msl

#endif
