#ifndef IP_STRIDE_H
#define IP_STRIDE_H

#include <cstdint>
#include <optional>

#include "address.h"
#include "champsim.h"
#include "modules.h"
#include "cache.h"
#include "msl/lru_table.h"
#include "bloom.h"

template<std::size_t TRACKER_SETS, std::size_t TRACKER_WAYS>
struct ip_stride2 : public champsim::modules::prefetcher {
  struct tracker_entry {
    champsim::address ip{};                                // the IP we're tracking
    champsim::block_number last_cl_addr{};                 // the last address accessed by this IP
    champsim::block_number::difference_type last_stride{}; // the stride between the last two addresses accessed by this IP

    auto index() const
    {
      using namespace champsim::data::data_literals;
      return ip.slice_upper<2_b>();
    }
    auto tag() const
    {
      using namespace champsim::data::data_literals;
      return ip.slice_upper<2_b>();
    }
  };

  struct lookahead_entry {
    champsim::address address{};
    champsim::address::difference_type stride{};
    int degree = 0; // degree remaining
  };

  int PREFETCH_DEGREE = 3;

  std::optional<lookahead_entry> active_lookahead;

  champsim::msl::lru_table<tracker_entry> table{TRACKER_SETS, TRACKER_WAYS};

  BloomFilter<L2_BLOOM_N, L2_BLOOM_M> *bloom = nullptr;

public:
  using champsim::modules::prefetcher::prefetcher;

  void set_bloom(BloomFilter<L2_BLOOM_N, L2_BLOOM_M> * bloom_) {
    bloom = bloom_;
  }

  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                               uint32_t metadata_in)
  {
    champsim::block_number cl_addr{addr};
    champsim::block_number::difference_type stride = 0;
  
    auto found = table.check_hit({ip, cl_addr, stride});
  
    // if we found a matching entry
    if (found.has_value()) {
      stride = champsim::offset(found->last_cl_addr, cl_addr);
  
      // Initialize prefetch state unless we somehow saw the same address twice in
      // a row or if this is the first time we've seen this stride
      if (stride != 0 && stride == found->last_stride)
        active_lookahead = {champsim::address{cl_addr}, stride, PREFETCH_DEGREE};
    }
  
    // update tracking set
    table.fill({ip, cl_addr, stride});
  
    return metadata_in;
  }
  
  void prefetcher_cycle_operate()
  {
    // If a lookahead is active
    if (active_lookahead.has_value()) {
      auto [old_pf_address, stride, degree] = active_lookahead.value();
      if(degree == 0) {
        return;
      }
  
      champsim::address pf_address{champsim::block_number{old_pf_address} + stride};

      if(bloom && bloom->test((uint64_t) pf_address.to<uint64_t>())) {
        active_lookahead = {pf_address, stride, degree - 1};
        return;
      }
  
      // If the next step would exceed the degree or run off the page, stop
      if (intern_->virtual_prefetch || champsim::page_number{pf_address} == champsim::page_number{old_pf_address}) {
        // check the MSHR occupancy to decide if we're going to prefetch to this level or not
        const bool mshr_under_light_load = intern_->get_mshr_occupancy_ratio() < 0.5;
        const bool success = prefetch_line(pf_address, mshr_under_light_load, 0);

        if (success) {
          active_lookahead = {pf_address, stride, degree - 1};
          if (bloom) {
            bloom->insert((uint64_t) pf_address.to<uint64_t>());
          }
        }

        if (active_lookahead->degree == 0) {
          active_lookahead.reset();
        }
      } else {
        active_lookahead.reset();
      }
    }
  }
  
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
  {
    return metadata_in;
  }
};

#endif
