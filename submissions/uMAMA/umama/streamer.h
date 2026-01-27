#ifndef STREAMER_H
#define STREAMER_H

#include <deque>
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include "champsim.h"
#include "address.h"
#include "modules.h"
#include "extent.h"
#include "bloom.h"

using namespace std;

struct page_block_extent : champsim::dynamic_extent {
    page_block_extent() : dynamic_extent(champsim::data::bits{LOG2_PAGE_SIZE}, champsim::data::bits{LOG2_BLOCK_SIZE}) {}
};
using page_block = champsim::address_slice<page_block_extent>;

class Stream_Tracker
{
public:
    champsim::page_number page{};
    page_block last_offset{}; // TODO: confirm that this is correct.
    int32_t last_dir; /* +1 means +ve stream, -1 means -ve stream, 0: init */
    uint8_t conf;
    
public:
    Stream_Tracker(champsim::page_number _page, page_block _last_offset)
    {
        page = _page;
        last_offset = _last_offset;
        last_dir = 0;
        conf = 0;
    }
    ~Stream_Tracker(){}
};

template<uint32_t TRACKERS, uint32_t MAX_Q>
struct Streamer : public champsim::modules::prefetcher
{
private:
    deque<Stream_Tracker*> trackers;
    std::deque<champsim::address> active_streams{};

    struct 
    {
        uint64_t called;
        struct
        {
            uint64_t missed;
            uint64_t evict;
            uint64_t insert;
            uint64_t hit;
            uint64_t same_offset;
            uint64_t dir_match;
            uint64_t dir_mismatch;
        } tracker;
        struct
        {
            uint64_t dir_match;
            uint64_t total;
        } pred;
    } stats;

    //const uint32_t streamer_num_trackers = 256;
    uint32_t streamer_pref_degree = 4;
    //const uint32_t max_queued = 48; // Maximum number of requests to queue.
    
private:
    std::size_t size(page_block_extent ext) {
        return champsim::size(ext);
    }

    BloomFilter<L2_BLOOM_N, L2_BLOOM_M> *bloom;


public:
    using champsim::modules::prefetcher::prefetcher;

    void set_pref_degree(uint32_t deg)
    {
        streamer_pref_degree = deg;
    }

    void set_bloom(BloomFilter<L2_BLOOM_N, L2_BLOOM_M> *bloom_) {
        bloom = bloom_;
    }
    
    void prefetcher_initialize()
    {
        bzero(&stats, sizeof(stats));
    }
    
    void print_config()
    {
        std::cout << "TRACKERS " << TRACKERS << endl
                  << "streamer_pref_degree " << streamer_pref_degree << endl
                  << endl;
    }
    
    void prefetcher_final_stats()
    {
        cout << "streamer.called " << stats.called << endl
             << "streamer.tracker.missed " << stats.tracker.missed << endl
             << "streamer.tracker.evict " << stats.tracker.evict << endl
             << "streamer.tracker.insert " << stats.tracker.insert << endl
             << "streamer.tracker.hit " << stats.tracker.hit << endl
             << "streamer.tracker.same_offset " << stats.tracker.same_offset << endl
             << "streamer.tracker.dir_match " << stats.tracker.dir_match << endl
             << "streamer.tracker.dir_mismatch " << stats.tracker.dir_mismatch << endl
             << "streamer.pred.dir_match " << stats.pred.dir_match << endl
             << "streamer.pred.total " << stats.pred.total << endl
             << endl;
    }
    
    uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                          uint32_t metadata_in) 
    {
        champsim::page_number page{addr};
        page_block offset{addr};
        
        stats.called++;
    
        auto it = find_if(trackers.begin(), trackers.end(), 
                         [page](Stream_Tracker *tracker){return tracker->page == page;});
        
        Stream_Tracker *tracker = (it != trackers.end()) ? (*it) : NULL;
    
        if(!tracker)
        {
            stats.tracker.missed++;
            if(trackers.size() >= TRACKERS)
            {
                tracker = trackers.front();
                trackers.pop_front();
                delete(tracker);
                stats.tracker.evict++;
            }
    
            tracker = new Stream_Tracker(page, offset);
            trackers.push_back(tracker);
            stats.tracker.insert++;
            return metadata_in;
        }
    
        assert(tracker->page == page);
        stats.tracker.hit++;
    
        if(offset == tracker->last_offset)
        {
            stats.tracker.same_offset++;
            return metadata_in;
        }
    
        bool dir_match = false;
        int32_t dir = offset > tracker->last_offset ? +1 : -1;
        if(dir == tracker->last_dir)
        {
            tracker->conf = 1;
            dir_match = true;
            stats.tracker.dir_match++;
        }
        else
        {
            tracker->conf = 0;
            stats.tracker.dir_mismatch++;
        }
        tracker->page = page;
        tracker->last_offset = offset;
        tracker->last_dir = dir;
        /* update recency */
        trackers.erase(it);
        trackers.push_back(tracker);
    
        /* generate prefetch */
        if(dir_match)
        {
            stats.pred.dir_match++;
            page_block pref_offset = offset;
            for(uint32_t index = 0; index < streamer_pref_degree; ++index)
            {
                // Some extra overflow checking needs tobe implemented
                page_block next_pref_offset = (dir == +1) ? (pref_offset + 1) : (pref_offset - 1);
                if (dir == +1 && next_pref_offset <= pref_offset ) {
                    break;
                } else if(dir == -1 && next_pref_offset >= pref_offset) {
                    break;
                } else {
                    pref_offset = next_pref_offset;
                    champsim::address pf_addr{champsim::splice(page, pref_offset)};
                    uint64_t addr_int = pf_addr.to<uint64_t>();
                    if(std::find(active_streams.begin(), active_streams.end(), pf_addr) == active_streams.end()
                        && (bloom == nullptr || !bloom->test(addr_int)))
                    {
                        active_streams.push_back(pf_addr);
                        if(bloom != nullptr)
                            bloom->insert((uint64_t) pf_addr.to<uint64_t>());
                    }

                    if(active_streams.size() > MAX_Q) {
                        active_streams.pop_front(); // Drop older requests if the queue fills up
                    }
                }
            }
        }
    
        return metadata_in;
    }
    
    void prefetcher_cycle_operate()
    {
        while(active_streams.size() > 0) {
            champsim::address pf_addr = active_streams.front();
    
            // Try to prefetch
            const bool success = prefetch_line(pf_addr, true, 0);
    
            if (!success) {
                break; // If unsuccessful, stop here and try again next cycle
            }
    
            active_streams.pop_front();
        }
    }
    
    uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
    {
        return metadata_in;
    }
};

#endif /* STREAMER_H */

