#ifndef ANTI_INTERFERENCE_HPP
#define ANTI_INTERFERENCE_HPP

//#define TEST_HARNESS

#include <random>
#include <memory>
#include <vector>
#include <list>
#include <algorithm>
#ifndef TEST_HARNESS
#include "os_portability.hpp"
#else
using namespace std;
#define PORTABLE_ASSERT
#endif


class AntiInterference {
private:
    std::pair<int32_t, int32_t> freq_range{-1, -1};
    int num_timing_offsets;
    int cur_seed;
    int max_pwr_diff;
    int num_channels;
    uint8_t ttl;
    static constexpr int SEQUENCE_LEN = 32;
public:
    AntiInterference(const std::pair<int32_t, int32_t> my_freq_range, const int my_num_timing_offsets,
                        const int my_cur_seed, const int my_max_pwr_diff, const int my_num_channels) {
        freq_range = my_freq_range;
        num_timing_offsets = my_num_timing_offsets;
        cur_seed = my_cur_seed;
        max_pwr_diff = my_max_pwr_diff;
        num_channels = my_num_channels;
        ttl = 0;
        PORTABLE_ASSERT(freq_range.first < freq_range.second);
        PORTABLE_ASSERT(abs(freq_range.first-freq_range.second) > 0);
    }

    static auto seqLen() -> int {
        return SEQUENCE_LEN;
    }

    auto freqRange() -> std::pair<int32_t, int32_t> {
        return freq_range;
    }

    auto numTimingOffsets() const -> int {
        return num_timing_offsets;
    }

    auto maxPwrDiff() const -> int {
        return max_pwr_diff;
    } 

    auto numChannels() const -> int {
        return num_channels;
    }

    void setTTL(const uint8_t ttl_val) {
        ttl = ttl_val % SEQUENCE_LEN;
    }

    auto getTTL() const -> uint8_t {
        return ttl; 
    }

    virtual auto timingOffset() -> uint8_t = 0;
    virtual auto freqOffset() -> int32_t = 0;
    virtual auto pwrDiff() -> int8_t = 0;
    virtual auto nextChannel() -> int8_t = 0;
    virtual auto invertBits() -> bool = 0;
};


class AntiInterferenceRand : public AntiInterference {
private:
    std::mt19937 timing_rand;
    std::shared_ptr<std::uniform_int_distribution<uint8_t>> timing_off_dist_sptr;
    std::mt19937 freq_rand;
    std::shared_ptr<std::uniform_int_distribution<int32_t>> freq_off_dist_sptr;
    std::mt19937 pwr_rand;
    std::shared_ptr<std::uniform_int_distribution<int8_t>> pwr_off_dist_sptr;
    std::mt19937 chan_rand;
    std::shared_ptr<std::uniform_int_distribution<int8_t>> chan_dist_sptr;
    std::mt19937 inv_rand;
    std::shared_ptr<std::uniform_int_distribution<uint8_t>> inv_dist_sptr;

public:
    AntiInterferenceRand(const std::pair<int32_t, int32_t> my_freq_range, const int my_num_timing_offsets, //NOLINT
                        const int my_cur_seed, const int my_max_pwr_diff, const int my_num_channels) :
        AntiInterference(my_freq_range, my_num_timing_offsets, my_cur_seed, my_max_pwr_diff,
                            my_num_channels) { 
        timing_rand.seed(my_cur_seed);
        freq_rand.seed(my_cur_seed);
        pwr_rand.seed(my_cur_seed);
        chan_rand.seed(my_cur_seed);
        inv_rand.seed(my_cur_seed);

        freq_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int32_t>>(my_freq_range.first, my_freq_range.second);
        timing_off_dist_sptr = std::make_shared<std::uniform_int_distribution<uint8_t>>(0, my_num_timing_offsets-1);
        pwr_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int8_t>>(0, my_max_pwr_diff);
        chan_dist_sptr = std::make_shared<std::uniform_int_distribution<int8_t>>(0, my_num_channels-1);
        inv_dist_sptr = std::make_shared<std::uniform_int_distribution<uint8_t>>(0, 1);
    }

    auto timingOffset() -> uint8_t override {
        return (*timing_off_dist_sptr)(timing_rand);
    }

    auto freqOffset() -> int32_t override {
        return (*freq_off_dist_sptr)(freq_rand);        
    }

    auto pwrDiff() -> int8_t override {
        return (*pwr_off_dist_sptr)(pwr_rand);        
    }

    auto nextChannel() -> int8_t override {
        return (*chan_dist_sptr)(chan_rand);  
    }

    auto invertBits() -> bool override {
        return (*chan_dist_sptr)(chan_rand) != 0;
    }

};


class AntiInterferenceWalsh : public AntiInterference {
private:
    struct walsh_fields {
        uint32_t freq_off;
        uint32_t timing_off;
        uint32_t channel;
        uint32_t inv_bits;
    };
    std::vector<walsh_fields> walsh_sequence;
    int walsh_seq_idx;
    int freq_range_adj;
    std::mt19937 pwr_rand;
    std::shared_ptr<std::uniform_int_distribution<int8_t>> pwr_off_dist_sptr;
    std::mt19937 freq_rand;
    std::shared_ptr<std::uniform_int_distribution<int32_t>> freq_off_dist_sptr;
    std::pair<int32_t, int32_t> freq_range_noerr;

    static void load_field(std::list<bool> &slice, uint32_t &field, int num_bits);
public:
    AntiInterferenceWalsh(std::pair<int32_t, int32_t> freq_range, int num_timing_offsets,
                        int cur_seed, int max_pwr_diff, int num_channels);
    auto timingOffset() -> uint8_t override;
    auto freqOffset() -> int32_t override;
    auto pwrDiff() -> int8_t override;
    auto nextChannel() -> int8_t override;
    auto invertBits() -> bool override;
};


#endif /* ANTI_INTERFERENCE_HPP */
