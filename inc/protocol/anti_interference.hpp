#ifndef ANTI_INTERFERENCE_HPP
#define ANTI_INTERFERENCE_HPP

#include <random>
#include <memory>
#include <vector>
#include <list>
#include <algorithm>
#include "mbed.h"


class AntiInterference {
private:
    std::pair<int32_t, int32_t> freq_range;
    int num_timing_offsets;
    int cur_seed;
    int max_pwr_diff;
    int num_channels;
public:
    AntiInterference(const std::pair<int32_t, int32_t> my_freq_range, const int my_num_timing_offsets,
                        const int my_cur_seed, const int my_max_pwr_diff, const int my_num_channels) {
        freq_range = my_freq_range;
        num_timing_offsets = my_num_timing_offsets;
        cur_seed = my_cur_seed;
        max_pwr_diff = my_max_pwr_diff;
        num_channels = my_num_channels;
    }

    virtual auto timingOffset() -> uint8_t = 0;

    virtual auto freqOffset() -> int32_t = 0;

    virtual auto pwrDiff() -> int8_t = 0;

    virtual auto nextChannel() -> int8_t = 0;
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

public:
    AntiInterferenceRand(const std::pair<int32_t, int32_t> my_freq_range, const int my_num_timing_offsets, //NOLINT
                        const int my_cur_seed, const int my_max_pwr_diff, const int my_num_channels) :
        AntiInterference(my_freq_range, my_num_timing_offsets, my_cur_seed, my_max_pwr_diff,
                            my_num_channels) { 
        timing_rand.seed(my_cur_seed);
        freq_rand.seed(my_cur_seed);
        pwr_rand.seed(my_cur_seed);
        chan_rand.seed(my_cur_seed);

        freq_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int32_t>>(my_freq_range.first, my_freq_range.second);
        timing_off_dist_sptr = std::make_shared<std::uniform_int_distribution<uint8_t>>(0, my_num_timing_offsets-1);
        pwr_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int8_t>>(0, my_max_pwr_diff);
        pwr_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int8_t>>(0, my_num_channels-1);
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

};


class AntiInterferenceWalsh : public AntiInterference {
private:
    int num_freq_range_bits;
    int num_timing_offset_bits;
    int num_pwr_diff_bits;
    int num_channels_bits;
    int total_bits;
    struct fields_fresh {
        bool freq;
        bool timing;
        bool pwr;
        bool channel;
    } walsh_fresh{};
    struct walsh_fields {
        uint32_t freq_off;
        uint32_t timing_off;
        uint32_t pwr_off;
        uint32_t channel;
    };
    vector<walsh_fields> walsh_sequence;
    int walsh_seq_idx;
    static constexpr int SEQUENCE_LEN = 128;
    int freq_range_adj;

    static void load_field(list<bool> &slice, uint32_t &field, int num_bits);
    void refresh();
public:
    AntiInterferenceWalsh(pair<int32_t, int32_t> freq_range, int num_timing_offsets,
                        int cur_seed, int max_pwr_diff, int num_channels);
    auto timingOffset() -> uint8_t override;
    auto freqOffset() -> int32_t override;
    auto pwrDiff() -> int8_t override;
    auto nextChannel() -> int8_t override;
};


#endif /* ANTI_INTERFERENCE_HPP */