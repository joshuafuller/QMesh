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
    AntiInterferenceRand(const std::pair<int32_t, int32_t> my_freq_range, const int my_num_timing_offsets,
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
    
    static void load_field(list<bool> &slice, uint32_t &field, const int num_bits)  {
        field = 0;
        for(int i = 0; i < num_bits; i++) {
            field <<= 1U;
            field |= *(slice.begin()) ? 0x1U : 0x0U;
            slice.pop_front();
        }
    }

    void refresh() {
        MBED_ASSERT(!walsh_fresh.freq);
        MBED_ASSERT(!walsh_fresh.channel);
        MBED_ASSERT(!walsh_fresh.pwr);
        MBED_ASSERT(!walsh_fresh.timing);
        walsh_seq_idx += 1;
        walsh_seq_idx %= SEQUENCE_LEN;
        walsh_fresh.freq = true;
        walsh_fresh.channel = true;
        walsh_fresh.pwr = true;
        walsh_fresh.timing = true;
    }
public:
    AntiInterferenceWalsh(const std::pair<int32_t, int32_t> freq_range, const int num_timing_offsets,
                        const int cur_seed, const int max_pwr_diff, const int num_channels) :
        AntiInterference(freq_range, num_timing_offsets, cur_seed, max_pwr_diff, num_channels) {
        num_freq_range_bits = ceil(log2(std::abs(freq_range.first-freq_range.second)));
        num_timing_offset_bits = ceil(log2(num_timing_offsets));
        num_pwr_diff_bits = ceil(log2(max_pwr_diff));
        num_channels_bits = ceil(log2(num_channels));
        total_bits = num_freq_range_bits+num_timing_offset_bits+num_pwr_diff_bits+num_channels_bits;
        walsh_seq_idx = 0;
        walsh_fresh.freq = true;
        walsh_fresh.channel = true;
        walsh_fresh.pwr = true;
        walsh_fresh.timing = true;
        MBED_ASSERT(total_bits <= 32);

        auto cur_seed_u32 = static_cast<uint32_t>(cur_seed);
        vector<bool> walsh_code;
        constexpr int BITS_PER_ELEM = 32;
        uint32_t walsh_iters = BITS_PER_ELEM + static_cast<int>(ceil(log2(SEQUENCE_LEN)));
        MBED_ASSERT(walsh_iters < 2);
        // First pair of bits
        if((cur_seed_u32 & 0x1U) == 0) {
            walsh_code.push_back(false);
            walsh_code.push_back(false);
        } else {
            walsh_code.push_back(false);
            walsh_code.push_back(true);            
        }
        for(uint32_t i = 0; i < walsh_iters-1; i++) {
            vector<bool> walsh_code_tmp = walsh_code;
            if(((cur_seed_u32 >> (walsh_iters+1)) & 0x1U) == 0) {
                for(bool && it : walsh_code_tmp) {
                    walsh_code.push_back(it);
                }
            } else {
                for(bool && it : walsh_code_tmp) {
                    walsh_code.push_back(!it);
                }
            }
        }
        for(int i = 0; i < SEQUENCE_LEN; i++) {
            list<bool> slice(BITS_PER_ELEM);
            copy(walsh_code.begin()+BITS_PER_ELEM*i, walsh_code.begin()+BITS_PER_ELEM*(i+1), slice.end());
            // Break up the slice into fields
            walsh_fields fields{};
            load_field(slice, fields.freq_off, num_freq_range_bits);
            load_field(slice, fields.timing_off, num_timing_offset_bits);
            load_field(slice, fields.pwr_off, num_pwr_diff_bits);
            load_field(slice, fields.channel, num_channels_bits);
            walsh_sequence.push_back(fields);
        }
        MBED_ASSERT(walsh_code.empty());
    }

    auto timingOffset() -> uint8_t override {
        if(!walsh_fresh.timing) {
            refresh();
        }
        walsh_fresh.timing = false;
        return walsh_sequence[walsh_seq_idx].timing_off;
    }

    auto freqOffset() -> int32_t override {
        if(!walsh_fresh.freq) {
            refresh();
        }
        walsh_fresh.freq = false;
        return walsh_sequence[walsh_seq_idx].freq_off; 
    }

    auto pwrDiff() -> int8_t override {
        if(!walsh_fresh.pwr) {
            refresh();
        }
        walsh_fresh.pwr = false;
        return walsh_sequence[walsh_seq_idx].pwr_off;     
    }

    auto nextChannel() -> int8_t override {
        if(!walsh_fresh.channel) {
            refresh();
        }
        walsh_fresh.channel = false;
        return walsh_sequence[walsh_seq_idx].channel;
    }

};


#endif /* ANTI_INTERFERENCE_HPP */