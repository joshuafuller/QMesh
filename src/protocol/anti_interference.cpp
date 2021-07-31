#include "anti_interference.hpp"

AntiInterferenceWalsh::AntiInterferenceWalsh(const std::pair<int32_t, int32_t> freq_range, const int num_timing_offsets,
                    const int cur_seed, const int max_pwr_diff, const int num_channels) :
    AntiInterference(freq_range, num_timing_offsets, cur_seed, max_pwr_diff, num_channels) {
    num_freq_range_bits = ceil(log2(std::abs(freq_range.first-freq_range.second)));
    freq_range_adj = freq_range.first;
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
    MBED_ASSERT(walsh_iters > 1);
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

void AntiInterferenceWalsh::load_field(list<bool> &slice, uint32_t &field, const int num_bits)  {
    field = 0;
    for(int i = 0; i < num_bits; i++) {
        field <<= 1U;
        field |= *(slice.begin()) ? 0x1U : 0x0U;
        slice.pop_front();
    }
}

void AntiInterferenceWalsh::refresh() {
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

auto AntiInterferenceWalsh::timingOffset() -> uint8_t {
    if(!walsh_fresh.timing) {
        refresh();
    }
    walsh_fresh.timing = false;
    return walsh_sequence[walsh_seq_idx].timing_off;
}

auto AntiInterferenceWalsh::freqOffset() -> int32_t {
    if(!walsh_fresh.freq) {
        refresh();
    }
    walsh_fresh.freq = false;
    return static_cast<int32_t>(walsh_sequence[walsh_seq_idx].freq_off) - freq_range_adj; 
}

auto AntiInterferenceWalsh::pwrDiff() -> int8_t {
    if(!walsh_fresh.pwr) {
        refresh();
    }
    walsh_fresh.pwr = false;
    return walsh_sequence[walsh_seq_idx].pwr_off;     
}

auto AntiInterferenceWalsh::nextChannel() -> int8_t {
    if(!walsh_fresh.channel) {
        refresh();
    }
    walsh_fresh.channel = false;
    return walsh_sequence[walsh_seq_idx].channel;
}