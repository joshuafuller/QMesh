#include "anti_interference.hpp"
#include <cstdio>

AntiInterferenceWalsh::AntiInterferenceWalsh(const std::pair<int32_t, int32_t> freq_range, \
                            const int num_timing_offsets,
                            const int cur_seed, 
                            const int max_pwr_diff, 
                            const int num_channels) :
    AntiInterference(freq_range, num_timing_offsets, cur_seed, max_pwr_diff, num_channels) {
    num_freq_range_bits = ceil(log2(std::abs(freq_range.first-freq_range.second)));
    freq_range_adj = freq_range.second;
    num_timing_offset_bits = ceil(log2(num_timing_offsets));
    num_pwr_diff_bits = ceil(log2(max_pwr_diff));
    num_channels_bits = ceil(log2(num_channels));
    total_bits = num_freq_range_bits+num_timing_offset_bits+num_pwr_diff_bits+num_channels_bits;
    printf("diff bits %d %d %d %d\r\n", num_freq_range_bits, num_timing_offset_bits, num_pwr_diff_bits, num_channels_bits);
    walsh_seq_idx = 0;
    walsh_fresh.freq = true;
    walsh_fresh.channel = true;
    walsh_fresh.pwr = true;
    walsh_fresh.timing = true;
    MBED_ASSERT(total_bits <= 32);

    auto cur_seed_u32 = static_cast<uint32_t>(cur_seed);
    printf("seed %8x\r\n", cur_seed_u32);
    vector<bool> walsh_code;
    constexpr int BITS_PER_ELEM = 32;
    uint32_t walsh_iters = static_cast<int>(ceil(log2(BITS_PER_ELEM * SEQUENCE_LEN)));
    printf("walsh iters %d\r\n", walsh_iters);
    MBED_ASSERT(walsh_iters > 1);
    MBED_ASSERT(walsh_iters <= 32);
    // First pair of bits
    walsh_code.push_back(false);
    for(uint32_t i = 0; i < walsh_iters; i++) {
        vector<bool> walsh_code_tmp = walsh_code;
        if(((cur_seed_u32 >> i) & 0x1U) == 0) {
            for(bool && it : walsh_code_tmp) {
                walsh_code.push_back(it);
            }
        } else {
            for(bool && it : walsh_code_tmp) {
                walsh_code.push_back(!it);
            }
        }
    }
    MBED_ASSERT(walsh_code.size() == BITS_PER_ELEM*SEQUENCE_LEN);
    printf("num bits %d\r\n", walsh_code.size());
    for(int i = 0; i < SEQUENCE_LEN; i++) {
        list<bool> slice(BITS_PER_ELEM);
        MBED_ASSERT(walsh_code.begin()+BITS_PER_ELEM*i < walsh_code.end());
        copy(walsh_code.begin()+BITS_PER_ELEM*i, walsh_code.begin()+BITS_PER_ELEM*(i+1), slice.begin());
        // Break up the slice into fields
        walsh_fields fields{};
        load_field(slice, fields.channel, num_channels_bits);
        load_field(slice, fields.freq_off, num_freq_range_bits);
        load_field(slice, fields.timing_off, num_timing_offset_bits);
        load_field(slice, fields.pwr_off, num_pwr_diff_bits);
        walsh_sequence.push_back(fields);
    }
}

void AntiInterferenceWalsh::load_field(list<bool> &slice, uint32_t &field, const int num_bits)  {
    field = 0;
    for(int i = 0; i < num_bits; i++) {
        field <<= 1U;
        field |= *(slice.begin()) ? 0x1U : 0x0U;
        slice.pop_front();
    }
}

auto AntiInterferenceWalsh::timingOffset() -> uint8_t {
    uint8_t ret_val = walsh_sequence[getTTL()].timing_off;
    if(ret_val == 0) {
        ret_val = 0;
    } else if(ret_val > numTimingOffsets()-1) {
        ret_val = numTimingOffsets()-1;
    }
    return ret_val;
}

auto AntiInterferenceWalsh::freqOffset() -> int32_t {
    int32_t ret_val = static_cast<int32_t>(walsh_sequence[getTTL()].freq_off) - freq_range_adj;
    if(ret_val < freqRange().first) {
        ret_val = freqRange().first;
    }
    if(ret_val > freqRange().second) {
        ret_val = freqRange().second;
    }
    return ret_val; 
}

auto AntiInterferenceWalsh::pwrDiff() -> int8_t {
    int8_t ret_val = walsh_sequence[getTTL()].pwr_off;
    if(maxPwrDiff() == 0) {
        ret_val = 0;
    } else if(ret_val > maxPwrDiff()-1) {
        ret_val = maxPwrDiff()-1;
    }
    return ret_val;     
}

auto AntiInterferenceWalsh::nextChannel() -> int8_t {
    int8_t ret_val = walsh_sequence[getTTL()].channel;
    if(numChannels() == 0) {
        ret_val = 0;
    } else if(ret_val > numChannels()-1) {
        ret_val = numChannels()-1;
    }  
    return ret_val;
}


#ifdef TEST_HARNESS
int main(int argc, char **argv) {
    AntiInterferenceWalsh anti_intr(pair<int32_t, int32_t>(-90000, 90000), 8,
                            atoi(argv[1]), 0, 1);
    for(int i = 0; i < 128; i++) {
        printf("freq %d\n", anti_intr.freqOffset());
        printf("chan %d\n", anti_intr.nextChannel());
        printf("pwr %d\n", anti_intr.pwrDiff());
        printf("tmng %d\n", anti_intr.timingOffset()); 
    }   
}
#endif