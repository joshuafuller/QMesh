#include "anti_interference.hpp"
#include <cstdio>

AntiInterferenceWalsh::AntiInterferenceWalsh(const std::pair<int32_t, int32_t> freq_range, //NOLINT
                            const int num_timing_offsets,
                            const int cur_seed, 
                            const int max_pwr_diff, 
                            const int num_channels) : 
    AntiInterference(freq_range, num_timing_offsets, cur_seed, max_pwr_diff, num_channels) {
    pwr_rand.seed(cur_seed);
    pwr_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int8_t>>(0, max_pwr_diff);
    freq_rand.seed(cur_seed);
    constexpr int32_t FREQ_ERR = (3.0*433e6) / 1e6;
    freq_off_dist_sptr = std::make_shared<std::uniform_int_distribution<int32_t>>(-FREQ_ERR, FREQ_ERR);
    freq_range_noerr = freq_range;
    freq_range_noerr.first += FREQ_ERR;
    freq_range_noerr.second -= FREQ_ERR;
    int num_freq_range_bits = ceil(log2(std::abs(freq_range_noerr.first-freq_range_noerr.second)));
    freq_range_adj = freq_range_noerr.second;
    int num_timing_offset_bits = ceil(log2(num_timing_offsets));
    int num_channels_bits = ceil(log2(num_channels));
    int num_inv_bits = 1;
    int total_bits = num_freq_range_bits+num_timing_offset_bits+num_channels_bits+num_inv_bits;
    walsh_seq_idx = 0;
    constexpr int BITS_PER_ELEM = 32;
    PORTABLE_ASSERT(total_bits <= BITS_PER_ELEM);

    auto cur_seed_u32 = static_cast<uint32_t>(cur_seed);
    vector<bool> walsh_code;
    uint32_t walsh_iters = static_cast<int>(ceil(log2(BITS_PER_ELEM * seqLen())));
    PORTABLE_ASSERT(walsh_iters > 1);
    PORTABLE_ASSERT(walsh_iters <= BITS_PER_ELEM);
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
    PORTABLE_ASSERT(walsh_code.size() == static_cast<size_t>(BITS_PER_ELEM*seqLen()));
    for(int i = 0; i < seqLen(); i++) {
        list<bool> slice(BITS_PER_ELEM);
        PORTABLE_ASSERT(walsh_code.begin()+BITS_PER_ELEM*i < walsh_code.end());
        copy(walsh_code.begin()+BITS_PER_ELEM*i, walsh_code.begin()+BITS_PER_ELEM*(i+1), slice.begin());
        // Break up the slice into fields
        walsh_fields fields{};
        load_field(slice, fields.channel, num_channels_bits);
        load_field(slice, fields.inv_bits, num_inv_bits);
        load_field(slice, fields.freq_off, num_freq_range_bits);
        load_field(slice, fields.timing_off, num_timing_offset_bits);
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
    uint8_t ret_val = 0;
    ret_val = walsh_sequence[getTTL()].timing_off;
    if(ret_val == 0) {
        ret_val = 0;
    } else if(ret_val > numTimingOffsets()-1) {
        ret_val = numTimingOffsets()-1;
    }
    return ret_val;
}

auto AntiInterferenceWalsh::freqOffset() -> int32_t {
    int32_t ret_val = static_cast<int32_t>(walsh_sequence[getTTL()].freq_off) - freq_range_adj;
    PORTABLE_ASSERT(freq_range_noerr.first != -1);
    PORTABLE_ASSERT(freq_range_noerr.second != -1);
    if(ret_val < freq_range_noerr.first) {
        ret_val = freq_range_noerr.first;
    }
    if(ret_val > freq_range_noerr.second) {
        ret_val = freq_range_noerr.second;
    }
    ret_val += (*freq_off_dist_sptr)(freq_rand);
    return ret_val; 
}

auto AntiInterferenceWalsh::pwrDiff() -> int8_t {
    return (*pwr_off_dist_sptr)(pwr_rand);        
}

auto AntiInterferenceWalsh::nextChannel() -> int8_t {
    int8_t ret_val = 0;
    ret_val = walsh_sequence[getTTL()].channel;
    if(numChannels() == 0) {
        ret_val = 0;
    } else if(ret_val > numChannels()-1) {
        ret_val = numChannels()-1;
    }  
    return ret_val;
}

auto AntiInterferenceWalsh::invertBits() -> bool {
    int8_t ret_val = 0;
    ret_val = walsh_sequence[getTTL()].inv_bits;
    return ret_val != 0;
}


#ifdef TEST_HARNESS
int main(int argc, char **argv) {
    AntiInterferenceWalsh anti_intr(pair<int32_t, int32_t>(-90000, 90000), 8,
                            atoi(argv[1]), 3, 4);
    for(int i = 0; i < 32; i++) {
        anti_intr.setTTL(i);
        printf("freq %d\n", anti_intr.freqOffset());
        printf("chan %d\n", anti_intr.nextChannel());
        printf("pwr %d\n", anti_intr.pwrDiff());
        printf("tmng %d\n", anti_intr.timingOffset()); 
        printf("inv %d\n", anti_intr.invertBits());         
    }   
}
#endif
