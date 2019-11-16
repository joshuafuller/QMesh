/*
QMesh
Copyright (C) 2019 Daniel R. Fay

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FEC_HPP
#define FEC_HPP

#include "mbed.h"
#include "correct.h"
#include "params.hpp"
#include <cmath>
#include <map>
#include "PolarCode.h"
#include <string>
#include "MbedJSONValue.h"
#include "golay.h"

extern MbedJSONValue radio_cb;

// Convolutional Codes

// Polynomials
// These have been determined via find_conv_libfec_poly.c
// We could just make up new ones, but we use libfec's here so that
//   codes encoded by this library can be decoded by the original libfec
//   and vice-versa
#define V27POLYA 0155
#define V27POLYB 0117
static correct_convolutional_polynomial_t libfec_r12_7_polynomial[] = {V27POLYA, V27POLYB};

#define V29POLYA 0657
#define V29POLYB 0435
static correct_convolutional_polynomial_t libfec_r12_9_polynomial[] = {V29POLYA, V29POLYB};

#define V39POLYA 0755
#define V39POLYB 0633
#define V39POLYC 0447
static correct_convolutional_polynomial_t libfec_r13_9_polynomial[] = {V39POLYA, V39POLYB, V39POLYC};

#define V615POLYA 042631
#define V615POLYB 047245
#define V615POLYC 056507
#define V615POLYD 073363
#define V615POLYE 077267
#define V615POLYF 064537
static correct_convolutional_polynomial_t libfec_r16_15_polynomial[] = {V615POLYA, V615POLYB, V615POLYC,
                        V615POLYD, V615POLYE, V615POLYF};

static correct_convolutional_polynomial_t conv_r12_6_polynomial[] = {073, 061};
static correct_convolutional_polynomial_t conv_r12_7_polynomial[] = {0161, 0127};
static correct_convolutional_polynomial_t conv_r12_8_polynomial[] = {0225, 0373};
static correct_convolutional_polynomial_t conv_r12_9_polynomial[] = {0767, 0545};
static correct_convolutional_polynomial_t conv_r13_6_polynomial[] = {053, 075, 047};
static correct_convolutional_polynomial_t conv_r13_7_polynomial[] = {0137, 0153,
                                                                                   0121};
static correct_convolutional_polynomial_t conv_r13_8_polynomial[] = {0333, 0257,
                                                                                   0351};
static correct_convolutional_polynomial_t conv_r13_9_polynomial[] = {0417, 0627,
                                                                                   0675};                        

// This class provides a way to magically apply forward error correction to 
//  a character array provided.
class FEC {
protected:
    string name;

public:
    FEC(void) {
        name = "Dummy FEC";
    }

    static float getBER(const float snr);

    static uint8_t getNibble(const vector<uint8_t> &bytes, const size_t pos) {
        uint8_t my_byte = bytes[pos/2];
        if(pos%2 == 0) {
            return my_byte & 0x0F;
        }
        else {
            return (my_byte >> 4) & 0x0F;
        }
    }

    static void setNibble(const uint8_t nib, const size_t pos, vector<uint8_t> &bytes) {
        uint8_t my_byte = bytes[pos/2];
        if(pos%2 == 0) {
            my_byte &= 0xF0;
            my_byte |= nib;
        }
        else {
            my_byte &= 0x0F;
            my_byte |= (nib << 4);
        }
        bytes[pos/2] = my_byte;
    }

    static bool lowDataRate(const uint32_t bw, const uint8_t sf) {
        // 0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved
        const float bw_idx_bw_f[] = {125e3f, 250e3f, 500e3f};
        float bw_f = bw_idx_bw_f[bw];
        float sf_f = sf;
        float sym_time_s = powf(2, sf_f)/bw_f;
        float sym_time_ms = sym_time_s*1e3f;

        // Determine whether we need the low datarate optimize
        return sym_time_ms >= 16.f ? true : false;
    }

    static void interleave(const vector<uint8_t> &buf, vector<uint8_t> &int_buf) {
        int sf = radio_cb["SF"].get<int>();
        int bw = radio_cb["BW"].get<int>();
        int nibs_per_sym = sf - (!lowDataRate(bw,sf) ? 0 : 2);
        // Special case where there's only enough nibbles to cover one
        //  set of symbols
        if(buf.size()*2 <= nibs_per_sym) {
            int_buf = buf;
            return;
        }
        int extra_nibs = nibs_per_sym - (buf.size()*2 % nibs_per_sym);
        int extra_bytes = extra_nibs/2 + extra_nibs%2;
        vector<uint8_t> input_buf = buf;
        for(int i = 0; i < extra_bytes; i++) {
            input_buf.push_back(0xFF);
        }
        int_buf.resize(buf.size()+extra_bytes);

        for(int i, j, k = 0; i < input_buf.size()*2; i++, j += nibs_per_sym) {
            if(j > input_buf.size()*2) {
                j = ++k;
            }
            uint8_t input_nib = getNibble(input_buf, i);
            setNibble(input_nib, j, int_buf);
        }
    }

    static void deinterleave(const vector<uint8_t> &buf, vector<uint8_t> &deint_buf) {
        int sf = radio_cb["SF"].get<int>();
        int bw = radio_cb["BW"].get<int>();
        int nibs_per_sym = sf - (!lowDataRate(bw,sf) ? 0 : 2);
        // Special case where there's only enough nibbles to cover one
        //  set of symbols
        if(buf.size()*2 <= nibs_per_sym) {
            deint_buf = buf;
            return;
        }
        deint_buf.resize(buf.size());

        for(int i, j, k = 0; i < buf.size()*2; i++, j += nibs_per_sym) {
            if(j > buf.size()*2) {
                j = ++k;
            }
            uint8_t input_nib = getNibble(buf, j);
            setNibble(input_nib, i, deint_buf);
        }
    }

    virtual size_t getEncSize(const size_t msg_len) {
        return msg_len;
    }

    virtual size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        enc_msg = msg;
        return msg.size();
    }

    virtual ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        dec_msg = enc_msg;
        return dec_msg.size();
    }
 
    virtual ssize_t decodeSoft(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg, const float snr) {
        return decode(enc_msg, dec_msg);
    }

    void benchmark(size_t num_iters);
};


class FECInterleave : public FEC {
public:
    FECInterleave(void) {
        name = "Dummy Interleaver";   
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        FEC::interleave(msg, enc_msg);
        return enc_msg.size();
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        FEC::deinterleave(enc_msg, dec_msg);
        return dec_msg.size();
    }  
};


class FECGolay : public FEC {
public:
    FECGolay(void) {
        name = "Golay";
    }

    size_t getEncSize(const size_t msg_len) {
        int extra_nibs = (msg_len*2) % 3;
        if(extra_nibs) {
            return msg_len*2 + (3-extra_nibs);
        }
        return msg_len*2;
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        enc_msg.resize(getEncSize(msg.size()));
        vector<uint8_t> input_msg = msg;
        vector<uint8_t> pre_int_msg;
        input_msg.resize(getEncSize(msg.size()));
        for(int i = 0; i < input_msg.size()*2; i += 3) {
            uint32_t input_word = FEC::getNibble(input_msg, i);
            input_word <<= 4;
            input_word |= FEC::getNibble(input_msg, i+1);
            input_word <<= 4;
            input_word |= FEC::getNibble(input_msg, i+2);
            uint32_t enc_word = golay_encode(input_word);
            pre_int_msg.push_back(enc_word & 0xFF);
            pre_int_msg.push_back((enc_word >> 8) & 0xFF);
            pre_int_msg.push_back((enc_word >> 16) & 0xFF);
        }
        FEC::interleave(pre_int_msg, enc_msg);
        return enc_msg.size();
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        vector<uint8_t> deint_msg(enc_msg.size());
        dec_msg.resize(enc_msg.size()/2);
        FEC::deinterleave(enc_msg, deint_msg);
        bool decode_success = true;
        for(int i = 0; i < deint_msg.size(); i += 3) {
            uint32_t input_word = deint_msg[i];
            input_word <<= 8;
            input_word |= deint_msg[i+1];
            input_word <<= 8;
            input_word |= deint_msg[i+2];
            int32_t decode_val = golay_decode(input_word);
            if(decode_val == -1) {
                decode_success = false;
            }
            FEC::setNibble((decode_val>>16) & 0xFF, i, dec_msg);
            FEC::setNibble((decode_val>>8) & 0xFF, i, dec_msg);
            FEC::setNibble((decode_val) & 0xFF, i, dec_msg);
        }
        if(decode_success) {
            return dec_msg.size();
        }
        return -1;
    }  
};


class FECPolar : public FEC {
protected:
    shared_ptr<PolarCodeLib::PolarCode> polar_code;
    size_t block_length;
    size_t info_length;
    size_t list_size;
public:
    FECPolar(const size_t my_block_length, const size_t my_info_length, const size_t my_list_size) {
        name = "Polar Codes";
        block_length = my_block_length;
        info_length = my_info_length;
        list_size = my_list_size;
        polar_code = make_shared<PolarCodeLib::PolarCode>(PolarCodeLib::PolarCode(block_length, info_length, 0.32, 2));
    }

    size_t getEncSize(const size_t msg_len) {
        return (1 << block_length);
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        //std::vector<uint8_t> encode(std::vector<uint8_t> info_bits);
        enc_msg = polar_code->encode(msg);
        return enc_msg.size();
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        vector<float> p0, p1;
        for(vector<uint8_t>::const_iterator iter = enc_msg.begin(); iter != enc_msg.end(); iter++) {
            uint8_t my_byte = *iter;
            for(int i = 0; i < 8; i++) {
                if(my_byte & 0x1) {
                    p0.push_back(0.0);
                    p1.push_back(1.0);
                }
                else {
                    p0.push_back(1.0);
                    p1.push_back(0.0);
                }
                my_byte >>= 1;
            }
        }
        //int list_size = radio_cb["Polar List Size"].get<int>();
        dec_msg = polar_code->decode_scl_p1(p1, p0, list_size);
        return dec_msg.size();
    }

    ssize_t decodeSoft(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg, const float snr) {
        vector<float> p0, p1;
        for(vector<uint8_t>::const_iterator iter = enc_msg.begin(); iter != enc_msg.end(); iter++) {
            uint8_t my_byte = *iter;
            for(int i = 0; i < 8; i++) {
                if(my_byte & 0x1) {
                    p0.push_back(0.0);
                    p1.push_back(1.0);
                }
                else {
                    p0.push_back(1.0);
                    p1.push_back(0.0);
                }
                my_byte >>= 1;
            }
        }
        dec_msg = polar_code->decode_scl_p1(p1, p0, list_size);
        return dec_msg.size();
    }
};


class FECConv: public FEC {
protected:
    // Convolutional coding parameters
    size_t inv_rate;
    size_t order;
    correct_convolutional *corr_con;    

public:    
    FECConv(void) : FECConv(2, 9) { }

    FECConv(const size_t inv_rate, const size_t order);

    ~FECConv(void) {
        correct_convolutional_destroy(corr_con);
    }

    size_t getEncSize(const size_t msg_len) {
        return correct_convolutional_encode_len(corr_con, msg_len)/8;
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        enc_msg.resize(FECConv::getEncSize(msg.size()));
        vector<uint8_t> msg_int(msg.size());
        interleave(msg, msg_int);
        return correct_convolutional_encode(corr_con, msg_int.data(), msg_int.size(), enc_msg.data())/8;
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        dec_msg.resize(enc_msg.size());
        vector<uint8_t> dec_msg_int(dec_msg.size());
        size_t dec_size = correct_convolutional_decode(corr_con, enc_msg.data(), 
                                enc_msg.size()*8, dec_msg_int.data());
        dec_msg.resize(dec_size);
        deinterleave(dec_msg_int, dec_msg);
        return dec_size;
    }

};


class FECRSV: public FECConv {
protected:
    size_t rs_corr_bytes;
    correct_reed_solomon *rs_con;
    vector<uint8_t> rs_buf;

public:
    FECRSV(const size_t inv_rate, const size_t order, const size_t my_rs_corr_bytes) 
        : FECConv(inv_rate, order) {
        name = "RSV";
        rs_corr_bytes = my_rs_corr_bytes;
        rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,
                                                  1, 1, rs_corr_bytes);
    }

    FECRSV(void) : FECRSV(2, 9, 32) { };

    ~FECRSV(void ) {
        correct_reed_solomon_destroy(rs_con);
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &rsv_enc_msg);

    ssize_t decode(const vector<uint8_t> &rsv_enc_msg, vector<uint8_t> &dec_msg);

    size_t getEncSize(const size_t msg_len) {
        size_t enc_size = FECConv::getEncSize(msg_len+rs_corr_bytes);
        MBED_ASSERT(enc_size <= 256);
        return enc_size;
    }
};

#endif /* FEC_HPP */