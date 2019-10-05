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
#include "serial_data.hpp"
#include <math.h>

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
    size_t my_msg_size;
    // Reed-Solomon parameters
    size_t block_length;
    size_t min_distance;
    size_t message_length;    
    correct_convolutional *corr_con;
    size_t num_rs_bytes;
    // Convolutional coding parameters
    size_t inv_rate;
    size_t order;

    // Helper methods
    uint8_t extractBit(const uint8_t *buf, const size_t bit_pos);
    void writeBit(uint8_t *buf, const size_t bit_pos, const uint8_t bit_val);

    size_t encodeRSV(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg);

    ssize_t decodeRSV(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(dec_msg.size() == my_msg_size);
        dec_msg.resize(getEncSizeConv(my_msg_size));
        return correct_convolutional_decode(corr_con, enc_msg.data(), enc_msg.size()*8, dec_msg.data());
    }

    size_t encodeConv(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        MBED_ASSERT(msg.size() == my_msg_size);
        enc_msg.resize(getEncSizeConv(my_msg_size));
        return correct_convolutional_encode(corr_con, msg.data(), my_msg_size, enc_msg.data());
    }

    ssize_t decodeConv(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(enc_msg.size() == getEncSizeConv(my_msg_size));
        dec_msg.resize(my_msg_size);
        return correct_convolutional_decode(corr_con, enc_msg.data(), enc_msg.size()*8, dec_msg.data());
    }

    size_t getEncSizeConv(const size_t msg_len) {
        return correct_convolutional_encode_len(corr_con, msg_len)/8;
    }

public:
    FEC(const size_t inv_rate, const size_t order);

    virtual size_t getEncSize(size_t const msg_len) = 0;

    virtual size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) = 0;

    virtual ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) = 0;    

    virtual void benchmark(size_t num_iters) = 0;

};


class FECRSV: public FEC {
protected:
    size_t rs_corr_bytes;
    correct_reed_solomon *rs_con;
    vector<uint8_t> rs_buf;

public:
    FECRSV(const size_t inv_rate, const size_t order, const size_t rs_corr_bytes) 
        : FEC(inv_rate, order) {
        rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,
                                                  1, 1, rs_corr_bytes);
        rs_buf.resize(getEncSize(Frame::size()));
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        MBED_ASSERT(getEncSize(msg.size()) <= 256);
        MBED_ASSERT(msg.size() == my_msg_size);
        enc_msg.resize(getEncSize(msg.size()));
        size_t conv_len = correct_convolutional_encode(corr_con, msg.data(), msg.size(), rs_buf.data())/8;
        return correct_reed_solomon_encode(rs_con, rs_buf.data(), conv_len, enc_msg.data());
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(enc_msg.size() == getEncSize(my_msg_size));
        size_t rs_len = correct_reed_solomon_decode(rs_con, enc_msg.data(), enc_msg.size(), rs_buf.data());
        dec_msg.resize(my_msg_size);
        size_t conv_bytes = correct_convolutional_decode(corr_con, rs_buf.data(), rs_len*8, dec_msg.data());
        MBED_ASSERT(conv_bytes != -1);
        MBED_ASSERT(Frame::size() == conv_bytes);
        return conv_bytes;
    }

    size_t getEncSize(const size_t msg_len) {
        size_t enc_size = getEncSizeConv(msg_len) + rs_corr_bytes;
        MBED_ASSERT(enc_size <= 256);
        return enc_size;
    }

    void benchmark(const size_t num_iters);

    ~FECRSV() {
        correct_convolutional_destroy(corr_con);
        correct_reed_solomon_destroy(rs_con);
    }
};


class FECConv: public FEC {
public:    
    FECConv(const size_t inv_rate, const size_t order) : FEC(inv_rate, order) {}

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        MBED_ASSERT(msg.size() == my_msg_size);
        return encodeConv(msg, enc_msg);
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(enc_msg.size() == getEncSizeConv(my_msg_size));
        return decodeConv(enc_msg, dec_msg);
    }

    size_t getEncSize(const size_t msg_len) {
        return getEncSizeConv(msg_len);
    }

    void benchmark(const size_t num_iters);

    ~FECConv() {
        correct_convolutional_destroy(corr_con);
    }
};

#endif /* FEC_HPP */