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
#include <cmath>
#include <map>
#include "PolarCode.h"


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

    size_t getEncSize(const size_t msg_len) {
        return msg_len;
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        enc_msg = msg;
        return msg.size();
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        dec_msg = enc_msg;
        return dec_msg.size();
    }
 
    ssize_t decodeSoft(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg, const float snr) {
        return decode(enc_msg, dec_msg);
    }

    void benchmark(size_t num_iters);
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
        return block_length;
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        //std::vector<uint8_t> encode(std::vector<uint8_t> info_bits);
        enc_msg = polar_code->encode(msg);
        return enc_msg.size();
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        vector<double> p0, p1;
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
        int list_size = radio_cb["Polar List Size"].get<int>();
        dec_msg = polar_code->decode_scl_p1(p1, p0, list_size);
        return dec_msg.size();
    }

    ssize_t decodeSoft(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg, const float snr) {
        vector<double> p0, p1;
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
    FECConv(const size_t inv_rate, const size_t order);

    ~FECConv(void) {
        correct_convolutional_destroy(corr_con);
    }

    size_t getEncSize(const size_t msg_len) {
        return correct_convolutional_encode_len(corr_con, msg_len)/8;
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        enc_msg.resize(getEncSize(msg.size()));
        return correct_convolutional_encode(corr_con, msg.data(), msg.size(), enc_msg.data());
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        dec_msg.resize(Frame::size());
        return correct_convolutional_decode(corr_con, enc_msg.data(), enc_msg.size()*8, dec_msg.data());
    }

};


class FECRSV: public FECConv {
protected:
    size_t rs_corr_bytes;
    correct_reed_solomon *rs_con;
    vector<uint8_t> rs_buf;

public:
    FECRSV(const size_t inv_rate, const size_t order, const size_t rs_corr_bytes) 
        : FECConv(inv_rate, order) {
        name = "RSV";
        rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,
                                                  1, 1, rs_corr_bytes);
        rs_buf.resize(getEncSize(Frame::size()));
    }

    ~FECRSV(void ) {
        correct_reed_solomon_destroy(rs_con);
    }

    size_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        MBED_ASSERT(getEncSize(msg.size()) <= 256);
        MBED_ASSERT(msg.size() == Frame::size());
        enc_msg.resize(getEncSize(msg.size()));
        vector<uint8_t> rs_buf(getEncSize(msg.size()));
        size_t conv_len = FECConv::encode(msg, rs_buf);
        return correct_reed_solomon_encode(rs_con, rs_buf.data(), conv_len, enc_msg.data());
    }

    ssize_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(enc_msg.size() == getEncSize(Frame::size()));
        vector<uint8_t> rs_buf(enc_msg.size());
        size_t rs_len = correct_reed_solomon_decode(rs_con, enc_msg.data(), enc_msg.size(), rs_buf.data());
        rs_buf.resize(rs_len);
        dec_msg.resize(Frame::size());
        size_t conv_bytes = FECConv::decode(rs_buf, dec_msg);
        MBED_ASSERT(conv_bytes != -1);
        MBED_ASSERT(Frame::size() == conv_bytes);
        return conv_bytes;
    }

    size_t getEncSize(const size_t msg_len) {
        size_t enc_size = FECConv::getEncSize(msg_len) + rs_corr_bytes;
        MBED_ASSERT(enc_size <= 256);
        return enc_size;
    }
};

#endif /* FEC_HPP */