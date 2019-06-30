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
    uint8_t *rs_int_buf;
    // Convolutional coding parameters
    size_t inv_rate;
    size_t order;

    // Helper methods
    uint8_t extractBit(const uint8_t *buf, const size_t bit_pos);
    void writeBit(uint8_t *buf, const size_t bit_pos, const uint8_t bit_val);

    size_t encodeRSV(const uint8_t *msg, const size_t msg_len, uint8_t *enc_msg);

    ssize_t decodeRSV(const uint8_t *enc_msg, const size_t enc_len, uint8_t *dec_msg) {
        return correct_convolutional_decode(corr_con, enc_msg, enc_len*8, dec_msg);
    }

    size_t encodeConv(const uint8_t *msg, const size_t msg_len, uint8_t *enc_msg) {
        return correct_convolutional_encode(corr_con, msg, msg_len, enc_msg)/8;
    }

    ssize_t decodeConv(const uint8_t *enc_msg, const size_t enc_len, uint8_t *dec_msg) {
        return correct_convolutional_decode(corr_con, enc_msg, enc_len*8, dec_msg);
    }

    size_t getEncSizeConv(const size_t msg_len) {
        return correct_convolutional_encode_len(corr_con, msg_len)/8;
    }

public:
    FEC(const size_t msg_size, const size_t inv_rate, const size_t order);

    virtual size_t getEncSize(size_t const msg_len) = 0;

    virtual size_t encode(const uint8_t *msg, const size_t msg_len, uint8_t *enc_msg) = 0;

    virtual ssize_t decode(const uint8_t *enc_msg, const size_t enc_len, uint8_t *dec_msg) = 0;    

    virtual void benchmark(size_t num_iters) = 0;

};


class FECRSV: public FEC {
protected:
    size_t rs_corr_bytes;
    correct_reed_solomon *rs_con;
    uint8_t *rs_buf;

public:
    FECRSV(const size_t msg_size, const size_t inv_rate, const size_t order, const size_t rs_corr_num_bytes) 
        : FEC(msg_size, inv_rate, order) {
        rs_corr_bytes = rs_corr_num_bytes;
        rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,
                                                  1, 1, rs_corr_bytes);
        rs_buf = (uint8_t *) malloc(getEncSize(msg_size));
    }

    size_t encode(const uint8_t *msg, const size_t msg_len, uint8_t *enc_msg) {
        MBED_ASSERT(getEncSize(msg_len) <= 256);
        size_t conv_len = correct_convolutional_encode(corr_con, msg, msg_len, rs_buf)/8;
        MBED_ASSERT(correct_reed_solomon_encode(rs_con, rs_buf, conv_len, enc_msg) != 256);
        return conv_len + rs_corr_bytes;
    }

    ssize_t decode(const uint8_t *enc_msg, const size_t enc_len, uint8_t *dec_msg) {
        size_t rs_len = correct_reed_solomon_decode(rs_con, enc_msg, enc_len, rs_buf);
        size_t conv_bytes = correct_convolutional_decode(corr_con, rs_buf, rs_len*8, dec_msg);
        MBED_ASSERT(conv_bytes != -1);
        return conv_bytes;
    }

    size_t getEncSize(const size_t msg_len) {
        size_t enc_size = getEncSizeConv(msg_len) + rs_corr_bytes;
        MBED_ASSERT(enc_size <= 256);
        return enc_size;
    }

    void benchmark(size_t num_iters);

    ~FECRSV() {
        correct_convolutional_destroy(corr_con);
        correct_reed_solomon_destroy(rs_con);
        free(rs_buf);
    }
};


class FECConv: public FEC {
public:    
    FECConv(const size_t msg_size, const size_t inv_rate, const size_t order) : FEC(msg_size, inv_rate, order) {}

    size_t encode(const uint8_t *msg, size_t msg_len, uint8_t *enc_msg) {
        return encodeConv(msg, msg_len, enc_msg);
    }

    ssize_t decode(const uint8_t *enc_msg, size_t enc_len, uint8_t *dec_msg) {
        return decodeConv(enc_msg, enc_len, dec_msg);
    }

    size_t getEncSize(const size_t msg_len) {
        return getEncSizeConv(msg_len);
    }

    void benchmark(size_t num_iters);

    ~FECConv() {
        correct_convolutional_destroy(corr_con);
    }
};

#endif /* FEC_HPP */