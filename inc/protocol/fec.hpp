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
#include <string>
#include <random>
#include <list>
#include <algorithm>


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
static correct_convolutional_polynomial_t conv_r13_7_polynomial[] = {0137, 0153, 0121};
static correct_convolutional_polynomial_t conv_r13_8_polynomial[] = {0333, 0257, 0351};
static correct_convolutional_polynomial_t conv_r13_9_polynomial[] = {0417, 0627, 0675};                        

/**
 * Performs some testing of the different Forward Error Correction algorithms.
 */
void testFEC(void);

/**
 * Base class for Forward Error Correction. Provides some generically-useful functions,
 * like interleaving, but otherwise just functions as a dummy FEC class.
 */
class FEC {
protected:
    string name;
    int msg_len;

public:
    int enc_size;

    /// Constructor.
    FEC(const int32_t my_msg_len) {
        name = "Dummy FEC";
        msg_len = my_msg_len;
        enc_size = my_msg_len;
    }

    /// Gets the name of the FEC.
    string getName(void) {
        return name;
    }

    /**
     * Apply the FEC coding. Returns the encoded size, in bytes.
     * @param msg Byte vector of data to be encoded.
     * @param enc_msg Byte vector of encoded data.
     */
    virtual int32_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) {
        MBED_ASSERT(msg.size() == msg_len);
        enc_msg = msg;
        return msg.size();
    }

    /**
     * Decode FEC-coded data. Returns the decoded data size, in bytes.
     * @param enc_msg Byte vector of encoded data.
     * @param dec_msg Byte vector of decoded data.
     */
    virtual int32_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) {
        MBED_ASSERT(enc_msg.size() == msg_len);
        dec_msg = enc_msg;
        return msg_len;
    }

    void benchmark(const size_t num_iters);
};


/**
 * Derived class that just applies/removes the interleaving from the data.
 */
class FECInterleave : public FEC {
protected:
    struct {
        float bits_f, row_f, col_f;
        int32_t bits, bytes, row, col;   
        int32_t pre_bytes;
    } int_params;
    void interleaveBits(const vector<uint8_t> &bytes, vector<uint8_t> &bytes_int);
    void deinterleaveBits(const vector<uint8_t> &bytes_int, vector<uint8_t> &bytes_deint);

    static bool getBit(const vector<uint8_t> &bytes, const int32_t pos) {
        uint8_t byte = bytes[pos/8];
        size_t byte_pos = pos % 8;
        return ((((1 << byte_pos) & byte) == 0) ? false : true);
    }

    static void setBit(const bool bit, const int32_t pos, vector<uint8_t> &bytes) {
		size_t byte_pos = pos % 8;
        bytes[pos/8] &= ~(1 << byte_pos);
        uint8_t my_bit = (bit == false) ? 0 : 1;
        bytes[pos/8] |= (my_bit << byte_pos);
    }


public:
    FECInterleave(const int32_t my_msg_len);

    int32_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg);

    int32_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg);
};


/**
 * Derived class that uses convolutional coding.
 */
class FECConv: public FECInterleave {
protected:
    // Convolutional coding parameters
    int32_t inv_rate;
    int32_t order;
    correct_convolutional *corr_con;  
    struct {
        int32_t bits;
        int32_t bytes;
    } conv_params;  

public:    
    /** 
     * Default constructor. Creates an FECConv object with 1/2 rate and n=9.
     */
    FECConv(const int32_t my_msg_len) : FECConv(my_msg_len, 2, 9) { }

    /**
     * Constructor parameterizable with coding rate and order.
     * @param inv_rate Coding rate. 2 and 3 are currently the only rates implemented.
     * @param order Order of the coder. Values supported are 6, 7, 8, and 9.
     */
    FECConv(const int32_t my_msg_len, const int32_t inv_rate, const int32_t order);

    /// Destructor.
    ~FECConv(void) {
        correct_convolutional_destroy(corr_con);
    }

    int32_t encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg);

    int32_t decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg);
};


/**
 * Derived class that implements Reed-Solomon-Viterbi (RSV)
 * forward error correction. The convolutional coding is the same as 
 * implemented in the FECConv class, and the Reed-Solomon outer code
 * is a (256,223) code.
 */
class FECRSV: public FECConv {
protected:
    int32_t rs_corr_bytes;
    correct_reed_solomon *rs_con;
    int32_t rs_enc_msg_size;

public:
    /**
     * Constructor. 
     * @param inv_rate Convolutional coding rate.
     * @param order Convolutional coding order.
     * @param my_rs_corr_bytes Number of Reed-Solomon correction bytes.
     */
    FECRSV(const int32_t my_msg_len, const int32_t inv_rate, const int32_t order, 
            const int32_t my_rs_corr_bytes);

    /**
     * Default constructor. Initializes with a convolutional coding rate of 2,
     * n=9, and 32 Reed-Solomon correction bytes.
     */
    FECRSV(const int32_t my_msg_len) : FECRSV(my_msg_len, 2, 9, 8) { };

    /// Destructor.
    ~FECRSV(void) {
        correct_reed_solomon_destroy(rs_con);
    }

    int32_t encode(const vector<uint8_t> &msg, vector<uint8_t> &rsv_enc_msg);

    int32_t decode(const vector<uint8_t> &rsv_enc_msg, vector<uint8_t> &dec_msg);
};

#endif /* FEC_HPP */