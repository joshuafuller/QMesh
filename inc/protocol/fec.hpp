/*
QMesh
Copyright (C) 2021 Daniel R. Fay

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
#include <typeinfo>
#include "golay.h"

constexpr int BITS_IN_BYTE = 8;
constexpr int DEFAULT_CONV_CONS_LEN = 7;
constexpr int DEFAULT_CONV_ORDER = 2;
constexpr int DEFAULT_RS_BYTES = 8;

// Convolutional Codes

/**
 * Performs some testing of the different Forward Error Correction algorithms.
 */
void testFEC();

/**
 * Base class for Forward Error Correction. Provides some generically-useful functions,
 * like interleaving, but otherwise just functions as a dummy FEC class.
 */
class FEC {
protected:
    string name;
    size_t msg_len;
    size_t enc_size;
    Mutex lock;

public:
    /// Constructor.
    explicit FEC(const int32_t my_msg_len) {
        name = "Dummy FEC";
        msg_len = my_msg_len;
        enc_size = my_msg_len;
    }

    /// Gets the name of the FEC.
    auto getName() -> string {
        return name;
    }

    virtual auto encSize() -> int {
        return enc_size;
    }

    /**
     * Apply the FEC coding. Returns the encoded size, in bytes.
     * @param msg Byte vector of data to be encoded.
     * @param enc_msg Byte vector of encoded data.
     */
    virtual auto encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t {
        if(name == "Dummy FEC") {
            lock.lock();
        }
        MBED_ASSERT(msg.size() == msg_len);
        enc_msg = msg;
        return msg.size();
        if(name == "Dummy FEC") {
            lock.unlock();
        }
    }

    /**
     * Decode FEC-coded data. Returns the decoded data size, in bytes.
     * @param enc_msg Byte vector of encoded data.
     * @param dec_msg Byte vector of decoded data.
     */
    virtual auto decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t {
        if(name == "Dummy FEC") {
            lock.lock();
        }
        MBED_ASSERT(enc_msg.size() == msg_len);
        dec_msg = enc_msg;
        return msg_len;
        if(name == "Dummy FEC") {
            lock.unlock();
        }
    }

    void benchmark(size_t num_iters);
};


/**
 * Derived class that just applies/removes the interleaving from the data.
 */ 
class FECInterleave : public FEC {
protected:
    struct {
        float bits_f, row_f, col_f;
        uint32_t bits, bytes, row, col;   
        uint32_t pre_bytes;
    } int_params;
    void interleaveBits(const vector<uint8_t> &bytes, vector<uint8_t> &bytes_int) const;
    void deinterleaveBits(const vector<uint8_t> &bytes_int, vector<uint8_t> &bytes_deint) const;

    static auto getBit(const vector<uint8_t> &bytes, const int32_t pos) -> bool {
        uint8_t byte = bytes[pos/BITS_IN_BYTE];
        size_t byte_pos = pos % BITS_IN_BYTE;
        return (((1U << byte_pos) & byte) != 0);
    }

    static void setBit(const bool bit, const int32_t pos, vector<uint8_t> &bytes) {
		size_t byte_pos = pos % BITS_IN_BYTE;
        bytes[pos/BITS_IN_BYTE] &= ~(1U << byte_pos);
        uint8_t my_bit = (bit == false) ? 0 : 1;
        bytes[pos/BITS_IN_BYTE] |= static_cast<uint8_t>(my_bit << static_cast<uint8_t>(byte_pos));
    }


public:
    explicit FECInterleave(int32_t my_msg_len);

    auto encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t override;
};


/**
 * Derived class that uses convolutional coding.
 */
class FECConv: public FECInterleave {
protected:
    // Convolutional coding parameters
    int32_t inv_rate{};
    int32_t order{};
    correct_convolutional *corr_con{};  
    struct {
        int32_t bits;
        int32_t bytes;
    } conv_params{};  

public:    
    /** 
     * Default constructor. Creates an FECConv object with 1/2 rate and n=7.
     */
    explicit FECConv(const int32_t my_msg_len) : FECConv(my_msg_len, DEFAULT_CONV_ORDER, DEFAULT_CONV_ORDER) { }

    /**
     * Constructor parameterizable with coding rate and order.
     * @param inv_rate Coding rate. 2 and 3 are currently the only rates implemented.
     * @param order Order of the coder. Values supported are 6, 7, 8, and 9.
     */
    FECConv(int32_t my_msg_len, int32_t inv_rate, int32_t order);

    FECConv(const FECConv &old) : FECConv(old.msg_len, old.inv_rate, old.order) { };

    auto operator= (const FECConv &rhs) -> FECConv & {
        if(this == &rhs) {
            return *this;
        }
        correct_convolutional_destroy(corr_con);
        *this = FECConv(rhs.msg_len, rhs.inv_rate, rhs.order);
        return *this;
    }

    auto operator= (FECConv &&rhs) noexcept -> FECConv & {
        conv_params = rhs.conv_params;
        corr_con = rhs.corr_con;
        inv_rate = rhs.inv_rate;
        order = rhs.order;
        enc_size = rhs.enc_size;
        int_params = rhs.int_params;
        msg_len = rhs.msg_len;
        name = rhs.name;
        return *this;
    }
    
    FECConv(FECConv&& other) noexcept : FECInterleave(msg_len){
        *this = std::move(other);
    }

    /// Destructor.
    ~FECConv() {
        correct_convolutional_destroy(corr_con);
    }

    auto encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t override;
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
    FECRSV(int32_t my_msg_len, int32_t inv_rate, int32_t order, int32_t my_rs_corr_bytes);

    /**
     * Default constructor. Initializes with a convolutional coding rate of 2,
     * n=9, and 32 Reed-Solomon correction bytes.
     */
    explicit FECRSV(const int32_t my_msg_len) : FECRSV(my_msg_len, DEFAULT_CONV_ORDER, DEFAULT_CONV_CONS_LEN, 
                                                    DEFAULT_RS_BYTES) { };

    /// Destructor.
    ~FECRSV() {
        correct_reed_solomon_destroy(rs_con);
    }

    auto encode(const vector<uint8_t> &msg, vector<uint8_t> &rsv_enc_msg) -> int32_t override;

    auto decode(const vector<uint8_t> &rsv_enc_msg, vector<uint8_t> &dec_msg) -> int32_t override;
};


class FECRSVGolay: public FECRSV {
public:
    /**
     * Constructor. 
     * @param inv_rate Convolutional coding rate.
     * @param order Convolutional coding order.
     * @param my_rs_corr_bytes Number of Reed-Solomon correction bytes.
     */
    FECRSVGolay(const int32_t my_msg_len, const int32_t inv_rate, const int32_t order, 
            const int32_t my_rs_corr_bytes) : 
            FECRSV(my_msg_len-2, inv_rate, order, my_rs_corr_bytes) { 
        name = "RSVGolay";
    };

    /**
     * Default constructor. Initializes with a convolutional coding rate of 2,
     * n=9, and 32 Reed-Solomon correction bytes.
     */
    FECRSVGolay(const int32_t my_msg_len) : FECRSVGolay(my_msg_len, DEFAULT_CONV_ORDER, 
                                                DEFAULT_CONV_CONS_LEN, DEFAULT_RS_BYTES) { };

    /// Destructor.
    ~FECRSVGolay() {
        correct_reed_solomon_destroy(rs_con);
    }

    auto encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t override;

    auto encSize() -> int override {
        return static_cast<int>(enc_size+3);
    }
};

#endif /* FEC_HPP */