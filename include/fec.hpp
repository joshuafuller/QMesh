/*
QMesh
Copyright (C) 2022 Daniel R. Fay

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

#ifndef TEST_FEC
#include "os_portability.hpp"
#endif /* TEST_FEC */
#include "correct.h"
#include "params.hpp"
#include <cmath>
#include <map>
#include <random>
#include <list>
#include <string>
#include <algorithm>
#include <typeinfo>
#include "golay.h"

#ifdef TEST_FEC
class Mutex {
public:
    void lock() {}
    void unlock() {}
};

static constexpr int PORTABLE_ASSERT_CONDITION = 10;
void PORTABLE_ASSERT(bool condition);
//#define PORTABLE_ASSERT
#endif /* TEST_FEC */

constexpr int DEFAULT_CONV_CONS_LEN = 7;
constexpr int DEFAULT_CONV_ORDER = 2;
constexpr int DEFAULT_RS_BYTES = 8;

// Convolutional Codes

#ifndef TEST_FEC
/**
 * Performs some testing of the different Forward Error Correction algorithms.
 */
void testFEC();
#endif /* TEST_FEC */

/**
 * Base class for Forward Error Correction. Provides some generically-useful functions,
 * like interleaving, but otherwise just functions as a dummy FEC class.
 */
class FEC {
private:
    std::string name;
    size_t msg_len;
    size_t enc_size;
    Mutex lock;

public:
    auto get_lock() -> Mutex & {
        return lock;
    }

    auto get_msg_len() const -> const size_t & {
        return msg_len;
    }

    void set_msg_len(const size_t my_msg_len) {
        msg_len = my_msg_len;
    }

    void set_name(const std::string &my_name) {
        name = my_name;
    }

    auto get_name() const -> const std::string & {
        return name;
    }

    void set_enc_size(const size_t my_enc_size) {
        enc_size = my_enc_size;
    }

    /// Constructor.
    explicit FEC(const int32_t my_msg_len) :
        name("Dummy FEC"),
        msg_len(my_msg_len),
        enc_size(my_msg_len) { }

    /// Constructor to facilitate unit testing.
    FEC(const int32_t my_msg_len, const int32_t  /*inv_rate*/, 
            const int32_t  /*order*/, const int32_t  /*my_rs_corr_bytes*/) : 
            FEC(my_msg_len) {}

    /// Gets the name of the FEC.
    auto getName() -> std::string {
        return name;
    }

    virtual auto encSize() -> size_t {
        return enc_size;
    }

    /**
     * Apply the FEC coding. Returns the encoded size, in bytes.
     * @param msg Byte vector of data to be encoded.
     * @param enc_msg Byte vector of encoded data.
     */
    virtual auto encode(const std::vector<uint8_t> &msg, std::vector<uint8_t> &enc_msg) -> int32_t {
        if(name == "Dummy FEC") {
            lock.lock();
        }
        PORTABLE_ASSERT(msg.size() == msg_len);
        enc_msg = msg;
        if(name == "Dummy FEC") {
            lock.unlock();
        }
        return msg.size();
    }

    /**
     * Decode FEC-coded data. Returns the decoded data size, in bytes.
     * @param enc_msg Byte vector of encoded data.
     * @param dec_msg Byte vector of decoded data.
     */
    virtual auto decode(const std::vector<uint8_t> &enc_msg, std::vector<uint8_t> &dec_msg) -> int32_t {
        if(name == "Dummy FEC") {
            lock.lock();
        }
        PORTABLE_ASSERT(enc_msg.size() == msg_len);
        dec_msg = enc_msg;
        if(name == "Dummy FEC") {
            lock.unlock();
        }
        return msg_len;
    }

    void benchmark(size_t num_iters);
};


/**
 * Derived class that just applies/removes the interleaving from the data.
 */ 
typedef struct { //NOLINT
        float bits_f{}, row_f{}, col_f{};
        uint32_t bits{}, bytes{}, row{}, col{};   
        uint32_t pre_bytes{};
    } int_params_struct;
class FECInterleave : public FEC {
    int_params_struct int_params;
    
private:
    static auto getBit(const std::vector<uint8_t> &bytes, int32_t pos) -> bool;

    static void setBit(bool bit, int32_t pos, std::vector<uint8_t> &bytes);


public:
    auto get_int_params() const -> const int_params_struct & {
        return int_params;
    }

    void set_int_params(const int_params_struct &my_int_params) {
        int_params = my_int_params;
    }

    auto get_int_params_mutable() -> int_params_struct & {
        return int_params;
    }

    explicit FECInterleave(int32_t my_msg_len);

    /// Constructor to facilitate unit testing.
    FECInterleave(const int32_t my_msg_len, const int32_t  /*inv_rate*/, 
            const int32_t  /*order*/, const int32_t  /*my_rs_corr_bytes*/) : 
            FECInterleave(my_msg_len) {}

    auto encode(const std::vector<uint8_t> &msg, std::vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const std::vector<uint8_t> &enc_msg, std::vector<uint8_t> &dec_msg) -> int32_t override;

    void interleaveBits(const std::vector<uint8_t> &bytes, std::vector<uint8_t> &bytes_int) const;
    
    void deinterleaveBits(const std::vector<uint8_t> &bytes_int, std::vector<uint8_t> &bytes_deint) const;
};


/**
 * Derived class that uses convolutional coding.
 */
typedef struct { //NOLINT
        int32_t bits{};
        int32_t bytes{};
    } conv_params_struct;
class FECConv: public FECInterleave {
private:
    // Convolutional coding parameters
    int32_t inv_rate{};
    int32_t order{};
    correct_convolutional *corr_con{};  
    conv_params_struct conv_params{};  

public: 
    auto get_conv_params() const -> const conv_params_struct & {
        return conv_params;
    }

    auto get_conv_params_mutable() -> conv_params_struct & {
        return conv_params;
    }

    auto get_inv_rate() const -> const int32_t & {
        return inv_rate;
    }

    auto get_order() const -> const int32_t & {
        return order;
    }

    auto get_corr_con() -> correct_convolutional * {
        return corr_con;
    }

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

    /// Constructor to facilitate unit testing.
    FECConv(const int32_t my_msg_len, const int32_t inv_rate, 
            const int32_t order, const int32_t  /*my_rs_corr_bytes*/) : 
            FECConv(my_msg_len, inv_rate, order) {}

    FECConv(const FECConv &old) : FECConv(old.get_msg_len(), old.inv_rate, old.order) { };

    auto operator= (const FECConv &rhs) -> FECConv & {
        if(this == &rhs) {
            return *this;
        }
        correct_convolutional_destroy(corr_con);
        *this = FECConv(rhs.get_msg_len(), rhs.inv_rate, rhs.order);
        return *this;
    }

    auto operator= (FECConv &&rhs) noexcept -> FECConv & {
        conv_params = rhs.conv_params;
        corr_con = rhs.corr_con;
        inv_rate = rhs.inv_rate;
        order = rhs.order;
        set_enc_size(rhs.encSize());
        set_int_params(rhs.get_int_params());
        set_msg_len(rhs.get_msg_len());
        set_name(rhs.get_name());
        return *this;
    }
    
    FECConv(FECConv&& other) noexcept : FECInterleave(get_msg_len()) {
        *this = std::move(other);
    }

    /// Destructor.
    ~FECConv() {
        correct_convolutional_destroy(corr_con);
    }

    auto encode(const std::vector<uint8_t> &msg, std::vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const std::vector<uint8_t> &enc_msg, std::vector<uint8_t> &dec_msg) -> int32_t override;
};


/**
 * Derived class that implements Reed-Solomon-Viterbi (RSV)
 * forward error correction. The convolutional coding is the same as 
 * implemented in the FECConv class, and the Reed-Solomon outer code
 * is a (256,223) code.
 */
class FECRSV: public FECConv {
private:
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

    FECRSV(const FECRSV &old) : FECRSV(old.get_msg_len(), old.get_inv_rate(), 
                                        old.get_order(), old.rs_corr_bytes) { };

    auto operator= (const FECRSV &rhs) -> FECRSV & {
        if(this == &rhs) {
            return *this;
        }
        correct_convolutional_destroy(get_corr_con());
        correct_reed_solomon_destroy(rs_con);
        *this = FECRSV(rhs.get_msg_len(), rhs.get_inv_rate(), rhs.get_order(), rhs.rs_corr_bytes);
        return *this;
    }

    auto operator= (FECRSV &&rhs) noexcept -> FECRSV & {
        rs_con = rhs.rs_con;
        rs_enc_msg_size = rhs.rs_enc_msg_size;
        rs_corr_bytes = rhs.rs_corr_bytes;
        return *this;
    }
    
    FECRSV(FECRSV&& other) = default;

    /// Destructor.
    ~FECRSV() {
        correct_reed_solomon_destroy(rs_con);
    }

    auto encode(const std::vector<uint8_t> &msg, std::vector<uint8_t> &rsv_enc_msg) -> int32_t override;

    auto decode(const std::vector<uint8_t> &rsv_enc_msg, std::vector<uint8_t> &dec_msg) -> int32_t override;
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
        set_name("RSVGolay");
    };

    /**
     * Default constructor. Initializes with a convolutional coding rate of 2,
     * n=9, and 32 Reed-Solomon correction bytes.
     */
    explicit FECRSVGolay(const int32_t my_msg_len) : FECRSVGolay(my_msg_len, DEFAULT_CONV_ORDER, 
                                                DEFAULT_CONV_CONS_LEN, DEFAULT_RS_BYTES) { };

    auto encode(const std::vector<uint8_t> &msg, std::vector<uint8_t> &enc_msg) -> int32_t override;

    auto decode(const std::vector<uint8_t> &enc_msg, std::vector<uint8_t> &dec_msg) -> int32_t override;

    auto encSize() -> size_t override {
        return static_cast<int>(FECRSV::encSize()+3);
    }
};

#endif /* FEC_HPP */