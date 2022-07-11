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

#ifndef TEST_FEC
#include "os_portability.hpp"
#endif /* TEST_FEC */
#include "params.hpp"
#include "correct.h"
#include "fec.hpp"
#ifndef TEST_FEC
#include "serial_data.hpp"
#endif /* TEST_FEC */
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <random>
#include "mem_trace.hpp"

using namespace std; //NOLINT

#ifdef TEST_FEC
void PORTABLE_ASSERT(bool condition) {
    if(!condition) {
        throw PORTABLE_ASSERT_CONDITION;
    }
}
#endif /* TEST_FEC */

#ifdef TEST_FEC
// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
enum DBG_TYPES {
    DBG_INFO,
    DBG_WARN,
    DBG_ERR,
};

auto debug_printf(enum DBG_TYPES, const char *fmt, ...) -> int;
auto debug_printf(enum DBG_TYPES, const char *fmt, ...) -> int {
    return 0;
}

auto debug_printf_clean(enum DBG_TYPES, const char *fmt, ...) -> int;
auto debug_printf_clean(enum DBG_TYPES, const char *fmt, ...) -> int {
    return 0;
}
#endif /* TEST_FEC */

// Polynomials
// These have been determined via find_conv_libfec_poly.c
// We could just make up new ones, but we use libfec's here so that
//   codes encoded by this library can be decoded by the original libfec
//   and vice-versa
constexpr uint16_t V27POLYA = 0155;
constexpr uint16_t V27POLYB = 0117;

constexpr uint16_t V29POLYA = 0657; //NOLINT
constexpr uint16_t V29POLYB = 0435; //NOLINT

constexpr uint16_t V39POLYA = 0755;
constexpr uint16_t V39POLYB = 0633;
constexpr uint16_t V39POLYC = 0447;

constexpr uint16_t V615POLYA = 042631;
constexpr uint16_t V615POLYB = 047245;
constexpr uint16_t V615POLYC = 056507;
constexpr uint16_t V615POLYD = 073363;
constexpr uint16_t V615POLYE = 077267;
constexpr uint16_t V615POLYF = 064537;

static correct_convolutional_polynomial_t libfec_r12_7_polynomial[] = {V27POLYA, V27POLYB}; //NOLINT
static correct_convolutional_polynomial_t libfec_r13_9_polynomial[] = {V39POLYA, V39POLYB, V39POLYC}; //NOLINT
static correct_convolutional_polynomial_t libfec_r16_15_polynomial[] = {V615POLYA, V615POLYB, V615POLYC, //NOLINT
                        V615POLYD, V615POLYE, V615POLYF};
static correct_convolutional_polynomial_t conv_r12_6_polynomial[] = {073, 061}; //NOLINT
static correct_convolutional_polynomial_t conv_r12_8_polynomial[] = {0225, 0373}; //NOLINT
static correct_convolutional_polynomial_t conv_r12_9_polynomial[] = {0767, 0545}; //NOLINT
static correct_convolutional_polynomial_t conv_r13_6_polynomial[] = {053, 075, 047}; //NOLINT
static correct_convolutional_polynomial_t conv_r13_7_polynomial[] = {0137, 0153, 0121}; //NOLINT
static correct_convolutional_polynomial_t conv_r13_8_polynomial[] = {0333, 0257, 0351}; //NOLINT  

constexpr int ORDER_6 = 6;
constexpr int ORDER_7 = 7;
constexpr int ORDER_8 = 8;
constexpr int ORDER_9 = 9;
constexpr int ORDER_15 = 15;

static constexpr float BITS_IN_BYTE_F = 8.F;

constexpr size_t NUM_TESTS = 10;
constexpr size_t NUM_BENCHMARK_RUNS = 100;

#ifndef TEST_FEC
void testFEC() {
    PORTABLE_ASSERT(radio_cb.valid);
    vector<shared_ptr<FEC>> test_fecs;
    shared_ptr<FEC> fec_sptr;
    fec_sptr = make_shared<FEC>(Frame::size());   
    test_fecs.push_back(fec_sptr);
    fec_sptr = make_shared<FECInterleave>(Frame::size());
    test_fecs.push_back(fec_sptr);
    fec_sptr = make_shared<FECConv>(Frame::size());
    test_fecs.push_back(fec_sptr);
    fec_sptr = make_shared<FECRSV>(Frame::size());
    test_fecs.push_back(fec_sptr);
    debug_printf(DBG_INFO, "Initialized FEC objects\r\n");
    portability::sleep(ONE_SECOND);

    for(auto & test_fec : test_fecs) {
        debug_printf(DBG_INFO, "====================\r\n");
        debug_printf(DBG_INFO, "Now testing %s for correctness...\r\n", test_fec->getName().c_str());
        portability::sleep(ONE_SECOND);
        // Make a random string
        int fec_success = 0;
        int fec_fail = 0;
        int fec_total = 0;
        for(size_t i = 0; i < NUM_BENCHMARK_RUNS; i++) {
            vector<uint8_t> rand_data(radio_cb.net_cfg.pld_len);
            std::generate_n(rand_data.begin(), radio_cb.net_cfg.pld_len, rand);
            auto test_frame = make_shared<Frame>(test_fec);
            test_frame->loadTestFrame(rand_data);
            auto serialized_data = make_shared<vector<uint8_t>>();
            test_frame->serializeCoded(*serialized_data);
            auto test_output_frame = make_shared<Frame>(test_fec);           
			test_output_frame->deserializeCoded(serialized_data);     
            if(*test_frame != *test_output_frame) {
                fec_fail += 1;
            }
            else {
                fec_success += 1;
            }
            fec_total += 1;
        }
        debug_printf(DBG_INFO, "Finished testing. FEC succeeded %d times, failed %d times\r\n", 
            fec_success, fec_fail);  
        debug_printf(DBG_INFO, "====================\r\n");
        debug_printf(DBG_INFO, "Now testing %s for bit error resilience...\r\n", test_fec->getName().c_str());
        portability::sleep(ONE_SECOND);
        Frame size_frame(test_fec);
        debug_printf(DBG_INFO, "Size of frame is %d\r\n", Frame::size());
        fec_success = 0;
        fec_fail = 0;
        fec_total = 0;
        auto rand_gen = make_shared<mt19937>(radio_cb.address);
        for(size_t i = 0; i < Frame::size(); i++) {
            for(size_t j = 0; j < NUM_TESTS; j++) {
                vector<uint8_t> rand_data(radio_cb.net_cfg.pld_len);
                std::generate_n(rand_data.begin(), radio_cb.net_cfg.pld_len, rand);
                auto test_frame = make_shared<Frame>(test_fec);
                test_frame->loadTestFrame(rand_data);
                vector<uint8_t> serialized_data;
                test_frame->serializeCoded(serialized_data);
                serialized_data[i] = (*rand_gen)();
                auto test_output_frame = make_shared<Frame>(test_fec);
                auto deserialized_data = make_shared<vector<uint8_t>>(serialized_data);
                test_output_frame->deserializeCoded(deserialized_data);
                if(*test_frame != *test_output_frame) {
                    fec_fail += 1;
                }
                else {
                    fec_success += 1;
                }
                fec_total += 1;
            }       
        }   
        debug_printf(DBG_INFO, "Finished testing. FEC succeeded %d times, failed %d times, tested %d times\r\n", 
            fec_success, fec_fail, fec_total);     
        debug_printf(DBG_INFO, "====================\r\n");
        debug_printf(DBG_INFO, "Now benchmarking performance...\r\n");
        test_fec->benchmark(NUM_BENCHMARK_RUNS);
    }
}
#endif /* TEST_FEC */


auto FECInterleave::encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t {
    if(get_name() == "Dummy Interleaver") {
        get_lock().lock();
    }
    PORTABLE_ASSERT(msg.size() == get_msg_len());
    enc_msg.resize(int_params.bytes);
    copy(msg.begin(), msg.end(), enc_msg.begin());
    interleaveBits(msg, enc_msg);
    PORTABLE_ASSERT(enc_msg.size() == encSize());
    if(get_name() == "Dummy Interleaver") {
        get_lock().unlock();
    }

    return enc_msg.size();
}


auto FECInterleave::decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t {
    if(get_name() == "Dummy Interleaver") {
        get_lock().lock();
    }
    dec_msg.resize(get_msg_len());
    PORTABLE_ASSERT(enc_msg.size() == encSize());
    deinterleaveBits(enc_msg, dec_msg);
    PORTABLE_ASSERT(dec_msg.size() == get_msg_len());
    if(get_name() == "Dummy Interleaver") {
        get_lock().unlock();
    }

    return dec_msg.size();
}  


// Using a technique similar to the AO-40 OSCAR interleaving setup
void FECInterleave::interleaveBits(const vector<uint8_t> &bytes, vector<uint8_t> &bytes_int) const {
    PORTABLE_ASSERT(bytes.size() == int_params.pre_bytes);
    vector<uint8_t> new_bytes_preint(int_params.bytes, 0x00);
    vector<uint8_t> new_bytes_int(int_params.bytes, 0x00);
    copy(bytes.begin(), bytes.end(), new_bytes_preint.begin());
    uint32_t bit_idx = 0;
    for(uint32_t col = 0; col < int_params.row; col++) {
        for(uint32_t row = 0; row < int_params.col; row++) {
            bool bit = getBit(new_bytes_preint, col + row*int_params.row);
            setBit(bit, bit_idx++, new_bytes_int);
        }
    }
    bytes_int.resize(int_params.bytes);
    PORTABLE_ASSERT(bytes_int.size() == int_params.bytes);
    copy(new_bytes_int.begin(), new_bytes_int.end(), bytes_int.begin());
    PORTABLE_ASSERT(bytes_int.size() == int_params.bytes);
}


void FECInterleave::deinterleaveBits(const vector<uint8_t> &bytes_int, vector<uint8_t> &bytes_deint) const {
    PORTABLE_ASSERT(bytes_int.size() == int_params.bytes);
    vector<uint8_t> new_bytes_deint(bytes_int.size(), 0x00);
    uint32_t bit_idx = 0;
    for(uint32_t col = 0; col < int_params.row; col++) {
        for(uint32_t row = 0; row < int_params.col; row++) {
            bool bit = getBit(bytes_int, bit_idx++);
            setBit(bit, col + row*int_params.row, new_bytes_deint);
        }
    }
    PORTABLE_ASSERT(new_bytes_deint.size() >= int_params.pre_bytes);
    PORTABLE_ASSERT(bytes_deint.size() == int_params.pre_bytes);
    copy(new_bytes_deint.begin(), new_bytes_deint.begin()+int_params.pre_bytes, bytes_deint.begin());
}


FECConv::FECConv(const int32_t my_msg_len, const int32_t inv_rate, const int32_t order):
    FECInterleave(my_msg_len) {
    set_name("Convolutional Coding");
    // Set up the convolutional outer code
    this->inv_rate = inv_rate;
    this->order = order;
    correct_convolutional_polynomial_t *poly = nullptr;
    constexpr int32_t INV_RATE_2 = 2;
    constexpr int32_t INV_RATE_3 = 3;
    constexpr int32_t INV_RATE_6 = 6;
    switch(inv_rate) {
        case INV_RATE_2: // 1/2
            switch(order) {
                case ORDER_6: poly = conv_r12_6_polynomial; break;
                case ORDER_7: poly = libfec_r12_7_polynomial; break;
                case ORDER_8: poly = conv_r12_8_polynomial; break;
                case ORDER_9: poly = conv_r12_9_polynomial; break;
                default:
                    debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
                break;
            }
        break;
        case INV_RATE_3: // 1/3
            switch(order) {
                case ORDER_6: poly = conv_r13_6_polynomial; break;
                case ORDER_7: poly = conv_r13_7_polynomial; break;
                case ORDER_8: poly = conv_r13_8_polynomial; break;
                case ORDER_9: poly = libfec_r13_9_polynomial; break;
                default:
                    debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
                break;
            }
        break;
        case INV_RATE_6: // 1/6
            if(order == ORDER_15) {
                poly = libfec_r16_15_polynomial;
            }
            else {
                debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");                   
            }
        break;
        default:
            debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
        break;
    }
    corr_con = correct_convolutional_create(inv_rate, order, poly);

    // Set up all of the other major parameters
    set_msg_len(my_msg_len);

    // Get the post-convolutional encoding length
    conv_params.bits = correct_convolutional_encode_len(corr_con, get_msg_len());
    conv_params.bytes = ceilf(static_cast<float>(conv_params.bits)/static_cast<float>(BITS_IN_BYTE));
    get_int_params_mutable().pre_bytes = conv_params.bytes;

    // Set up the interleaving parameters
    get_int_params_mutable().bits_f = static_cast<float>(conv_params.bytes)*BITS_IN_BYTE;
    get_int_params_mutable().row_f = floorf(sqrtf(get_int_params().bits_f));
    get_int_params_mutable().col_f = ceilf(get_int_params().bits_f/get_int_params().row_f);
    debug_printf(DBG_INFO, "Size of row is %f col is %f\r\n", get_int_params().row_f, 
                    get_int_params().col_f);
    get_int_params_mutable().bits = static_cast<int32_t>(get_int_params().bits_f);
    get_int_params_mutable().bytes = 
        static_cast<size_t>(ceilf((get_int_params().row_f*get_int_params().col_f)/BITS_IN_BYTE_F));
    get_int_params_mutable().row = static_cast<int32_t>(get_int_params().row_f);
    get_int_params_mutable().col = static_cast<int32_t>(get_int_params().col_f);

    set_enc_size(get_int_params().bytes);
}


constexpr float MS_IN_SEC = 1000.F;
void FEC::benchmark(size_t num_iters) {
#ifndef TEST_FEC
    PORTABLE_ASSERT(radio_cb.valid);
    debug_printf(DBG_INFO, "====================\r\n");
    debug_printf(DBG_INFO, "Now benchmarking the %s. Running for %d iterations\r\n", name.c_str(), num_iters);
    debug_printf(DBG_INFO, "Current frame size is %d\r\n", Frame::size());
    vector<uint8_t> msg_data(Frame::size(), 0);
    vector<uint8_t> enc_data(enc_size, 0);
    vector<uint8_t> dec_data(Frame::size(), 0);
    debug_printf(DBG_INFO, "Testing the correctness...\r\n");
    int correct_test = 0;
    int incorrect_test = 0;
    auto rand_gen = make_shared<mt19937>(radio_cb.address);
    for(size_t j = 0; j < NUM_TESTS; j++) {
        for(size_t i = 0; i < Frame::size(); i++) {
            msg_data[i] = (*rand_gen)();    
        }
        encode(msg_data, enc_data);
        decode(enc_data, dec_data);
        if(msg_data == dec_data) {
            correct_test += 1;
        }
        else {
            incorrect_test += 1;
        }
    }
    debug_printf(DBG_INFO, "%d PASSED; %d FAILED\r\n", correct_test, incorrect_test);

    debug_printf(DBG_INFO, "Benchmarking the encode...\r\n");
    CalTimer enc_timer;
    enc_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        encode(msg_data, enc_data);
    }
    enc_timer.stop();
    int enc_num_ms = enc_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");
    debug_printf(DBG_INFO, "Benchmarking the decode...\r\n");
    CalTimer dec_timer;
    dec_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        decode(enc_data, msg_data);
    }
    dec_timer.stop();
    int dec_num_ms = dec_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");        
    debug_printf(DBG_INFO, "Benchmarking complete! \r\n");
    debug_printf(DBG_INFO, "Encode: %d iterations complete in %d ms\r\n", num_iters, enc_num_ms);
    float enc_ms_per_iter = static_cast<float>(enc_num_ms) / static_cast<float>(num_iters);
    float enc_iters_per_sec = static_cast<float>(num_iters) / (static_cast<float>(enc_num_ms) / MS_IN_SEC);
    debug_printf(DBG_INFO, "Encode: %f ms/iteration, %f iterations/s\r\n", enc_ms_per_iter, enc_iters_per_sec);
    debug_printf(DBG_INFO, "Decode: %d iterations complete in %d ms\r\n", num_iters, dec_num_ms);
    float dec_ms_per_iter = static_cast<float>(dec_num_ms) / static_cast<float>(num_iters);
    float dec_iters_per_sec = static_cast<float>(num_iters) / (static_cast<float>(dec_num_ms) / MS_IN_SEC);
    debug_printf(DBG_INFO, "Decode: %f ms/iteration, %f iterations/s\r\n", dec_ms_per_iter, dec_iters_per_sec);        
    debug_printf(DBG_INFO, "====================\r\n");
#endif /* TEST_FEC */
}


FECRSV::FECRSV(const int32_t my_msg_len, const int32_t inv_rate, const int32_t order, 
            const int32_t my_rs_corr_bytes) 
    : FECConv(my_msg_len, inv_rate, order) {
    set_name("RSV");

    // Set up all of the other major parameters
    set_msg_len(my_msg_len);

    // Reed-Solomon parameters
    rs_corr_bytes = my_rs_corr_bytes;
    rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,
                                                1, 1, rs_corr_bytes);
    rs_enc_msg_size = get_msg_len() + rs_corr_bytes;

    // Get the post-convolutional encoding length
    get_conv_params_mutable().bits = correct_convolutional_encode_len(get_corr_con(), rs_enc_msg_size);
    get_conv_params_mutable().bytes = ceilf(static_cast<float>(get_conv_params().bits)/BITS_IN_BYTE_F);
    get_int_params_mutable().pre_bytes = get_conv_params().bytes;

    // Set up the various other parameters
    get_int_params_mutable().bits_f = static_cast<float>(get_conv_params().bytes*BITS_IN_BYTE);
    get_int_params_mutable().row_f = floorf(sqrtf(get_int_params().bits_f));
    get_int_params_mutable().col_f = ceilf(get_int_params().bits_f/get_int_params().row_f);
    get_int_params_mutable().bits = static_cast<size_t>(get_int_params().bits_f);
    get_int_params_mutable().bytes = 
        static_cast<size_t>(ceilf((get_int_params().row_f*get_int_params().col_f)/BITS_IN_BYTE_F));
    get_int_params_mutable().row = static_cast<size_t>(get_int_params().row_f);
    get_int_params_mutable().col = static_cast<size_t>(get_int_params().col_f);
    set_enc_size(get_int_params().bytes);
}


auto FECConv::encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t {
    PORTABLE_ASSERT(msg.size() <= 256);
    if(get_name() == "Convolutional Coding") {
        get_lock().lock();
    }
    // Convolutional encode
    PORTABLE_ASSERT(msg.size() == get_msg_len());
    vector<uint8_t> conv_msg(conv_params.bytes, 0);
    auto conv_len = static_cast<int32_t>(ceilf(static_cast<float>(correct_convolutional_encode(corr_con, msg.data(), 
            msg.size(), conv_msg.data()))/BITS_IN_BYTE_F));
    PORTABLE_ASSERT(conv_len == conv_params.bytes);
    PORTABLE_ASSERT((int32_t) conv_msg.size() == conv_params.bytes);
    // Interleave
    vector<uint8_t> int_msg(get_int_params().bytes, 0);
    copy(conv_msg.begin(), conv_msg.end(), int_msg.begin());
    interleaveBits(conv_msg, int_msg);
    PORTABLE_ASSERT(int_msg.size() == encSize());

    enc_msg.resize(get_int_params().bytes);
    copy(int_msg.begin(), int_msg.end(), enc_msg.begin());
    PORTABLE_ASSERT(enc_msg.size() == encSize());
    if(get_name() == "Convolutional Coding") {
        get_lock().unlock();
    }

    return enc_msg.size();
}


auto FECRSV::encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t {
    PORTABLE_ASSERT(msg.size() <= 256);
    if(get_name() == "RSV") {
        get_lock().lock();
    }
    // Reed-Solomon encode
    PORTABLE_ASSERT(msg.size() == get_msg_len());
    vector<uint8_t> rs_enc_msg(rs_enc_msg_size, 0);
    correct_reed_solomon_encode(rs_con, msg.data(), get_msg_len(), rs_enc_msg.data());
    // Convolutional encode
    vector<uint8_t> conv_enc_msg(get_conv_params().bytes, 0);
    int32_t conv_enc_len = correct_convolutional_encode(get_corr_con(), rs_enc_msg.data(), 
                            rs_enc_msg_size, conv_enc_msg.data());
    PORTABLE_ASSERT(conv_enc_len == get_conv_params().bits);
    PORTABLE_ASSERT(static_cast<int32_t>(conv_enc_msg.size()) == get_conv_params().bytes);
    // Interleave
    vector<uint8_t> int_enc_msg(encSize(), 0);
    interleaveBits(conv_enc_msg, int_enc_msg);
	PORTABLE_ASSERT(int_enc_msg.size() == FECRSV::encSize());
    enc_msg.resize(encSize());
    copy(int_enc_msg.begin(), int_enc_msg.end(), enc_msg.begin());
    if(get_name() == "RSV") {
        get_lock().unlock();
    }

    return enc_msg.size();
}


auto FECConv::decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t {
    PORTABLE_ASSERT(enc_msg.size() <= 256);
    if(get_name() == "Convolutional Coding") {
        get_lock().lock();
    }
    // Deinterleave
	PORTABLE_ASSERT(enc_msg.size() == encSize());
	vector<uint8_t> deint_msg(conv_params.bytes, 0);
    deinterleaveBits(enc_msg, deint_msg);
    PORTABLE_ASSERT(static_cast<int32_t>(deint_msg.size()) == conv_params.bytes);
    // Convolutional decode
    dec_msg.resize(get_msg_len());
    int32_t dec_size = correct_convolutional_decode(corr_con, deint_msg.data(), 
                            conv_params.bits, dec_msg.data());
    PORTABLE_ASSERT(dec_size == static_cast<int32_t>(get_msg_len()));
    if(get_name() == "Convolutional Coding") {
        get_lock().unlock();
    }

    return dec_size;
}


auto FECRSV::decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t {
    PORTABLE_ASSERT(enc_msg.size() <= 256);
    if(get_name() == "RSV") {
        get_lock().lock();
    }
    // Deinterleave
    PORTABLE_ASSERT(enc_msg.size() == encSize());
    vector<uint8_t> deint_msg(get_conv_params().bytes, 0);
    deinterleaveBits(enc_msg, deint_msg);
    PORTABLE_ASSERT(static_cast<int32_t>(deint_msg.size()) == get_conv_params().bytes);
    // Convolutional decode
    vector<uint8_t> rs_enc_msg(rs_enc_msg_size, 0);
    size_t deconv_bytes = correct_convolutional_decode(get_corr_con(), deint_msg.data(), 
                            get_conv_params().bits, rs_enc_msg.data());
    PORTABLE_ASSERT(static_cast<int32_t>(deconv_bytes) == rs_enc_msg_size);
    PORTABLE_ASSERT(static_cast<int32_t>(rs_enc_msg.size()) == rs_enc_msg_size);
    // Reed-Solomon decode
    dec_msg.resize(get_msg_len());  
    correct_reed_solomon_decode(rs_con, rs_enc_msg.data(), rs_enc_msg_size, dec_msg.data());

    PORTABLE_ASSERT(dec_msg.size() == get_msg_len());
    if(get_name() == "RSV") {
        get_lock().unlock();
    }

    return dec_msg.size();
}


FECInterleave::FECInterleave(const int32_t my_msg_len) : 
    FEC(my_msg_len) {
    PORTABLE_ASSERT(my_msg_len <= 256);
    set_name("Dummy Interleaver");
    int_params.bits_f = NAN;
    int_params.row_f = NAN;
    int_params.col_f = NAN;
    int_params.bits = 0;
    int_params.bytes = 0;
    int_params.row = 0;
    int_params.col = 0;
    int_params.pre_bytes = 0;
    set_msg_len(my_msg_len);
    int_params.pre_bytes = my_msg_len;
    int_params.bits_f = get_msg_len()*BITS_IN_BYTE_F;
    int_params.row_f = floorf(sqrtf(int_params.bits_f));
    int_params.col_f = ceilf(int_params.bits_f/int_params.row_f);
    int_params.bits = static_cast<size_t>(int_params.bits_f);
    int_params.bytes = static_cast<size_t>(ceilf((int_params.row_f*int_params.col_f)/BITS_IN_BYTE_F));
    int_params.row = static_cast<size_t>(int_params.row_f);
    int_params.col = static_cast<size_t>(int_params.col_f);
    //printf("Parameters are pre_bytes %d, bits %f %d, row %d, col %d, bytes %d\r\n", 
    //    int_params.pre_bytes, int_params.bits_f, int_params.bits, int_params.row, int_params.col, int_params.bytes);
    set_enc_size(int_params.bytes);
}


static constexpr uint32_t SHIFT_ONE_BYTE = 8U;
static constexpr uint32_t SHIFT_TWO_BYTES = 16U; 
static constexpr uint32_t BYTE_MASK = 0x000000FF;
auto FECRSVGolay::encode(const vector<uint8_t> &msg, vector<uint8_t> &enc_msg) -> int32_t {
    PORTABLE_ASSERT(msg.size() <= 256);
    if(get_name() == "RSVGolay") {
        get_lock().lock();
    }
    uint16_t golay_msg = 0x0000;
    golay_msg |= (msg[1] & BYTE_MASK) << SHIFT_ONE_BYTE;
    golay_msg |= (msg[0] & BYTE_MASK);
    uint32_t golay_enc_msg = 0;
    golay_enc_msg = golay_encode(golay_msg);
    vector<uint8_t> rsv_msg(msg.size()-2);
    copy(msg.begin()+2, msg.end(), rsv_msg.begin());
    vector<uint8_t> rsv_enc_msg;
    FECRSV::encode(rsv_msg, rsv_enc_msg);
    vector<uint8_t> golay_rsv_enc_msg(3+rsv_enc_msg.size());
    copy(rsv_enc_msg.begin(), rsv_enc_msg.end(), golay_rsv_enc_msg.begin()+3);
    golay_rsv_enc_msg[0] = golay_enc_msg & BYTE_MASK;
    golay_rsv_enc_msg[1] = (golay_enc_msg >> SHIFT_ONE_BYTE) & BYTE_MASK;
    golay_rsv_enc_msg[2] = (golay_enc_msg >> SHIFT_TWO_BYTES) & BYTE_MASK;
    enc_msg = golay_rsv_enc_msg;
    if(get_name() == "RSVGolay") {
        get_lock().unlock();
    }

    return enc_msg.size();
}


auto FECRSVGolay::decode(const vector<uint8_t> &enc_msg, vector<uint8_t> &dec_msg) -> int32_t {
    PORTABLE_ASSERT(enc_msg.size() <= 256);
    if(get_name() == "RSVGolay") {
        get_lock().lock();
    }
    uint32_t golay_enc_msg = 0;
    golay_enc_msg = 0x00000000;
    golay_enc_msg |= (enc_msg[0] & BYTE_MASK);
    golay_enc_msg |= (enc_msg[1] & BYTE_MASK) << SHIFT_ONE_BYTE;
    golay_enc_msg |= (enc_msg[2] & BYTE_MASK) << SHIFT_TWO_BYTES;
    uint16_t golay_msg = 0x0000;
    golay_msg = golay_decode(golay_enc_msg);
    vector<uint8_t> rsv_enc_msg(enc_msg.size()-3);
    copy(enc_msg.begin()+3, enc_msg.end(), rsv_enc_msg.begin());
    vector<uint8_t> rsv_dec_msg;
    FECRSV::decode(rsv_enc_msg, rsv_dec_msg);
    dec_msg.resize(2+rsv_dec_msg.size());
    copy(rsv_dec_msg.begin(), rsv_dec_msg.end(), dec_msg.begin()+2);
    dec_msg[0] = (golay_msg & BYTE_MASK);
    dec_msg[1] = (golay_msg & BYTE_MASK) << SHIFT_ONE_BYTE;
    if(get_name() == "RSVGolay") {
        get_lock().unlock();
    }

    return dec_msg.size();
}