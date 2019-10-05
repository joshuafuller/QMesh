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

#include "mbed.h"
#include "params.hpp"
#include "correct.h"
#include "fec.hpp"
#include "serial_data.hpp"
#include <math.h>
#include <stdlib.h>
#include <string.h>


FEC::FEC(const size_t inv_rate, const size_t order) {
    my_msg_size = Frame::size();
    // Set up the convolutional outer code
    this->inv_rate = inv_rate;
    this->order = order;
    correct_convolutional_polynomial_t *poly = NULL;
    switch(inv_rate) {
        case 2: // 1/2
            switch(order) {
                case 6: poly = conv_r12_6_polynomial; break;
                case 7: poly = libfec_r12_7_polynomial; break;
                case 8: poly = conv_r12_8_polynomial; break;
                case 9: poly = conv_r12_9_polynomial; break;
                default:
                    debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
                    MBED_ASSERT(false);
                break;
            }
        break;
        case 3: // 1/3
            switch(order) {
                case 6: poly = conv_r13_6_polynomial; break;
                case 7: poly = conv_r13_7_polynomial; break;
                case 8: poly = conv_r13_8_polynomial; break;
                case 9: poly = libfec_r13_9_polynomial; break;
                default:
                    debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
                    MBED_ASSERT(false);
                break;
            }
        break;
        case 6: // 1/6
            if(order == 15) {
                poly = libfec_r16_15_polynomial;
            }
            else {
                debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
                MBED_ASSERT(false);                    
            }
        break;
        default:
            debug_printf(DBG_ERR, "Invalid convolutional coding parameters selected\r\n");
            MBED_ASSERT(false);
        break;
    }
    corr_con = correct_convolutional_create(inv_rate, order, poly);
}


void FECRSV::benchmark(size_t num_iters) {
    debug_printf(DBG_INFO, "====================\r\n");
    debug_printf(DBG_INFO, "Now benchmarking the RSV FEC. Running for %d iterations\r\n", num_iters);
    debug_printf(DBG_INFO, "Message size is %d, encoded size is %d\r\n", my_msg_size, getEncSize(my_msg_size));
    //uint8_t msg_data[my_msg_size];
    vector<uint8_t> msg_data(my_msg_size);
    vector<uint8_t> enc_data(my_msg_size);
    //uint8_t enc_data[my_msg_size];
    debug_printf(DBG_INFO, "Benchmarking the encode...\r\n");
    Timer enc_timer;
    enc_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        encode(msg_data, enc_data);
    }
    enc_timer.stop();
    int enc_num_ms = enc_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");
    debug_printf(DBG_INFO, "Benchmarking the decode...\r\n");
    Timer dec_timer;
    size_t enc_size = getEncSize(my_msg_size);
    dec_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        decode(enc_data, msg_data);
    }
    dec_timer.stop();
    int dec_num_ms = dec_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");        
    debug_printf(DBG_INFO, "Benchmarking complete! \r\n");
    debug_printf(DBG_INFO, "Encode: %d iterations complete in %d ms\r\n", num_iters, enc_num_ms);
    float enc_ms_per_iter = (float) enc_num_ms / (float) num_iters;
    float enc_iters_per_sec = (float) num_iters / ((float) enc_num_ms / 1000.f);
    debug_printf(DBG_INFO, "Encode: %f ms/iteration, %f iterations/s\r\n", enc_ms_per_iter, enc_iters_per_sec);
    debug_printf(DBG_INFO, "Decode: %d iterations complete in %d ms\r\n", num_iters, dec_num_ms);
    float dec_ms_per_iter = (float) dec_num_ms / (float) num_iters;
    float dec_iters_per_sec = (float) num_iters / ((float) dec_num_ms / 1000.f);
    debug_printf(DBG_INFO, "Decode: %f ms/iteration, %f iterations/s\r\n", dec_ms_per_iter, dec_iters_per_sec);        
    debug_printf(DBG_INFO, "====================\r\n");
}


void FECConv::benchmark(size_t num_iters) {
    debug_printf(DBG_INFO, "====================\r\n");
    debug_printf(DBG_INFO, "Now benchmarking the Conv FEC. Running for %d iterations\r\n", num_iters);
    //uint8_t msg_data[my_msg_size];
    vector<uint8_t> msg_data(my_msg_size);
    //uint8_t enc_data[my_msg_size];
    vector<uint8_t> enc_data(my_msg_size);
    debug_printf(DBG_INFO, "Benchmarking the encode...\r\n");
    Timer enc_timer;
    enc_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        encode(msg_data, enc_data);
    }
    enc_timer.stop();
    int enc_num_ms = enc_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");
    debug_printf(DBG_INFO, "Benchmarking the decode...\r\n");
    Timer dec_timer;;
    size_t enc_size = getEncSize(my_msg_size);
    dec_timer.start();
    for(size_t i = 0; i < num_iters; i++) {
        decode(enc_data, msg_data);
    }
    dec_timer.stop();
    int dec_num_ms = dec_timer.read_ms();
    debug_printf(DBG_INFO, "Done!\r\n");        
    debug_printf(DBG_INFO, "Benchmarking complete! \r\n");
    debug_printf(DBG_INFO, "Encode: %d iterations complete in %d ms\r\n", num_iters, enc_num_ms);
    float enc_ms_per_iter = (float) enc_num_ms / (float) num_iters;
    float enc_iters_per_sec = (float) num_iters / ((float) enc_num_ms / 1000.f);
    debug_printf(DBG_INFO, "Encode: %f ms/iteration, %f iterations/s\r\n", enc_ms_per_iter, enc_iters_per_sec);
    debug_printf(DBG_INFO, "Decode: %d iterations complete in %d ms\r\n", num_iters, dec_num_ms);
    float dec_ms_per_iter = (float) dec_num_ms / (float) num_iters;
    float dec_iters_per_sec = (float) num_iters / ((float) dec_num_ms / 1000.f);
    debug_printf(DBG_INFO, "Decode: %f ms/iteration, %f iterations/s\r\n", dec_ms_per_iter, dec_iters_per_sec);        
    debug_printf(DBG_INFO, "====================\r\n");
}
