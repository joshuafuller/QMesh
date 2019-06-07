#include "mbed.h"
#include "params.hpp"
#include "correct.h"
#include "fec.hpp"
#include "serial_data.hpp"
#include <math.h>
#include <stdlib.h>
#include <string.h>


FEC::FEC(size_t msg_size, size_t inv_rate, size_t order) {
    // Set up the Reed-Solomon inner code
    block_length = 255;
    min_distance = 32;
    message_length = block_length - min_distance;
    rs_con = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds, 1, 1, min_distance);
    // Determine number of RS blocks per message
    double num_rs_blks = ceil((msg_size*8.0)/223.0);
    num_rs_bytes = (256*num_rs_blks)/8;
    rs_int_buf = (uint8_t *) malloc(num_rs_bytes);
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

uint8_t FEC::extractBit(uint8_t *buf, size_t bit_pos) {
    size_t byte_idx = bit_pos / 8;
    size_t bit_idx = bit_pos % 8;
    uint8_t val = buf[byte_idx];
    uint8_t ret_val = (val >> (7-bit_idx)) & 0x01;
    return ret_val;
}

void FEC::writeBit(uint8_t *buf, size_t bit_pos, uint8_t bit_val) {
    size_t byte_idx = bit_pos / 8;
    size_t bit_idx = bit_pos % 8;
    buf[byte_idx] |= (bit_val << (7-bit_idx));
}

size_t FEC::encodeRSV(uint8_t *msg, size_t msg_len, uint8_t *enc_msg) {
    uint8_t rs_enc_msg[MAX_FRAME_SIZE];
    memset(rs_enc_msg, 0xAB, sizeof(rs_enc_msg));
    // First, the Reed-Solomon
    for(int i = 0; i < msg_len*8 / 223; i+=223) {
        // extract 223 bits
        uint8_t buf_223[28];
        memset(buf_223, 0x00, sizeof(buf_223));
        for(int j = 0; j < 223; j++) {
            writeBit(buf_223, j, extractBit(msg, i*223+j));
        }
        // Perform the RS encode
        uint8_t buf_256[32];
        MBED_ASSERT(256 == correct_reed_solomon_encode(rs_con, buf_223, 223, buf_256));
        memcpy(rs_enc_msg+i*32, buf_256, 32);
    }
    size_t remaining_bits = msg_len*8 % 223;
    size_t full_rs_blocks = msg_len*8 / 223;
    // extract the remaining bits
    uint8_t buf_bits[28];
    memset(buf_bits, 0x00, sizeof(buf_bits));
    for(int j = 0; j < remaining_bits; j++) {
        writeBit(buf_bits, j, extractBit(msg, full_rs_blocks*223+j));
    }
    // Perform the RS encode
    uint8_t buf_bytes[32];
    memset(buf_bytes, 0x00, sizeof(buf_bytes));
    size_t bits_enc = correct_reed_solomon_encode(rs_con, buf_bits, remaining_bits, buf_bytes);
    memcpy(rs_enc_msg+full_rs_blocks*32, buf_bytes, 32);      
    size_t total_bits_enc = 256*full_rs_blocks + bits_enc;
    // Then, the convolutional coding
    return correct_convolutional_encode(corr_con, rs_enc_msg, total_bits_enc, enc_msg);
}
