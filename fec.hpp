#ifndef FEC_HPP
#define FEC_HPP

#include "mbed.h"
#include "correct.h"


// This class provides a way to magically apply forward error correction to 
//  a character array provided.
// For now, we'll just do a 1/2 encoding to make things simple
class FEC {
private:
    correct_convolutional *corr_con;
public:
    FEC() {
        corr_con = correct_convolutional_create(2, 7, correct_conv_r12_7_polynomial);
    }

    size_t encode(const uint8_t *msg, size_t msg_len, uint8_t *enc_msg) {
        return correct_convolutional_encode(corr_con, msg, msg_len, enc_msg);
    }

    ssize_t decode(const uint8_t *enc_msg, size_t enc_len, uint8_t *dec_msg) {
        return correct_convolutional_decode(corr_con, enc_msg, enc_len*8, dec_msg);
    }

    size_t getEncSize(const size_t msg_len) {
        return correct_convolutional_encode_len(corr_con, msg_len)/8;
    }

};

#endif /* FEC_HPP */