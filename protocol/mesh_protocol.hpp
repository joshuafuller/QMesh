#ifndef MESH_PROTOCOL_HPP
#define MESH_PROTOCOL_HPP

#include "mbed.h"
#include "radio.hpp"

// Class that stores the different radio timing values.
class RadioTiming {
public:
    float sym_time_s;
    float pre_time_s;
    float pld_time_s;
    float pkt_time_s;
    float sym_frac_s;

    uint32_t sym_time_ms;
    uint32_t pre_time_ms;
    uint32_t pld_time_ms;
    uint32_t pkt_time_ms;
    uint32_t sym_frac_ms;

    uint32_t sym_time_us;
    uint32_t pre_time_us;
    uint32_t pld_time_us;
    uint32_t pkt_time_us;
    uint32_t sym_frac_us;
    
    uint32_t n_sym_pre;
    uint32_t n_sym_pld;
    uint32_t n_sym_pkt;

    uint32_t sym_wait_factor;
    uint32_t sym_wait_us;
    uint32_t pre_wait_factor;
    uint32_t pre_wait_us;

    Timer tmr;

void computeTimes(uint32_t bw, uint8_t sf, uint8_t cr, 
        uint32_t n_pre_sym, uint8_t n_pld_bytes);

void waitFullSlots(size_t num_slots);

void waitRxRemainder(size_t sym_wait, size_t pre_wait);

void calcWaitSymbol(void);

void calcWaitPreamble(void);

void waitTx(void);

void startTimer(void);
};

class RadioFrequency {
public:
    uint32_t getWobbledFreq(void);
};

extern RadioTiming radio_timing;

void mesh_protocol_fsm(void);

void beacon_fn(void);



#endif /* MESH_PROTOCOL_HPP */