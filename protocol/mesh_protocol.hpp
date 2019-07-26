#ifndef MESH_PROTOCOL_HPP
#define MESH_PROTOCOL_HPP

#include "mbed.h"
#include "serial_data.hpp"
#include "radio.hpp"

extern Mail<shared_ptr<Frame>, 16> tx_frame_mail, rx_frame_mail;

// Class that stores the different radio timing values.
class RadioTiming {
public:
    float sym_time_s;
    float pre_time_s;
    float pld_time_s;
    float pkt_time_s;

    uint32_t sym_time_ms;
    uint32_t pre_time_ms;
    uint32_t pld_time_ms;
    uint32_t pkt_time_ms;

    uint32_t sym_time_us;
    uint32_t pre_time_us;
    uint32_t pld_time_us;
    uint32_t pkt_time_us;
    
    uint32_t n_sym_pre;
    uint32_t n_sym_pld;
    uint32_t n_sym_pkt;

void computeTimes(uint32_t bw, uint8_t sf, uint8_t cr, 
        uint32_t n_pre_sym, uint8_t n_pld_bytes);

void waitSlots(size_t num_slots);

void startTimer(void);
};

extern RadioTiming radio_timing;

void mesh_protocol_fsm(void);



#endif /* MESH_PROTOCOL_HPP */