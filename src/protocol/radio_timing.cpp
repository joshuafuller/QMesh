#include "mbed.h"
#include "serial_data.hpp"
#include "radio_timing.hpp"


void RadioTiming::computeTimes(const uint32_t bw, const uint8_t sf, const uint8_t cr, 
        const uint32_t n_pre_sym, const uint8_t n_pld_bytes) {
    float bw_f = lora_bw[bw];
    float sf_f = sf;
    float cr_f = cr;
    float n_pre_sym_f = n_pre_sym;
    float n_pld_bytes_f = n_pld_bytes;
    n_sym_pre = n_pre_sym;

    // Compute duration of a symbol
    sym_time_s = powf(2, sf_f)/bw_f;
    sym_time_ms = sym_time_s*1e3f;
    sym_time_us = sym_time_s*1e6f;

    // Determine whether we need the low datarate optimize
    float low_dr_opt_f = sym_time_ms >= 16.f ? 1.f : 0.f;

    // Compute the duration of the preamble
    pre_time_s = sym_time_s*n_pre_sym_f;
    pre_time_ms = sym_time_s*n_pre_sym_f*1e3f;
    pre_time_us = sym_time_s*n_pre_sym_f*1e6f;

    // Compute number of payload symbols
    float val = ceilf((8.f*n_pld_bytes_f-4.f*sf_f+28.f-20.f)/
            (4.f*(sf_f-2.f*low_dr_opt_f)));
    float n_sym_pld_f = 8.f + fmaxf(val*(cr_f+4.f), 0.f);
    n_sym_pld = n_sym_pld_f;

    // Compute the duration of the payload
    pld_time_s = n_sym_pld_f*sym_time_s;
    pld_time_ms = n_sym_pld_f*sym_time_s*1e3f;
    pld_time_us = n_sym_pld_f*sym_time_s*1e6f;

    // Compute the duration of the entire LoRa packet
    float n_sym_pkt_f = n_pre_sym_f+n_sym_pld_f;
    n_sym_pkt = n_sym_pkt_f;
    pkt_time_s = n_sym_pkt_f*sym_time_s;
    pkt_time_ms = n_sym_pkt_f*sym_time_s*1e3f;
    pkt_time_us = n_sym_pkt_f*sym_time_s*1e6f;

    // Compute the symbol wait factors
    sym_frac_s = sym_time_s/SYM_FRAC_DELAY_SLOTS;
    sym_frac_ms = sym_frac_s*1e3f;
    sym_frac_us = sym_frac_s*1e6f;
}


// Just set the wait duration.
void RadioTiming::waitFullSlots(const size_t num_slots) {
    wait_duration_us = pkt_time_us + PADDING_TIME_US;
    wait_duration_us *= num_slots;
}


// Actually do the waiting
void RadioTiming::wait(void) {
    int elapsed_us = tmr_sptr->read_us();
    if(wait_duration_us-elapsed_us < 0) {
        debug_printf(DBG_WARN, "Wait duration is negative!\r\n");
        return;
    }
    wait_us(wait_duration_us-elapsed_us);
}

void RadioTiming::waitNoWarn(void) {
    int elapsed_us = tmr_sptr->read_us();
    if(wait_duration_us-elapsed_us < 0) {
        return;
    }
    wait_us(wait_duration_us-elapsed_us);
}


void RadioTiming::waitRxRemainder(const size_t sym_wait, const size_t pre_wait) {
    uint32_t wait_duration_us = (8-sym_wait)*sym_frac_us + (4-pre_wait)*pre_time_us;
    wait_duration_us += 16*sym_time_us; // 16 symbol padding for right now
    int elapsed_us = tmr_sptr->read_us();
    wait_us(wait_duration_us-elapsed_us);
}

void RadioTiming::calcWaitSymbol(void) {
    sym_wait_factor = rand() & 0x07; // 8 different symbol wait options
    sym_wait_us = sym_frac_us*sym_wait_factor;
}

void RadioTiming::calcWaitPreamble(void) {
    pre_wait_factor = rand() & 0x03; // 4 different preamble wait options
    pre_wait_us = pre_time_us*pre_wait_factor;
}

void RadioTiming::waitTx(void) {
    wait_us(sym_wait_us+pre_wait_us);
}

