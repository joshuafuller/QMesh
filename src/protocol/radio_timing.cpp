/*
QMesh
Copyright (C) 2020 Daniel R. Fay

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
#include "serial_data.hpp"
#include "radio_timing.hpp"

DigitalOut rx_int_mon(MBED_CONF_APP_RX_INT_MON, 0);
DigitalOut tx_int_mon(MBED_CONF_APP_TX_INT_MON, 0);
DigitalOut int_trig_mon(MBED_CONF_APP_INT_TRIG_MON, 0);
DigitalOut rssi_mon(MBED_CONF_APP_RSSI_MON, 0);

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
    wait_duration_us = pkt_time_us + PADDING_TIME_US + sym_time_us;
    wait_duration_us *= num_slots;
}


// Add in the per-symbol timing offset
void RadioTiming::waitSymOffset(const uint8_t symb_frac, const float direction, 
    const uint8_t num_inc) {
    if(num_inc != 0) {
        float sym_frac_us = 0.5f * ((sym_time_us*0.9f)/(float) num_inc + 1) \
                            * ((float) symb_frac) * direction;
        wait_duration_us += sym_frac_us;
    }
}

int32_t RadioTiming::getWaitNoWarn(void) {
    int elapsed_us = tmr_sptr->read_us();
    if(wait_duration_us-elapsed_us < 0) {
        return 0;
    }
    return wait_duration_us-elapsed_us;
}
