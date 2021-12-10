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

#include <cmath>
#include "os_portability.hpp"
#include "serial_data.hpp"
#include "radio_timing.hpp"

DigitalOut *rx_int_mon;
DigitalOut *tx_int_mon;
DigitalOut *int_trig_mon;
DigitalOut *rssi_mon;

void create_radio_timing_data_objects() {
    rx_int_mon = new DigitalOut(MBED_CONF_APP_RX_INT_MON, 0);
    tx_int_mon = new DigitalOut(MBED_CONF_APP_TX_INT_MON, 0);
    int_trig_mon = new DigitalOut(MBED_CONF_APP_INT_TRIG_MON, 0);
    rssi_mon = new DigitalOut(MBED_CONF_APP_RSSI_MON, 0);
}

static constexpr float MS_IN_S = 1e3F;
static constexpr float US_IN_S = 1e6F;
void RadioTiming::computeTimes(const uint32_t bw, const uint8_t sf, const uint8_t cr, 
        const uint32_t n_pre_sym, const uint8_t n_pld_bytes) {
    float bw_f = NAN;
    bw_f = lora_bw.at(bw);
    float sf_f = sf;
    float cr_f = cr;
    float n_pre_sym_f = n_pre_sym;
    float n_pld_bytes_f = n_pld_bytes;
    n_sym_pre = n_pre_sym;

    // Compute duration of a symbol
    sym_time_s = powf(2, sf_f)/bw_f;
    sym_time_ms = sym_time_s*MS_IN_S;
    sym_time_us = sym_time_s*US_IN_S;

    // Determine whether we need the low datarate optimize
    constexpr float LDR_SYM_DURATION_MS = 16.F;
    float low_dr_opt_f = sym_time_ms >= LDR_SYM_DURATION_MS ? 1.F : 0.F;

    // Compute the duration of the preamble
    pre_time_s = sym_time_s*n_pre_sym_f;
    pre_time_ms = sym_time_s*n_pre_sym_f*MS_IN_S;
    pre_time_us = sym_time_s*n_pre_sym_f*US_IN_S;

    // Compute number of payload symbols
    constexpr float BITS_PER_BYTE = 8.F;
    float val = ceilf((BITS_PER_BYTE*n_pld_bytes_f-4.f*sf_f+28.f-20.f)/ //NOLINT
            (4.f*(sf_f-2.f*low_dr_opt_f))); //NOLINT
    // Check this calculation
    float n_sym_pld_f = NAN;
    constexpr int SF5 = 5;
    constexpr int SF6 = 6;
    constexpr float BASE_SYMS_SF5_SF6 = 6.25F + 8.F;
    constexpr float BASE_SYMS_OTHER_SF = 4.25F + 8.F;
    if(sf == SF5 || sf == SF6) { 
        n_sym_pld_f = BASE_SYMS_SF5_SF6 + fmaxf(val*(cr_f+4.f), 0.f); //NOLINT        
    } else { 
        n_sym_pld_f = BASE_SYMS_OTHER_SF + fmaxf(val*(cr_f+4.f), 0.f); //NOLINT
    }
    n_sym_pld = n_sym_pld_f;

    // Compute the duration of the payload
    pld_time_s = n_sym_pld_f*sym_time_s;
    pld_time_ms = n_sym_pld_f*sym_time_s*MS_IN_S;
    pld_time_us = n_sym_pld_f*sym_time_s*US_IN_S;

    // Compute the duration of the entire LoRa packet
    float n_sym_pkt_f = n_pre_sym_f+n_sym_pld_f;
    n_sym_pkt = n_sym_pkt_f;
    pkt_time_s = n_sym_pkt_f*sym_time_s;
    pkt_time_ms = n_sym_pkt_f*sym_time_s*MS_IN_S;
    pkt_time_us = n_sym_pkt_f*sym_time_s*US_IN_S;

    // Compute the symbol wait factors
    sym_frac_s = sym_time_s/SYM_FRAC_DELAY_SLOTS;
    sym_frac_ms = sym_frac_s*MS_IN_S;
    sym_frac_us = sym_frac_s*US_IN_S;
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
        float sym_frac_us = 0.5f * ((sym_time_us*0.9f)/static_cast<float>(num_inc) + 1)  //NOLINT
                            * (static_cast<float>(symb_frac)) * direction;
        wait_duration_us += sym_frac_us;
    }
}


auto RadioTiming::getNumTotalDeadlines() -> uint32_t {
    return num_total_deadlines;
}


auto RadioTiming::getNumMissedDeadlines() -> uint32_t {
    return num_missed_deadlines;
}


uint32_t RadioTiming::num_total_deadlines = 0;
uint32_t RadioTiming::num_missed_deadlines = 0;
auto RadioTiming::getWaitNoWarn() -> int32_t {
    PORTABLE_ASSERT(tmr_sptr);
    int elapsed_us = tmr_sptr->read_us();
    PORTABLE_ASSERT(wait_duration_us < INT32_MAX);
    PORTABLE_ASSERT(wait_duration_us-elapsed_us < INT32_MAX);
    num_total_deadlines += 1;
    if(wait_duration_us-elapsed_us < 0) {
        num_missed_deadlines += 1;
        return 0;
    }
    return wait_duration_us-elapsed_us;
}
