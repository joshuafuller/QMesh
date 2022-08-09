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

#ifndef RADIO_TIMING_HPP
#define RADIO_TIMING_HPP

#include "mbed.h"
#include <memory>
#include <array>
#include <atomic>

extern DigitalOut *rx_int_mon, *tx_int_mon, *int_trig_mon; /// GPIO signals to monitor timing accuracy/jitter
extern DigitalOut *rssi_mon;

const array<float, 3> lora_bw = {125e3F, 250e3F, 500e3F}; //NOLINT /// Different LoRa bandwidths, in KHz

/**
 * A synchronized flooded mesh has all sort of timing-related things it needs to 
 * handle. This class handles all of these different timing things. Specifically, it
 * handles:
 * - Calculating LoRa timing values, such as symbol duration, preamble duration, 
 * payload duration, etc.
 * - Starting and stopping the fine-grained (millisecond or less) timer(s) necessary
 * to make sure transmissions are properly synchronized.
 * - Waiting the precise amount of time before e.g. retransmitting.
 */
class RadioTiming {
private:
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

    shared_ptr<Timer> tmr_sptr;
    int32_t wait_duration_us;


public:
    static uint32_t num_total_deadlines;
    static uint32_t num_missed_deadlines;
    
auto get_pkt_time_us() const -> uint32_t {
    return pkt_time_us;
}


static auto getNumTotalDeadlines() -> uint32_t;
static auto getNumMissedDeadlines() -> uint32_t;


/**
 * Given the various LoRa parameters, pre-compute all of the different timing
 * parameters.
 * @param bw LoRa bandwidth.
 * @param sf LoRa spreading factor.
 * @param cr LoRa coding rate.
 * @param n_pre_sym Number of preamble symbols.
 * @param n_pld_bytes Number of payload bytes.
 */
void computeTimes(uint32_t bw, uint8_t sf, uint8_t cr, uint32_t n_pre_sym, uint8_t n_pld_bytes);

/**
 * Wait for a number of transmit slots. Note that this function uses the timer's
 * start time as the basis for how long to wait. Since the timer is typically
 * started right after a LoRa packet is received, this function allows us to 
 * prepare a packet for retransmission, wait the remainder of one timeslot, and
 * then retransmit it.
 * @param num_slots Number of slots to wait.
 */
void waitFullSlots(size_t num_slots);


/**
 * Wait for a fraction of a symbol. 
 * @param symb_frac The fraction of the symbol, in eigths of half a symbol
 *  duration, to wait.
 * @papram direction should be either 1.0f or -1.0f, used to set the direction
 */
void waitSymOffset(uint8_t symb_frac, float direction, uint8_t num_inc);

/**
 * Calculate a random intra-symbol wait duration. Chooses between [0,7]
 * eights of the symbol duration. Stores the result internally within
 * the object.
 */
void calcWaitSymbol();

/** 
 * Starts the protocol timer.
 */
void setTimer(const shared_ptr<Timer> &my_tmr_sptr) {
    tmr_sptr = my_tmr_sptr;
}

/**
 * Returns the wait duration, in microseconds. This wait duration factors in 
 *  remaining time as well as any sort of intra-symbol timing offset.
 */
auto getWaitNoWarn() -> int32_t;

};

extern RadioTiming *radio_timing;


#endif /* RADIO_TIMING_HPP */