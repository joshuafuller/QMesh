/*
 * Copyright (c) 2019, Daniel R. Fay.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MESH_PROTOCOL_HPP
#define MESH_PROTOCOL_HPP

#include "mbed.h"
#include "radio.hpp"

typedef enum {
    BOOTING,
    MANAGEMENT,
    RUNNING
} system_state_t;

extern system_state_t current_mode;
extern bool stay_in_management;

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

/**
 * Given the various LoRa parameters, pre-compute all of the different timing
 * parameters.
 * @param bw LoRa bandwidth.
 * @param sf LoRa spreading factor.
 * @param cr LoRa coding rate.
 * @param n_pre_sym Number of preamble symbols.
 * @param n_pld_bytes Number of payload bytes.
 */
void computeTimes(const uint32_t bw, const uint8_t sf, const uint8_t cr, 
        const uint32_t n_pre_sym, uint8_t n_pld_bytes);

/**
 * Wait for a number of transmit slots. Note that this function uses the timer's
 * start time as the basis for how long to wait. Since the timer is typically
 * started right after a LoRa packet is received, this function allows us to 
 * prepare a packet for retransmission, wait the remainder of one timeslot, and
 * then retransmit it.
 * @param num_slots Number of slots to wait.
 */
void waitFullSlots(const size_t num_slots);

/**
 * Wait for the "remaining time" that includes various timing offsets
 * used to implement QMesh's collision resistance:
 * - Number of Preamble durations to wait
 * - Intra-symbol delay fraction
 * @param sym_wait How many eighths of a symbol to delay
 * @param pre_wait Number of preambles to wait
 */
void waitRxRemainder(const size_t sym_wait, const size_t pre_wait);

/**
 * Calculate a random intra-symbol wait duration. Chooses between [0,7]
 * eights of the symbol duration. Stores the result internally within
 * the object.
 */
void calcWaitSymbol(void);

/**
 * Calculate the random preamble-length wait period. Chooses between [0,3]
 * different preamble offsets.
 */
void calcWaitPreamble(void);

/**
 * Wait for the symbol and preamble offset durations.
 */
void waitTx(void);

/** 
 * Starts the protocol timer.
 */
void startTimer(void);
};

/**
 * This class handles "wobbling" the frequency. Wobbling the frequency 
 * involves randomly changing the center frequency to some value that
 * stays within the frequency error tolerance of LoRa (roughly +/- 20% 
 * of the modulated bandwidth). The goal of frequency wobbling is to 
 * reduce the amount of overlap between concurrent transmissions in 
 * order to increase the likelihood of successful capture of one of the
 * transmissions.
 */
#if 0
class RadioFrequency {
public:
#if 0
    /// Get a wobbled frequency.
    uint32_t getWobbledFreq(void);
#endif
};
#endif

extern RadioTiming radio_timing;

/**
 * Core function that implmements the QMesh mesh protocol.
 */
void mesh_protocol_fsm(void);

extern string beacon_msg;

/**
 * Core function that periodically sends out a beacon message.
 */
void beacon_fn(void);



#endif /* MESH_PROTOCOL_HPP */