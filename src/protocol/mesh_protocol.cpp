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

#include "mbed.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <math.h>
#include <list>
#include <map>
#include <algorithm>
#include "LoRaRadio.h"
#include "mesh_protocol.hpp"
#include "radio.hpp"

const float lora_bw[] = {125e3f, 250e3f, 500e3f};
static Frame mesh_frame;
static Timeout rx_mesh_time;
typedef enum {
    TX,
    RX,
} mesh_state_t;
mesh_state_t mesh_state; 

#define TX_TIMEOUT 10
static int tx_timeout = 0;
void txCallback(void) {
    if(mesh_state == TX) {
        // If there's something to send in the TX queue, then send it
        tx_timeout = 0;
        // Otherwise, wait until the next timeslot to try again
        tx_timeout += 1;
        if(tx_timeout > 10) {
            tx_timeout = 0;
            mesh_state = RX;
        }
        else {
            
        }
    }
    else if(mesh_state == RX) {
        // send(mesh_data);
    }
}


static list<uint32_t> past_crc;
static map<uint32_t, time_t> past_timestamp;
static bool checkRedundantPkt(shared_ptr<Frame> rx_frame);
static bool checkRedundantPkt(shared_ptr<Frame> rx_frame) {
    uint32_t crc = rx_frame->calculateUniqueCrc();
    bool ret_val = false;
    if(find(past_crc.begin(), past_crc.end(), crc) == past_crc.end()) {
        ret_val = true;
        past_crc.push_back(crc);
        past_timestamp.insert(pair<uint32_t, time_t>(crc, time(NULL)));
        if(past_crc.size() > PKT_CHK_HISTORY) {
            past_crc.pop_front();
            past_timestamp.erase(crc);
        }
    }
    else { // redundant packet was found. Check the age of the packet.
        map<uint32_t, time_t>::iterator it = past_timestamp.find(crc);
        if(time(NULL) - it->second > 60) { // Ignore entries more than a minute old
            ret_val = true;
            past_crc.erase(find(past_crc.begin(), past_crc.end(), crc));
            past_timestamp.erase(crc);
        }
    }
    return ret_val;
}

void RadioTiming::computeTimes(uint32_t bw, uint8_t sf, uint8_t cr, 
        uint32_t n_pre_sym, uint8_t n_pld_bytes) {
    float bw_f = bw;
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

void RadioTiming::waitFullSlots(size_t num_slots) {
    uint32_t wait_duration_us = pkt_time_us + (4-1)*pre_time_us + sym_time_us;
    wait_duration_us *= num_slots;
    int elapsed_us = tmr.read_us();
    wait_us(wait_duration_us-elapsed_us);
}

void RadioTiming::waitRxRemainder(size_t sym_wait, size_t pre_wait) {
    uint32_t wait_us = (8-sym_wait)*sym_frac_us + (4-pre_wait)*pre_time_us;
    wait_us += 16*sym_time_us; // 16 symbol padding for right now
    int elapsed_us = tmr.read_us();
    wait_us(wait_us-elapsed_us);
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

void RadioTiming::startTimer(void) {
    tmr.stop();
    tmr.reset();
    tmr.start();
}

uint32_t RadioFrequency::getWobbledFreq(void) {
    float wobble_factor = (float) rand() / RAND_MAX;
    float wobble_amount = wobble_factor * lora_bw[radio_cb["BW"].get<int>()] * FREQ_WOBBLE_PROPORTION;
    int wobble_direction = rand() & 0x1;
    wobble_amount = wobble_direction ? -wobble_amount : wobble_amount;
    uint32_t wobbled_freq = (int32_t) radio_cb["Freq"].get<int>() + (int32_t) wobble_amount;
    return wobbled_freq;   
}

RadioFrequency radio_frequency;

static enum {
    WAIT_FOR_RX,
    CHECK_TX_QUEUE,
    TX_PACKET,
    WAIT_TWO_SLOTS,
    PROCESS_PACKET,
    WAIT_ONE_SLOT,
    RETRANSMIT_PACKET,
} state = WAIT_FOR_RX;

RadioTiming radio_timing;

void mesh_protocol_fsm(void) {
    std::shared_ptr<RadioEvent> rx_radio_event, tx_radio_event;
    std::shared_ptr<Frame> tx_frame_sptr;
    std::shared_ptr<Frame> rx_frame_sptr;
    static uint8_t tx_frame_buf[256], rx_frame_buf[256];
    for(;;) {
        switch(state) {
            case WAIT_FOR_RX:
                rx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(rx_radio_evt_mail);
                if(rx_radio_event->evt_enum == RX_DONE_EVT) {
                    // Load up the frame
                    radio_timing.startTimer();
                    led2.LEDSolid();
                    rx_frame_sptr = make_shared<Frame>();
                    PKT_STATUS_ENUM pkt_status = rx_frame_sptr->deserialize(rx_radio_event->buf);
                    rx_frame_sptr->incrementTTL();
                    if(!nv_logger_mail.full()) {
                        enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, rx_frame_sptr);
                    }
                    else {
                        debug_printf(DBG_WARN, "NV Logging Queue full, dropping frame\r\n");
                    }
                    if(!rx_frame_mail.full()) {
                        enqueue_mail<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_sptr);
                    }
                    if(pkt_status == PKT_OK && checkRedundantPkt(rx_frame_sptr)) {
                        debug_printf(DBG_WARN, "Rx Queue full, dropping frame\r\n");
                        state = WAIT_FOR_RX;
                    }
                    else if(pkt_status != PKT_OK) {
                        debug_printf(DBG_INFO, "Rx packet not received correctly\r\n");
                    }
                    else {
                        radio.set_channel(radio_frequency.getWobbledFreq());
                        state = RETRANSMIT_PACKET;
                    }
                    radio_timing.waitFullSlots(1);
                }
                else if(rx_radio_event->evt_enum == RX_TIMEOUT_EVT) {
                    state = CHECK_TX_QUEUE;
                }
                else { MBED_ASSERT(false); }
            break;

            case CHECK_TX_QUEUE:
                if(!tx_frame_mail.empty()) {
                    tx_frame_sptr = dequeue_mail<std::shared_ptr<Frame>>(tx_frame_mail);
                    radio.set_channel(radio_frequency.getWobbledFreq());
                    state = TX_PACKET;
                }
                else {
                    state = WAIT_FOR_RX;
                }
            break;

            case TX_PACKET:
                { radio_timing.startTimer();
                led3.LEDSolid();
                size_t tx_frame_size = tx_frame_sptr->serialize(tx_frame_buf);
                MBED_ASSERT(tx_frame_size > 256);
                radio.send(tx_frame_buf, tx_frame_size);
                radio_timing.waitFullSlots(1);
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                led3.LEDOff();
                radio_timing.waitFullSlots(2);
                state = CHECK_TX_QUEUE;
                }
            break;

            case RETRANSMIT_PACKET:
                { radio_timing.startTimer();
                led3.LEDSolid();
                size_t rx_frame_size = rx_frame_sptr->serialize(tx_frame_buf);
                MBED_ASSERT(rx_frame_size > 256);
                radio.send(rx_frame_buf, rx_frame_size);
                radio_timing.waitFullSlots(1);
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                led2.LEDOff();
                led3.LEDOff();
                state = WAIT_FOR_RX;
                }
            break;

            default:
                MBED_ASSERT(false);
            break;
        }   
    }
}


string beacon_msg;
void beacon_fn(void) {
    auto beacon_frame_sptr = shared_ptr<Frame>();
    beacon_frame_sptr->setBeaconPayload(radio_cb["Beacon Msg"].get<string>());
    for(;;) {
        enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, beacon_frame_sptr);
        wait(radio_cb["Beacon Interval"].get<int>());
    }
}