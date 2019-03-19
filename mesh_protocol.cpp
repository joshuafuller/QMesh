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
#include "serial_data.hpp"
#include "params.hpp"
#include <list>
#include <algorithm>

extern FrameQueue tx_queue, rx_queue;

static frame_t mesh_data;
static Timeout rx_mesh_time;
typedef enum {
    TX,
    RX,
} radio_state_t;
radio_state_t radio_state; 

#define TX_TIMEOUT 10
static int tx_timeout = 0;
void txCallback(void) {
    if(radio_state == TX) {
        // If there's something to send in the TX queue, then send it
        tx_timeout = 0;
        // Otherwise, wait until the next timeslot to try again
        tx_timeout += 1;
        if(tx_timeout > 10) {
            tx_timeout = 0;
            radio_state = RX;
        }
        else {
            
        }
    }
    else if(radio_state == RX) {
        // send(mesh_data);
    }
}


void rxCallback(frame_t *rx_frame) {
    rx_mesh_time.attach(txCallback, TIME_SLOT_SECONDS);
    rx_queue.enqueue(rx_frame);
    memcpy(&mesh_data, rx_frame, sizeof(frame_t));
}


static list<uint32_t> crc_list_vals;
bool checkRedundantPkt(const uint8_t *pld, const size_t len) {
    bool ret_val = false;
    MbedCRC<POLY_32BIT_ANSI, 32> ct;
    uint32_t crc = 0;
    ct.compute((void *) pld, len, &crc);
    if(find(crc_list_vals.begin(), crc_list_vals.end(), crc) == crc_list_vals.end()) {
        ret_val = true;
        crc_list_vals.push_back(crc);
        if(crc_list_vals.size() > PKT_CHK_HISTORY) {
            crc_list_vals.pop_front();
        }
    }
    return ret_val;
}