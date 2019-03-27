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

//#include "lora_radio_helper.h"
#include "mbed.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <string>
#include "SX1272_LoRaRadio.h"

extern SX1272_LoRaRadio radio;

#if 0
union nv_radio_settings_union nv_radio_settings;
#endif

// Time on air, in ms
uint32_t time_on_air_ms = 0;
float time_on_air_s = 0.0;

// Radio state information
enum {
    IDLE,
    TX,
    RX,
} radio_state = IDLE;

// The callbacks used by the LoRa radio driver
static radio_events_t radio_events;

// Prototypes for the callbacks
static void tx_done_cb(void);
static void rx_done_cb(const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void tx_timeout_cb(void);
static void rx_timeout_cb(void);
static void rx_error_cb(void);
static void fhss_change_channel_cb(uint8_t current_channel);

// Included from lora_radio_helper.h is a radio object for our radio.
// Let's set it up.
void init_radio(void) {
    // Initialize Radio driver
    radio_events.tx_done = tx_done_cb;
    radio_events.rx_done = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    radio.init_radio(&radio_events);
    radio.set_rx_config(MODEM_LORA, RADIO_BANDWIDTH,
                            RADIO_SF, RADIO_CODERATE,
                            0, RADIO_PREAMBLE_LEN,
                            RADIO_SYM_TIMEOUT, RADIO_FIXED_LEN,
                            FRAME_PAYLOAD_LEN,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(MODEM_LORA, RADIO_POWER, 0,
                            RADIO_BANDWIDTH, RADIO_SF,
                            RADIO_CODERATE, RADIO_PREAMBLE_LEN,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
    radio.set_channel(RADIO_FREQUENCY);
    time_on_air_ms = radio.time_on_air(MODEM_LORA, FRAME_PAYLOAD_LEN);
    time_on_air_s = time_on_air_ms/1000.0;
}

// Simple function that just sends out a simple packet every 1s
static std::string send_str("KG5VBY: This is a test string\r\n");
static char send_string[256];
void test_radio(void) {
    while(true) {
        memcpy(send_string, send_str.c_str(), send_str.length());
        radio.send((uint8_t *) send_string, send_str.length());
        wait(1.0);
    }
}

static void tx_done_cb(void)
{
    // If we just finished retransmitting a frame
    //  Don't do anything, just let the meshing receive another frame
    // If we just finished transmitting a local frame
    //  check to see if another frame's sitting in the queue
    //  if so, grab another frame and set it up to be sent one time unit in the future
}

#warning "Need to properly implement extra_frame"
frame_t extra_frame;
static void process_rx_extra_frame(void) {}
static void process_rx_main_frame(void) {}
static void process_rx_aux_frame(void) {}
static void rx_done_cb(const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    // WE JUST RECEIVED A FRAME
    // Are we waiting for another received mesh frame to be retransmitted?
    //  If yes, then receive the frame, but process it using the "redundant frame" code path
    memcpy(&extra_frame.frame.data, payload, size);
    extra_frame.frame.rx_size = size;
    extra_frame.frame.rssi = rssi;
    extra_frame.frame.snr = snr;
    // What does process_extra_frame() do?
    //  Checks if this frame has been seen before -- if so:
    //      Checks for "new" information (like "doubling" message)
    //      If the first packet had errors, it uses this packet to help reconstruct the packet
    //  If this frame hasn't been seen before:
    //      Check to see if the hop_count is lower than the current hop_count -- if so, 
    //      we should "reset" ourselves to this transmitter instead of the current, 
    //      higher-hop-count transmitter
    process_rx_extra_frame();
    //***********************************
    // If we aren't waiting to retransmit a frame, then we should just prepare to 
    //  retransmit the received frame at the right time.
    process_rx_main_frame();
    //***********************************
    // If we are waiting to transmit another one of OUR frames, then we should 
    //  process this frame in order to understand if there's any sideband information
    //  we should be dealing with.
    process_rx_aux_frame();
}
 
static void tx_timeout_cb(void)
{
    printf("ERROR: Tx Timeout\r\n");
}
 
static void rx_timeout_cb(void)
{
    printf("ERROR: Rx Timeout\r\n");
}
 
static void rx_error_cb(void)
{
    printf("ERROR: Rx Error\r\n");
}

static void fhss_change_channel_cb(uint8_t current_channel) {

}