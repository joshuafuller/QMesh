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
#include "rtos.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <string>
#if (MBED_CONF_APP_LORA_RADIO == SX1272)
#include "SX1272_LoRaRadio.h"
#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
#include "SX1276_LoRaRadio.h"
#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
#include "SX126X_LoRaRadio.h"
#endif
#include "radio.hpp"
#include "correct.h"

#if (MBED_CONF_APP_LORA_RADIO == SX1272)
extern SX1272_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
extern SX1276_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
extern SX126X_LoRaRadio radio;
#endif

// Nonvolatile radio settings
extern NVSettings *nv_settings;

// Main thread for working with the LoRa radio
Thread radio_thread;

// Radio event flags for synchronizing threads
EventFlags rx_done_evt;

// Time on air for a packet
float time_on_air_us = 0.0;
float time_on_air_ms = 0.0;
float time_on_air_s = 0.0;

// Symbol period
float time_per_symbol_s = 0.0;
float time_per_symbol_ms = 0.0;
float time_per_symbol_us = 0.0;

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
static void rx_done_cb(uint8_t const *payload, uint16_t size, int16_t rssi, int8_t snr);
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
    printf("before unique variable\r\n");
    radio.primary_active = false;
    radio.secondary_active = false;
    printf("after unique variable\r\n");
    radio.init_radio(&radio_events);
    printf("afer init_radio\r\n");
    uint8_t radio_bw = nv_settings->getBW();
    uint8_t radio_sf = nv_settings->getSF();
    uint8_t radio_cr = nv_settings->getCR();
    uint8_t radio_freq = nv_settings->getFrequency();
    Frame tmp_frame;
    uint8_t full_pkt_len = tmp_frame.getFullPktSize();
    debug_printf(DBG_INFO, "Setting RX size to %d\r\n", full_pkt_len);
    radio.set_rx_config(MODEM_LORA, RADIO_BANDWIDTH,
                            RADIO_SF, RADIO_CODERATE,
                            0, RADIO_PREAMBLE_LEN,
                            RADIO_SYM_TIMEOUT, RADIO_FIXED_LEN,
                            full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, false);
    radio.set_tx_config(MODEM_LORA, RADIO_POWER, 0,
                            RADIO_BANDWIDTH, RADIO_SF,
                            RADIO_CODERATE, RADIO_PREAMBLE_LEN,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
    radio.set_channel(RADIO_FREQUENCY);

    // Total time on air for a single packet
    time_on_air_ms = radio.time_on_air(MODEM_LORA, FRAME_PAYLOAD_LEN);
    time_on_air_s = time_on_air_ms/1000.0;
    time_on_air_us = time_on_air_ms * 1000;

    // Symbol rate : time for one symbol (secs)
    float rs = RADIO_BANDWIDTH / (1 << RADIO_SF);
    float time_per_symbol_s = 1 / rs;
    float time_per_symbol_ms = 1000.0f * time_per_symbol_s;
    float time_per_symbol_us = 1000.0f * time_per_symbol_us;
}

// Simple function that just sends out a simple packet every 1s
static std::string send_str("KG5VBY: 01234567890123456789--\r\n");
static uint8_t pld_string[512];
static uint8_t send_string[512];
static Frame tx_test_frame;
void tx_test_radio(void) {
    while(true) {
#if 0        
        //memcpy(send_string, send_str.c_str(), send_str.length());
        // apply FEC
        //fec.encode((uint8_t *) send_str.c_str(), send_str.length(), send_string);
        //radio.send((uint8_t *) send_string, send_str.length()*2);
        debug_printf(DBG_INFO, "Length of send string is %d\r\n", send_str.length());
        radio.send((uint8_t *) send_string, send_str.length());
#else
        memcpy(pld_string, send_str.c_str(), send_str.length());
        tx_test_frame.loadTestFrame(pld_string);
        tx_test_frame.prettyPrint(DBG_INFO);
        size_t send_len = tx_test_frame.serialize(send_string);
        debug_printf(DBG_INFO, "Sending %d bytes\r\n", send_len);
        radio.send(send_string, send_len);
#endif
        debug_printf(DBG_INFO, "Transmitted Packet\r\n");
        wait(5.0);
    }
}

// Simple test function that just prints out received packets
void rx_test_radio(void) {
    while(true) {
        radio.set_channel( RADIO_FREQUENCY );
        radio.receive();
        rx_done_evt.wait_all(0x1);
        debug_printf(DBG_INFO, "Received Packet\r\n");
    }
}

// Originator node function. Sends out a frame once a second
//  in order to test the mesh network capability
void mesh_originator_test(void) {
    while(true) {
        // Build the frame
        memcpy(send_string, send_str.c_str(), send_str.length());
        // Send the frame
        radio.send((uint8_t *) send_string, send_str.length());
        debug_printf(DBG_INFO, "Transmitted Frame\r\n");
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
    debug_printf(DBG_INFO, "TX Done interrupt generated\r\n");
    radio.set_channel( RADIO_FREQUENCY );
}

#warning "Need to properly implement extra_frame"
Frame rx_frame;
Frame extra_frame;
static void process_rx_extra_frame(void) {}
static void process_rx_main_frame(void) {}
static void process_rx_aux_frame(void) {}
static void rx_done_cb(uint8_t const *payload, uint16_t size, int16_t rssi, int8_t snr)
{
#ifndef RX_TEST_MODE
#ifndef TX_TEST_MODE
    static bool first_time = true;
    // Seed the PRNG with the RSSI to try to add some more entropy
    if(first_time) {
        first_time = false;
        srand(rssi);
    }
    radio.set_channel( RADIO_FREQUENCY );
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
#endif
#else
    radio.set_channel( RADIO_FREQUENCY );
    //radio.sleep();
    static uint8_t rx_buf[1024];
    memcpy(rx_buf, payload, size);
    for(int i = 0; i < size; i++) {
        printf("%02x ", rx_buf[i]);
    }
    printf("\r\n");
    rx_frame.deserialize(rx_buf, size);
    rx_frame.setRxStats(rssi, snr, size);
    rx_frame.prettyPrint(DBG_INFO);
    rx_done_evt.set(0x1);
#endif /* RX_TEST_MODE */
}
 
static void tx_timeout_cb(void)
{
    radio.set_channel( RADIO_FREQUENCY );
    debug_printf(DBG_ERR, "Tx Timeout\r\n");
}
 
static void rx_timeout_cb(void)
{
    radio.set_channel( RADIO_FREQUENCY );
    radio.sleep();
    debug_printf(DBG_ERR, "Rx Timeout\r\n");
    rx_done_evt.set(0x1);
}
 
static void rx_error_cb(void)
{
    radio.set_channel( RADIO_FREQUENCY );
    radio.sleep();
    debug_printf(DBG_ERR, "Rx Error\r\n");
    rx_done_evt.set(0x1);
}

static void fhss_change_channel_cb(uint8_t current_channel) {
    int32_t new_channel = hopping_channels[current_channel % 
                (sizeof(hopping_channels)/sizeof(uint32_t))];
    uint32_t new_frequency = new_channel*HOP_CHANNEL_SIZE + 915000000;
    radio.set_channel( new_frequency );
}