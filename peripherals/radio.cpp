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

// The callbacks used by the LoRa radio driver
static radio_events_t radio_events;

// Event queue for communicating events from the radio
Mail<shared_ptr<RadioEvent>, 16> tx_radio_evt_mail, rx_radio_evt_mail;

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
    printf("after init_radio\r\n");
    uint8_t radio_bw = nv_settings.getBW();
    uint8_t radio_sf = nv_settings.getSF();
    uint8_t radio_cr = nv_settings.getCR();
    uint8_t radio_freq = nv_settings.getFrequency();
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
    radio_timing.computeTimes(RADIO_BANDWIDTH, RADIO_SF, RADIO_CODERATE, RADIO_PREAMBLE_LEN, full_pkt_len);
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum) {
    timer.start();
    evt_enum = my_evt_enum;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, const uint8_t *my_buf, 
        const size_t my_size, const int16_t my_rssi, const int8_t my_snr) {
    timer.start();
    evt_enum = my_evt_enum;
    MBED_ASSERT(my_size <= 256);
    buf = std::make_shared<vector<uint8_t>>();
    buf->resize(my_size);
    memcpy(buf->data(), my_buf, my_size);
    rssi = my_rssi;
    snr = my_snr;
}

static void tx_done_cb(void)
{
    // If we just finished retransmitting a frame
    //  Don't do anything, just let the meshing receive another frame
    // If we just finished transmitting a local frame
    //  check to see if another frame's sitting in the queue
    //  if so, grab another frame and set it up to be sent one time unit in the future
    auto radio_event = make_shared<RadioEvent>(TX_DONE_EVT);
    MBED_ASSERT(!tx_radio_evt_mail.full());
    radio.set_channel(RADIO_FREQUENCY);
    tx_radio_evt_mail.put(&radio_event);
    debug_printf(DBG_INFO, "TX Done interrupt generated\r\n");
}

static void rx_done_cb(uint8_t const *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    auto radio_event = make_shared<RadioEvent>(RX_DONE_EVT, payload, (size_t) size, rssi, snr);
    MBED_ASSERT(!rx_radio_evt_mail.full());
    radio.set_channel(RADIO_FREQUENCY);
    rx_radio_evt_mail.put(&radio_event);
    debug_printf(DBG_INFO, "RX Done interrupt generated\r\n");    
}
 
static void tx_timeout_cb(void)
{
    auto radio_event = make_shared<RadioEvent>(TX_TIMEOUT_EVT);
    MBED_ASSERT(!tx_radio_evt_mail.full());
    radio.set_channel(RADIO_FREQUENCY);
    tx_radio_evt_mail.put(&radio_event);    
    debug_printf(DBG_ERR, "Tx Timeout\r\n");
}
 
static void rx_timeout_cb(void)
{
    auto radio_event = make_shared<RadioEvent>(RX_TIMEOUT_EVT);
    MBED_ASSERT(!rx_radio_evt_mail.full());
    radio.set_channel(RADIO_FREQUENCY);
    rx_radio_evt_mail.put(&radio_event);   
    debug_printf(DBG_ERR, "Rx Timeout\r\n");
}
 
static void rx_error_cb(void)
{
    auto radio_event = make_shared<RadioEvent>(RX_ERROR_EVT);
    MBED_ASSERT(!rx_radio_evt_mail.full());
    radio.set_channel(RADIO_FREQUENCY);
    rx_radio_evt_mail.put(&radio_event);   
    debug_printf(DBG_ERR, "Rx Error\r\n");
}

static void fhss_change_channel_cb(uint8_t current_channel) {
    int32_t new_channel = hopping_channels[current_channel % 
                (sizeof(hopping_channels)/sizeof(uint32_t))];
    uint32_t new_frequency = new_channel*HOP_CHANNEL_SIZE + 915000000;
    radio.set_channel(new_frequency);
}