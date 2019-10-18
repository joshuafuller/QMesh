/*
QMesh
Copyright (C) 2019 Daniel R. Fay

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
#include "rtos.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <string>
#include "radio.hpp"
#include "correct.h"

#define SX1272   0xFF
#define SX1276   0xEE
#define SX126X   0xDD

#if (MBED_CONF_APP_LORA_RADIO == SX1272)
#include "SX1272_LoRaRadio.h"
#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
#include "SX1276_LoRaRadio.h"
#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
#include "SX126X_LoRaRadio.h"
#endif

#if (MBED_CONF_APP_LORA_RADIO == SX1272)
SX1272_LoRaRadio radio(MBED_CONF_APP_LORA_SPI_MOSI,
                       MBED_CONF_APP_LORA_SPI_MISO,
                       MBED_CONF_APP_LORA_SPI_SCLK,
                       MBED_CONF_APP_LORA_CS,
                       MBED_CONF_APP_LORA_RESET,
                       MBED_CONF_APP_LORA_DIO0,
                       MBED_CONF_APP_LORA_DIO1,
                       MBED_CONF_APP_LORA_DIO2,
                       MBED_CONF_APP_LORA_DIO3,
                       MBED_CONF_APP_LORA_DIO4,
                       MBED_CONF_APP_LORA_DIO5,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL1,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL2,
                       MBED_CONF_APP_LORA_TXCTL,
                       MBED_CONF_APP_LORA_RXCTL,
                       MBED_CONF_APP_LORA_ANT_SWITCH,
                       MBED_CONF_APP_LORA_PWR_AMP_CTL,
                       MBED_CONF_APP_LORA_TCXO);

#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
SX1276_LoRaRadio radio(MBED_CONF_APP_LORA_SPI_MOSI,
                       MBED_CONF_APP_LORA_SPI_MISO,
                       MBED_CONF_APP_LORA_SPI_SCLK,
                       MBED_CONF_APP_LORA_CS,
                       MBED_CONF_APP_LORA_RESET,
                       MBED_CONF_APP_LORA_DIO0,
                       MBED_CONF_APP_LORA_DIO1,
                       MBED_CONF_APP_LORA_DIO2,
                       MBED_CONF_APP_LORA_DIO3,
                       MBED_CONF_APP_LORA_DIO4,
                       MBED_CONF_APP_LORA_DIO5,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL1,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL2,
                       MBED_CONF_APP_LORA_TXCTL,
                       MBED_CONF_APP_LORA_RXCTL,
                       MBED_CONF_APP_LORA_ANT_SWITCH,
                       MBED_CONF_APP_LORA_PWR_AMP_CTL,
                       MBED_CONF_APP_LORA_TCXO);

#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
#warning Pins being used for SX1262 Mbed board

#ifdef CDEBYTES_E22
SX126X_LoRaRadio radio(MBED_CONF_APP_LORA_SPI_MOSI, // PinName mosi
                       MBED_CONF_APP_LORA_SPI_MISO, // PinName miso
                       MBED_CONF_APP_LORA_SPI_SCLK, // PinName sclk
                       MBED_CONF_APP_LORA_CS,  // PinName nss
                       MBED_CONF_APP_LORA_RXCTL,  // PinName rxen
                       MBED_CONF_APP_LORA_TXCTL,  // PinName txen
                       MBED_CONF_APP_LORA_RESET,  // PinName reset
                       MBED_CONF_APP_LORA_DIO1,  // PinName dio1
                       MBED_CONF_APP_LORA_DIO2,  // PinName dio2
                       MBED_CONF_APP_LORA_RESET,  // PinName nrst
                       MBED_CONF_APP_LORA_BUSY);  // PinName busy,
#else
#warning Pins being used for Mbed SX1262 board
SX126X_LoRaRadio radio(D11, // PinName mosi
                       D12, // PinName miso
                       D13, // PinName sclk
                       D7,  // PinName nss
                       NC,  // PinName rxen
                       NC,  // PinName txen
                       A0,  // PinName reset
                       D5,  // PinName dio1
                       NC,  // PinName dio2   
                       NC,  // PinName nrst  
                       D3);  // PinName busy
#endif

#else
#error "Unknown LoRa radio specified (SX1272,SX1276 are valid)"
#endif

MbedJSONValue radio_cb;

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
    radio.init_radio(&radio_events);
    uint8_t radio_bw = radio_cb["BW"].get<int>();
    uint8_t radio_sf = radio_cb["SF"].get<int>();
    uint8_t radio_cr = radio_cb["CR"].get<int>();
    uint8_t radio_freq = radio_cb["Freq"].get<int>();
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
#warning This is likely to be slow in a place where speed matters!
    uint32_t new_frequency = new_channel*HOP_CHANNEL_SIZE + radio_cb["Freq"].get<int>();
    radio.set_channel(new_frequency);
}