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


#ifndef RADIO_HPP
#define RADIO_HPP

#include "mbed.h"
#include <stdint.h>
#include <memory>
#include <vector>

#if (MBED_CONF_APP_LORA_RADIO == SX1272)
#include "SX1272_LoRaRadio.h"
extern SX1272_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
#include "SX1276_LoRaRadio.h"
extern SX1276_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
#include "SX126X_LoRaRadio.h"
extern SX126X_LoRaRadio radio;
#endif

/// Two modes: normal mesh mode, and beacon mode.
typedef enum {
    MESH_MODE_NORMAL,
    MESH_MODE_BEACON,
} radio_mode_t;

typedef struct {
    radio_mode_t mode;
    uint32_t freq;
    uint32_t bw;
    uint32_t cr;
    uint32_t sf;
    uint32_t pre_len;
} nv_settings_t;

extern MbedJSONValue radio_cb;

// Main thread for working with the LoRa radio
extern Thread radio_thread;

// Initialize the radio
void init_radio(void);

// Test functions, one for transmitting and one for receiving
#define RX_DONE_SIGNAL 0xAB
void tx_test_radio(void);
void rx_test_radio(void);

typedef enum {
    TX_DONE_EVT,
    RX_DONE_EVT,
    TX_TIMEOUT_EVT,
    RX_TIMEOUT_EVT,
    RX_ERROR_EVT,
} radio_evt_enum_t;

class RadioEvent {
public:
    radio_evt_enum_t evt_enum;
    Timer timer;
    int16_t rssi;
    int8_t snr;
    std::shared_ptr<vector<uint8_t>> buf;

    RadioEvent(const radio_evt_enum_t my_evt_enum);

    RadioEvent(const radio_evt_enum_t my_evt_enum, const uint8_t *my_buf, 
                const size_t my_size, const int16_t my_rssi, const int8_t my_snr);
};

extern Mail<shared_ptr<RadioEvent>, QUEUE_DEPTH> rx_radio_evt_mail, tx_radio_evt_mail;


#endif /* RADIO_HPP */