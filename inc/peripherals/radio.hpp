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
#include <cstdint>
#include <memory>
#include <vector>
#include "SX126X_LoRaRadio.h"
#include "serial_data.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"

extern SX126X_LoRaRadio radio;
extern SysCfgMsg radio_cb;

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

// Main thread for working with the LoRa radio
extern Thread radio_thread;

// Send out a POCSAG pager message
void send_pocsag_msg(string &msg);

// Initialize the radio
void init_radio(void);
void reinit_radio(void);
void reinit_radio_pocsag(void);

// Test functions, one for transmitting and one for receiving
#define RX_DONE_SIGNAL 0xAB
void tx_test_radio(void);
void rx_test_radio(void);

#endif /* RADIO_HPP */