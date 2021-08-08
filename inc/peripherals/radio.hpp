/*
QMesh
Copyright (C) 2021 Daniel R. Fay

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
#include "SX126X_LoRaRadio.h"

extern SX126X_LoRaRadio radio;

/// Two modes: normal mesh mode, and beacon mode.
enum class radio_mode_t { MESH_MODE_NORMAL, MESH_MODE_BEACON, };
using nv_settings_t = struct { 
    radio_mode_t mode; 
    uint32_t freq;
    uint32_t bw;
    uint32_t cr;
    uint32_t sf;
    uint32_t pre_len;
};

// Main thread for working with the LoRa radio
extern Thread radio_thread;

// Initialize the radio
void init_radio();
void reinit_radio();

// Test functions, one for transmitting and one for receiving
constexpr int RX_DONE_SIGNAL = 0xAB;
void tx_test_radio();
void rx_test_radio();

#endif /* RADIO_HPP */