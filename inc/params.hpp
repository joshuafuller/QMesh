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

#ifndef PARAMS_HPP
#define PARAMS_HPP

// Defines key parameters

// Test mode selection
#define TX_TEST_MODE
//#define RX_TEST_MODE

// Filesystem parameters. If HEAP_FS isn't defined, then use the external
// SPI flash
//#define HEAP_FS

// Frame parameters
#define FRAME_PAYLOAD_LEN 16

// Radio parameters
#define RADIO_INVERT_IQ false
#define RADIO_HOP_PERIOD 1
#define RADIO_FREQ_HOP false
#define RADIO_CRC_ON false
#define RADIO_FIXED_LEN true
#define RADIO_SYM_TIMEOUT 512
#define RADIO_PREAMBLE_LEN 12
// 0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved
#define RADIO_BANDWIDTH 1
#define RADIO_SF 12
// 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
#define RADIO_CODERATE 1
// Tx power, in mW
#define RADIO_POWER 22
// Transmit timeout, in ms
#define RADIO_TX_TIMEOUT 3000
#define RADIO_FREQUENCY 915000000
#define HOP_CHANNEL_SIZE 12500
// Number of past packets to check the current packet against
#define PKT_CHK_HISTORY 32
// Maximum frame size. Currently the LoRa frame max, but may expand
//  to accommodate fragmentation in the future.
#define MAX_FRAME_SIZE 256

// Various parameters for the TDMA meshing
#define FRAME_PADDING_SYMS 8 // Amount of padding between frames, in symbols
#define NUM_PREAMBLE_SLOTS 4 // Number of TDMA slots for the preamble offsets

#define RADIO_BEACON_INTERVAL 600
#define RADIO_BEACON_MSG "KG5VBY Auto Station\r\n"

// Frequency wobble parameters
#define FREQ_WOBBLE_PROPORTION 0.1f // Fraction of the bandwidth we'll wobble over

// Symbol delay granularity
#define SYM_FRAC_DELAY_SLOTS 8

// FEC parameters
#define FEC_CONV
//#define FEC_RSV

#define LOG_BASEADDR 1024

// Debug options
#define DEBUG_INFO
#define DEBUG_WARN
#define DEBUG_ERR

#define SX1272   0xFF
#define SX1276   0xEE
#define SX126X   0xDD

#define CDEBYTES_E22

#include "sx126x_ds.h"
#define LORA_DEVICE SX1262
#ifdef CDEBYTES_E22
#define USES_TCXO
#define TCXO_VOLTAGE TCXO_CTRL_1_8V
#else
#endif

#endif /* PARAMS_HPP */