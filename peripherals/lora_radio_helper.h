/**
 * Copyright (c) 2017, Arm Limited and affiliates.
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

#include "lorawan/LoRaRadio.h"

#ifndef APP_LORA_RADIO_HELPER_H_
#define APP_LORA_RADIO_HELPER_H_

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
SX126X_LoRaRadio radio(D11, // PinName mosi
                       D12, // PinName miso
                       D13, // PinName sclk
                       D10,  // PinName nss
                       D7,  // PinName rxen
                       D6,  // PinName txen
                       A0,  // PinName reset
                       D5,  // PinName dio1
                       D4,  // PinName dio2
                       D3,  // PinName nrst
                       D2,  // PinName busy,
                       A1,  // PinName freq_select,
                       A2,  // PinName device_select,
                       A3,  // PinName crystal_select,
                       D8); // PinName ant_switch);
#else
#error "Unknown LoRa radio specified (SX1272,SX1276 are valid)"
#endif

#endif /* APP_LORA_RADIO_HELPER_H_ */
