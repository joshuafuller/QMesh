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
#ifndef RADIO_HPP
#define RADIO_HPP

#include "mbed.h"
#include <stdint.h>

// Radio settings struct. 
// Why? Because it makes it easy to save and restore the state to NVRAM
typedef struct {
    uint32_t bw;
    uint32_t sf;
    uint8_t cr;
    uint32_t freq;
} nv_radio_settings_t;


union {
    nv_radio_settings_t radio_settings;
    uint8_t buf[sizeof(nv_radio_settings_t)];
} nv_radio_settings_union;


#endif /* RADIO_HPP */