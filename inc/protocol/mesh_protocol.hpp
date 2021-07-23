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

#ifndef MESH_PROTOCOL_HPP
#define MESH_PROTOCOL_HPP

#include "mbed.h"
#include "radio.hpp"

enum class system_state_t { BOOTING, MANAGEMENT, RUNNING };

extern system_state_t current_mode;
extern atomic<bool> stay_in_management;

/**
 * Core function that implmements the QMesh mesh protocol.
 */
void mesh_protocol_fsm();

extern string beacon_msg;

/**
 * Core function that periodically sends out a beacon message.
 */
void beacon_fn();

void oled_mon_fn();


#endif /* MESH_PROTOCOL_HPP */