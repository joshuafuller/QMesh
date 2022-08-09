/*
QMesh
Copyright (C) 2022 Daniel R. Fay

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


/**
 * Core function that periodically sends out a beacon message.
 */
void beacon_fn();

void oled_mon_fn();


#endif /* MESH_PROTOCOL_HPP */