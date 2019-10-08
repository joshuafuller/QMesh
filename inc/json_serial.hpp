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

#ifndef JSON_SERIAL_HPP
#define JSON_SERIAL_HPP

/* 
    This code exists to allow data to go across the serial port as JSON-formatted
    data. Doing so allows for different types of data (frames, debug messages, 
    configuration commands) to use the same UART simultaneously, while keeping
    all data being conveyed as at least somewhat-readable using just a serial 
    terminal application.

    To facilitate readability within a terminal application, any binary data is 
    encoded as Base64.
*/

#include "mbed.h"
#include "MbedJSONValue.h"
#include "peripherals.hpp"
#include "serial_data.hpp"
#include <string>
#include "mesh_protocol.hpp"

extern Mail<std::shared_ptr<string>, 16> tx_ser_queue;
void tx_serial_thread_fn(void);
void rx_serial_thread_fn(void);

class JSONSerial {
protected:
    MbedJSONValue json;
public:
// Creates a JSON-formatted string for a given setting
void settingsToJSON(MbedJSONValue &nv_settings, string &json_str);

// Creates a JSON-formatted string for the current status
void statusToJSON(string &status, string &value, string &json_str);

// Creates a JSON-formatted string for a debug printf, with the message being
//  encoded as Base64
void dbgPrintfToJSON(string &dbg_msg, string &json_str);

// Loads a JSON-formatted string into the internal data structures
void loadJSONStr(string &json_str);

// Returns the type of the message
void getType(string &type_str);

// Loads a setting from the JSON string
void getSettings(MbedJSONValue &nv_setting);

// Get the JSON object. Needed to initialize a Frame.
MbedJSONValue *getJSONObj(void);
};

extern JSONSerial tx_json_ser;

#endif /* JSON_SERIAL_HPP */