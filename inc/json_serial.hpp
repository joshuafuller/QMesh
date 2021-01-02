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
#include "MbedJSONValue.hpp"
#include "peripherals.hpp"
#include "serial_data.hpp"
#include <string>
#include "Adafruit_SSD1306.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"

extern Mail<shared_ptr<SerialMsg>, QUEUE_DEPTH> tx_ser_queue;

/// Produces an MbedJSONValue with the current status and queues it for transmission.
void tx_serial_thread_fn(void);
/// Serial thread function that receives serial data, and processes it accordingly.
void rx_serial_thread_fn(void);

/**
 * Class that converts various data types into JSON-formatted strings for 
 * storage and/or transmission over the serial port.
 */
class JSONSerial {
protected:
    MbedJSONValue json;
public:
/** 
* Converts an MbedJSONValue settings variable into a JSON-formatted string.
* @param my_nv_settings Settings variable input.
* @param json_str JSON-formatted output string.
*/
void settingsToJSON(MbedJSONValue &nv_settings, string &json_str);

/** 
* Creates a JSON-formatted string for the current status 
* @param status Name of status value.
* @param value Status value.
* @param json_str JSON-formatted output string.
*/
void statusToJSON(string &status, string &value, string &json_str);

/**
 * Creates a JSON-formatted string for the debug_printf() function. 
 * The message is encoded as Base64.
 * @param dbg_msg Plaintext debug message (input), as a string
 * @param json_str JSON-formatted debug message (output), as a string
 */
void dbgPrintfToPB(string &dbg_msg, vector<uint8_t> &json_str);

/**
 * Loads a JSON-formatted string into the internal data structures
 * @param json_str The JSON-formatted input string.
 */
void loadJSONStr(string &json_str);

/**
 * Get the type of the JSONSerial block.
 * @param type_str The string where the type is put.
 */
void getType(string &type_str);

/**
 * Get the type of the JSONSerial block.
 * @param nv_setting MbedJSONValue with the settings loaded into it.
 */
void getSettings(MbedJSONValue &nv_setting);

};

extern JSONSerial tx_json_ser;

#endif /* JSON_SERIAL_HPP */