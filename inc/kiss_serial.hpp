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
#include "peripherals.hpp"
#include "serial_data.hpp"
#include <string>
#include "Adafruit_SSD1306.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"

using crc_t = uint16_t;
using entry_size_t = uint16_t;
using read_ser_msg_err_t = enum read_ser_msg_err_enum {
    READ_SUCCESS = 0,
    READ_SER_EOF,
    READ_INVALID_KISS_ID,
    READ_ENTRY_SIZE_ERR,
    READ_MSG_OVERRUN_ERR,
    INVALID_ENTRY_SIZE,
    READ_SER_MSG_ERR,
    DECODE_SER_MSG_ERR,
    READ_CRC_ERR,
    CRC_ERR,
    INVALID_CHAR
};

using write_ser_msg_err_t = enum write_ser_msg_err_enum {
    WRITE_SUCCESS = 0,
    ENCODE_SER_MSG_ERR, 
    WRITE_SER_MSG_ERR
};

auto save_SerialMsg(const SerialMsg &ser_msg, FILE *f, bool kiss_data_msg = false) -> write_ser_msg_err_t;
auto load_SerialMsg(SerialMsg &ser_msg, FILE *f) -> read_ser_msg_err_t;

using ser_port_type_t = enum ser_port_type_enum {
    DEBUG_PORT, // both types of traffic
    VOICE_PORT, // voice/streaming only
    APRS_PORT   // data/telemetry only  
};

class KISSSerial { 
private:
    PinName tx_port, rx_port;
    UARTSerial *ser;
    bool hc05;
    bool using_stdio;
    string port_name;
    DigitalOut *en_pin;
    DigitalIn *state_pin;
    Thread *rx_ser_thread, *tx_ser_thread;
    ser_port_type_t port_type;
    Mail<std::shared_ptr<SerialMsg>, QUEUE_DEPTH> tx_ser_queue;
    vector<string> logfile_names;
    SerialMsg past_log_msg;
    atomic<bool> kiss_extended;

    void configure_hc05();
    void send_ack();
    void send_error(const string &err_str);
    /// Serial thread function that receives serial data, and processes it accordingly.
    void rx_serial_thread_fn();
    /// Produces an MbedJSONValue with the current status and queues it for transmission.
    void tx_serial_thread_fn();
    auto save_SerialMsg(const SerialMsg &ser_msg, FILE *f, bool kiss_data_msg) -> write_ser_msg_err_t;
    auto load_SerialMsg(SerialMsg &ser_msg, FILE *f) -> read_ser_msg_err_t;

public:
    void send_status();
    KISSSerial(const string &my_port_name, ser_port_type_t ser_port_type);
    KISSSerial(PinName tx, PinName Rx, const string &my_port_name, ser_port_type_t ser_port_type);
    KISSSerial(PinName tx, PinName Rx, PinName En, PinName State,
                const string &my_port_name, ser_port_type_t ser_port_type);
    ~KISSSerial();
    void enqueue_msg(shared_ptr<SerialMsg> ser_msg_sptr);
    void sleep();
    void wake();
    auto isKISSExtended() -> bool {
        return kiss_extended;
    }
};

// debug_printf() uses this vector to determine which serial ports to send out
extern Mutex kiss_sers_mtx;
extern vector<KISSSerial *> kiss_sers;
//extern UARTSerial kiss_ser_fh;


#endif /* JSON_SERIAL_HPP */