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

typedef uint16_t crc_t;
typedef uint16_t entry_size_t;
typedef enum {
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
} read_ser_msg_err_t;

typedef enum {
    WRITE_SUCCESS = 0,
    ENCODE_SER_MSG_ERR, 
    WRITE_SER_MSG_ERR
} write_ser_msg_err_t;

write_ser_msg_err_t save_SerialMsg(const SerialMsg &ser_msg, FILE *f, const bool kiss_data_msg = false);
read_ser_msg_err_t load_SerialMsg(SerialMsg &ser_msg, FILE *f);

typedef enum {
    DEBUG_PORT, // both types of traffic
    VOICE_PORT, // voice/streaming only
    APRS_PORT   // data/telemetry only  
} ser_port_type_t;

class KISSSerial { 
protected:
    UARTSerial *ser;
    bool using_stdio;
    Thread *rx_ser_thread, *tx_ser_thread;
    ser_port_type_t port_type;
    Mail<std::shared_ptr<SerialMsg>, QUEUE_DEPTH> tx_ser_queue;
    vector<string> logfile_names;
    SerialMsg past_log_msg;

    void send_ack(void);
    void send_error(const string &err_str);
    /// Serial thread function that receives serial data, and processes it accordingly.
    void rx_serial_thread_fn(void);
    /// Produces an MbedJSONValue with the current status and queues it for transmission.
    void tx_serial_thread_fn(void);

public:
    atomic<bool> kiss_mode;
    void send_status(void);
    KISSSerial(const string &port_name, const ser_port_type_t ser_port_type);
    KISSSerial(UARTSerial &ser_port, const string &port_name, const ser_port_type_t ser_port_type);
    ~KISSSerial();
    void enqueue_msg(shared_ptr<SerialMsg> ser_msg_sptr);
};

// debug_printf() uses this vector to determine which serial ports to send out
extern Mutex kiss_sers_mtx;
extern vector<KISSSerial *> kiss_sers;
//extern UARTSerial kiss_ser_fh;


#endif /* JSON_SERIAL_HPP */