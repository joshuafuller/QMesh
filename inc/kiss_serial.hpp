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
#include <string>
#include <utility>
#include <atomic>
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial_msg.hpp"
#include "pseudo_serial.hpp"
#include "params.hpp"
#if MBED_CONF_APP_HAS_BLE == 1
#include "ble/BLE.h"
#include "ble_serial.hpp"
#endif /* MBED_CONF_APP_HAS_BLE == 1 */


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

auto save_SerMsg(SerMsg &ser_msg, PseudoSerial &ps, bool kiss_data_msg = false) -> write_ser_msg_err_t;
auto load_SerMsg(SerMsg &ser_msg, PseudoSerial &ps) -> read_ser_msg_err_t;

class KISSSerial {
private:
    Mail<std::shared_ptr<SerMsg>, QUEUE_DEPTH> tx_ser_queue;
    vector<string> logfile_names;
    SerMsg past_log_msg;
    Thread *rx_ser_thread{}, *tx_ser_thread{};
    ser_port_type_t port_type;
    string port_name;
    atomic<bool> kiss_extended{};
    static constexpr int SER_THREAD_STACK_SIZE = 8192;
    /// Serial thread function that receives serial data, and processes it accordingly.
    void rx_serial_thread_fn();
    /// Produces an MbedJSONValue with the current status and queues it for transmission.
    void tx_serial_thread_fn();
    shared_ptr<PseudoSerial> pser_rd, pser_wr;
    static auto save_SerMsg(SerMsg &ser_msg, PseudoSerial &ps, bool kiss_data_msg) -> write_ser_msg_err_t;
    auto load_SerMsg(SerMsg &ser_msg, PseudoSerial &ps) -> read_ser_msg_err_t;

public:
    KISSSerial(string my_port_name, ser_port_type_t ser_port_type);
    ~KISSSerial();
    KISSSerial(const KISSSerial &) = delete;
    auto operator=(const KISSSerial &) -> KISSSerial& = delete;
    KISSSerial(KISSSerial &&) = delete;
    auto operator=(KISSSerial &&) -> KISSSerial& = delete;

    void send_status();
    void send_ack();
    void send_error(const string &err_str);
    auto isKISSExtended() -> bool { return kiss_extended; }
    auto portName() -> string { return port_name; }
    //auto rxSerThread() -> Thread ** { return &rx_ser_thread; }
    //auto txSerThread() -> Thread ** { return &tx_ser_thread; }
    void kissExtended(const bool val) { kiss_extended = val; }
    void startThreads() {
        tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
        rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
    }
    auto pserRd() -> shared_ptr<PseudoSerial> * {
        return &pser_rd;
    }
    auto pserWr() -> shared_ptr<PseudoSerial> * {
        return &pser_wr;
    }

    void enqueue_msg(shared_ptr<SerMsg> ser_msg_sptr);
    virtual void sleep() = 0;
    virtual void wake() = 0;
};


class KISSSerialUART : public KISSSerial { 
private:
    PinName tx_port, rx_port;
    shared_ptr<UARTSerial> ser;
    bool hc05;
    bool using_stdio;
    DigitalOut *en_pin;
    DigitalIn *state_pin;

    void configure_hc05();

public:
    KISSSerialUART(const string &my_port_name, ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName Rx, const string &my_port_name, ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName Rx, PinName En, PinName State,
                const string &my_port_name, ser_port_type_t ser_port_type);
    ~KISSSerialUART();
    KISSSerialUART(const KISSSerialUART &) = delete;
    auto operator=(const KISSSerialUART &) -> KISSSerialUART& = delete;
    KISSSerialUART(KISSSerialUART &&) = delete;
    auto operator=(KISSSerialUART &&) -> KISSSerialUART& = delete;

    void sleep() override;
    void wake() override;
};


#if MBED_CONF_APP_HAS_BLE == 1
class KISSSerialBLE : public KISSSerial { 
public:
    explicit KISSSerialBLE(ser_port_type_t ser_port_type);
    KISSSerialBLE(const string &my_port_name, ser_port_type_t ser_port_type) : 
            KISSSerial(my_port_name, ser_port_type) {
        *pserRd() = make_shared<BLEPseudoSerial>(ser_port_type);
        *pserWr() = make_shared<BLEPseudoSerial>(ser_port_type);
    }
    KISSSerialBLE(const KISSSerialBLE &) = delete;
    auto operator=(const KISSSerialBLE &) -> KISSSerialBLE& = delete;
    KISSSerialBLE(KISSSerialBLE &&) = delete;
    auto operator=(KISSSerialBLE &&) -> KISSSerialBLE& = delete;

    void enqueue_msg(shared_ptr<SerMsg> ser_msg_sptr);
    void sleep() override { };
    void wake() override { };
};
#endif /* MBED_CONF_APP_HAS_BLE == 1 */


// debug_printf() uses this vector to determine which serial ports to send out
extern Mutex kiss_sers_mtx;
extern vector<KISSSerial *> kiss_sers;


#endif /* JSON_SERIAL_HPP */