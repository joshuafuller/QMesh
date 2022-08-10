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

#ifndef KISS_SERIAL_HPP
#define KISS_SERIAL_HPP

#ifndef TEST_FEC
#include "mbed.h"
#endif /* TEST_FEC */
#include <string>
#include <utility>
#include <atomic>
#ifndef TEST_FEC
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#endif /* TEST_FEC */
#include "serial_msg.hpp"
#include "pseudo_serial.hpp"
#include "params.hpp"
#include "mail_queues.hpp"
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
    EventMail<std::shared_ptr<SerMsg>> tx_ser_queue;
    vector<string> logfile_names;
    SerMsg past_log_msg;
    portability::Thread *rx_ser_thread{}, *tx_ser_thread{};
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
        osStatus stat = tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
        PORTABLE_ASSERT(stat == osOK);
        stat = rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
        PORTABLE_ASSERT(stat == osOK);
    }
    auto pserRd() -> shared_ptr<PseudoSerial> * {
        return &pser_rd;
    }
    auto pserWr() -> shared_ptr<PseudoSerial> * {
        return &pser_wr;
    }

    void enqueue_msg(const shared_ptr<SerMsg> &ser_msg_sptr);
    virtual void sleep() = 0;
    virtual void wake() = 0;
};


class KISSSerialUART : public KISSSerial { 
private:
    ESP32CfgSubMsg cfg;
    bool isESP32;
    PinName tx_port, rx_port, cts_port, rts_port;
    DigitalInOut esp32_rst;
    shared_ptr<UARTSerial> ser;
    bool using_stdio;
    bool flow_control;

    void configure_esp32_bt();
    void configure_esp32_wifi();
    void set_uart_flow_ctl(FILE *ser_fh);

public:
    KISSSerialUART(const string &my_port_name, ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName rx, PinName cts, PinName rts, ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName rx, PinName rst, ESP32CfgSubMsg &my_cfg, 
                    ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts,
                    ESP32CfgSubMsg &my_cfg, ser_port_type_t ser_port_type);
    KISSSerialUART(PinName tx, PinName rx, ser_port_type_t ser_port_type);
    
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
extern Mutex *kiss_sers_mtx;
extern vector<KISSSerial *> kiss_sers;


#endif /* KISS_SERIAL_HPP */