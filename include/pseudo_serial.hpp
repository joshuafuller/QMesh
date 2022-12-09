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

#ifndef PSEUDO_SERIAL_HPP
#define PSEUDO_SERIAL_HPP

#ifndef TEST_FEC
#include "os_portability.hpp"
#endif /* TEST_FEC */
#include <map>
#include <vector>
#include <string>
#include <utility>
#include <fstream>
#include <deque>
#include <memory>
#include <atomic>
#include "qmesh.pb.h"
#include "platform/ATCmdParser.h"


class PseudoSerial {
public:
    static auto safe_pcts(const string &send_str) -> string;
    virtual auto putc(int val) -> int = 0;
    virtual auto getc() -> int = 0;
};


class UARTPseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    ~UARTPseudoSerial() {
        fclose(f_rd);
        fclose(f_wr);
    }

    UARTPseudoSerial(const UARTPseudoSerial &obj) = delete;
    auto operator= (UARTPseudoSerial &&) -> UARTPseudoSerial & = delete;
    UARTPseudoSerial(UARTPseudoSerial &&) = delete;
    auto operator=(const UARTPseudoSerial &) -> UARTPseudoSerial & = delete; 	

    explicit UARTPseudoSerial(const shared_ptr<UARTSerial>& ser, const bool read) :
        f_rd(nullptr),
        f_wr(nullptr)
    {
        if(read) {
            f_rd = fdopen(&*ser, "r");
            PORTABLE_ASSERT(f_rd != nullptr);
        } else {
            f_wr = fdopen(&*ser, "w");
            PORTABLE_ASSERT(f_wr != nullptr);
        }
    }

    auto putc(const int val) -> int override {
        PORTABLE_ASSERT(f_wr != nullptr);
        return fputc(val, f_wr);
    }

    auto getc() -> int override {
        PORTABLE_ASSERT(f_rd != nullptr);
        return fgetc(f_rd);
    }
};


using ser_port_type_t = enum ser_port_type_enum {
    DATA_PORT = 0, // both types of traffic
    VOICE_PORT, // voice/streaming only
    DEBUG_PORT   // data/telemetry only  
};


#if MBED_CONF_APP_HAS_BLE == 0
class ESP32PseudoSerial : public PseudoSerial {
private:
    static constexpr int RX_BUF_SIZE = 128;
    static constexpr int MAX_NUM_CMDS = 16;
    static constexpr uint8_t KISS_FEND = 0xC0;
    vector<uint8_t> outbuf;
    ESP32CfgSubMsg cfg;
    map<int, bool> tcp_conns;
    shared_ptr<ATCmdParser> at_parser;
    deque<uint8_t> recv_data;
    
    PinName tx_port, rx_port, cts_port, rts_port;
    DigitalInOut esp32_rst;
    atomic<bool> bt_active;
    shared_ptr<UARTSerial> ser;
    portability::mutex ser_mtx, recv_data_mtx;

    atomic<int32_t> idle_time;
    auto putc(int val, bool dummy_char) -> int;

public:
    ~ESP32PseudoSerial() = default;

    ESP32PseudoSerial(const ESP32PseudoSerial &obj) = delete;
    auto operator= (ESP32PseudoSerial &&) -> ESP32PseudoSerial & = delete;
    ESP32PseudoSerial(ESP32PseudoSerial &&) = delete;
    auto operator=(const ESP32PseudoSerial &) -> ESP32PseudoSerial & = delete; 	

    ESP32PseudoSerial(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts, ESP32CfgSubMsg &my_cfg);

    void at_callback_bt();
    void at_callback_tcp();

    auto putc(int val) -> int override;
    auto getc() -> int override;
};

#endif /* #if MBED_CONF_APP_HAS_BLE == 0 */


class FilePseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    explicit FilePseudoSerial(FILE *f) :
        f_rd(f),
        f_wr(f) { }

    auto putc(const int val) -> int override {
        return fputc(val, f_wr);
    }

     auto getc() -> int override {
        return fgetc(f_rd);
    }
};


using esp32_cfg_t = enum esp32_cfg_enum {
    NONE,
    BT,
    WIFI_AP,
    WIFI_STA,
};


#if MBED_CONF_APP_HAS_BLE == 1
static constexpr int BLE_QUEUE_SIZE = 16;
extern Mail<pair<ser_port_type_t, shared_ptr<vector<uint8_t>>>, BLE_QUEUE_SIZE> ble_out_queue;
extern Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> voice_in_queue;
extern Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> aprs_in_queue;
extern Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> dbg_in_queue;
class BLEPseudoSerial : public PseudoSerial {
private:
    static constexpr int BLE_MAX_PDU = 64;
    static constexpr int BLE_SER_QUEUE_LEN = BLE_MAX_PDU*4;
    deque<uint8_t> tx_queue;
    deque<uint8_t> rx_queue;
    ser_port_type_t ser_type;
    Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> *my_in_queue;
    static constexpr uint8_t KISS_FEND = 0xC0;

    void checkAndSend(const uint8_t last_val) {
        PORTABLE_ASSERT(!tx_queue.empty());
        if(last_val == KISS_FEND || tx_queue.size() == BLE_MAX_PDU) {
            auto send_data_blob = make_shared<vector<uint8_t>>();
            for(int i = 0; i < BLE_MAX_PDU; i++) {
                send_data_blob->push_back(*tx_queue.begin());
                tx_queue.pop_front();
            }
            if(!ble_out_queue.full()) {
                auto *mail_item = ble_out_queue.alloc();
                PORTABLE_ASSERT(mail_item != nullptr);
                *mail_item = pair<ser_port_type_t, shared_ptr<vector<uint8_t>>>(ser_type, send_data_blob);
                ble_out_queue.put(mail_item);
            }
        } 
    }

    void checkAndGet() {
        if(rx_queue.empty()) {
            osEvent evt = my_in_queue->get();
            if(evt.status == osEventMail) {
                auto mail_item = *(static_cast<shared_ptr<vector<uint8_t>> *>(evt.value.p));
                my_in_queue->free(static_cast<shared_ptr<vector<uint8_t>> *>(evt.value.p));
                for(unsigned char & it : *mail_item) {
                    rx_queue.push_back(it);
                }
            } else {
                PORTABLE_ASSERT(false);
            }
        } 
    }

public:
    explicit BLEPseudoSerial(ser_port_type_t my_ser_type) : ser_type(my_ser_type) { 
        if(ser_type == DEBUG_PORT) {
            my_in_queue = &dbg_in_queue;
        } else if(ser_type == VOICE_PORT) {
            my_in_queue = &voice_in_queue;
        } else {
            PORTABLE_ASSERT(false);
        }
    }

    auto putc(const int val) -> int override {
        tx_queue.push_back(val);
        checkAndSend(val);
        return val;
    }

    auto getc() -> int override {
        checkAndGet();
        auto val = *rx_queue.begin();
        rx_queue.pop_front();
        return val;
    }
};
#endif /* MBED_CONF_APP_HAS_BLE == 1 */

// ESP32 pseudo-serial
// What does this need to do?
//  It needs to:
//    0. Reset the ESP32 on startup
//    1. Properly set up the ESP32 to be able to accept TCP connections
//    2. Accept TCP connections
//    3. Accept multiple TCP connections
//    4. Distribute out KISS data to the different clients
//    5. Eventually, be able to use a magic byte on Codec 2 Walkie-Talkie to distinguish between voice and data
//    6. Ignore certain classes of OOB data, like the +DIST_STA_IP strings
//    7. +IPD,<connection>,<data size> is the received data
//    8. <connection>,CLOSED is the TCP/IP socket closing
//    9. <connection>,CONNECT is the TCP/IP socket opening 
//   10. Use different KISS TNC ports for different types of data (debug, voice, data)
//   11. Have multiple ESP32 pseudo serial ports created (debug, voice, data). Each one operates on the same ESP32 object.

/// Manages the possibly-multiple connections of an ESP32
static constexpr uint32_t SER_BAUD_RATE = 115200;
static constexpr uint32_t ESP_BAUD_RATE = 115200;
static constexpr int SSID_MAX_LEN = 16;
static constexpr int PASS_MAX_LEN = 32;
static constexpr int QUARTER_SECOND = 250;
static constexpr int HALF_SECOND = 500;
static constexpr int ONE_SECOND = 1000;

#endif /* PSEUDO_SERIAL_HPP */