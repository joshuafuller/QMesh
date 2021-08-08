#ifndef PSEUDO_SERIAL_HPP
#define PSEUDO_SERIAL_HPP

#include "mbed.h"
#include <vector>
#include <string>
#include <utility>
#include <fstream>
#include <deque>
#include <memory>


class PseudoSerial {
public:
    virtual auto putc(int val) -> int = 0;
    virtual auto getc() -> int = 0;
};


class UARTPseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    explicit UARTPseudoSerial(UARTSerial &ser, const bool read) {
        f_rd = nullptr;
        f_wr = nullptr;
        if(read) {
            f_rd = fdopen(&ser, "r");
        } else {
            f_wr = fdopen(&ser, "w");
        }
    }

     auto putc(const int val) -> int override {
        MBED_ASSERT(f_wr != nullptr);
        return fputc(val, f_wr);
    }

     auto getc() -> int override {
        MBED_ASSERT(f_rd != nullptr);
        return fgetc(f_rd);
    }
};


class FilePseudoSerial : public PseudoSerial {
private:
    FILE *f_rd, *f_wr;
public:

    explicit FilePseudoSerial(FILE *f) {
        f_rd = f;
        f_wr = f;
    }

    auto putc(const int val) -> int override {
        return fputc(val, f_wr);
    }

     auto getc() -> int override {
        return fgetc(f_rd);
    }
};


using ser_port_type_t = enum ser_port_type_enum {
    DEBUG_PORT, // both types of traffic
    VOICE_PORT, // voice/streaming only
    APRS_PORT   // data/telemetry only  
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
        MBED_ASSERT(!tx_queue.empty());
        if(last_val == KISS_FEND || tx_queue.size() == BLE_MAX_PDU) {
            auto send_data_blob = make_shared<vector<uint8_t>>();
            for(int i = 0; i < BLE_MAX_PDU; i++) {
                send_data_blob->push_back(*tx_queue.begin());
                tx_queue.pop_front();
            }
            if(!ble_out_queue.full()) {
                auto *mail_item = ble_out_queue.alloc();
                MBED_ASSERT(mail_item != nullptr);
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
                MBED_ASSERT(false);
            }
        } 
    }

public:
    explicit BLEPseudoSerial(ser_port_type_t my_ser_type) : ser_type(my_ser_type) { 
        if(ser_type == DEBUG_PORT) {
            my_in_queue = &dbg_in_queue;
        } else if(ser_type == APRS_PORT) {
            my_in_queue = &aprs_in_queue;
        } else if(ser_type == VOICE_PORT) {
            my_in_queue = &voice_in_queue;
        } else {
            MBED_ASSERT(false);
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

#endif /* PSEUDO_SERIAL_HPP */