#ifndef BLE_SERIAL_HPP
#define BLE_SERIAL_HPP
#if MBED_CONF_APP_HAS_BLE == 1

#include "os_portability.hpp"
#include <string>
#include <utility>
#include <fstream>
#include <deque>
#include "Adafruit_SSD1306.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial_msg.hpp"
#include "ble/BLE.h"
#include "UARTService.h"
#include "gatt_server_process.h"


class UARTServiceBlocking : public UARTService {
private:
    static constexpr int BUF_SIZE = 32;
    Queue<char, BUF_SIZE> data_buf;
public:
    explicit UARTServiceBlocking(BLE &_ble) : UARTService(_ble) {
        ble.onDataWritten(this, &UARTServiceBlocking::onDataWritten);
    }

    auto putc(int c) -> int {
        return _putc(c);
    }

    auto getc() -> int {
        osEvent evt = data_buf.get();
        if(evt.status == osEventMessage) {
            char *val = static_cast<char *>(evt.value.p);
            return *val;
        }
        PORTABLE_ASSERT(false);
    }

    void onDataWritten(const GattWriteCallbackParams *params) {
        if (params->handle == getTXCharacteristicHandle()) {
            uint16_t bytesRead = params->len;
            if (bytesRead <= BLE_UART_SERVICE_MAX_DATA_LEN) {
                for(int i = 0; i < bytesRead; i++) {
                    if(!data_buf.full()) {
                        char val = params->data[i];
                        data_buf.put(&val);
                    } else {
                        break;
                    }
                }
            }
        }
    }
};


class UARTKISSService : public UARTServiceBlocking {
private:
    static constexpr uint8_t KISS_FRAME_END = 0xC0;
public:
    explicit UARTKISSService(BLE &_ble) : UARTServiceBlocking(_ble) {};
    auto putc(int c) -> int {
        int err = UARTServiceBlocking::putc(c);
        if(c == KISS_FRAME_END) {
            flush();
        }
        return err;
    }
};


extern EventQueue background_queue;

class BLESerial {
private:
    UARTKISSService *uart_service;
public:
    BLESerial() {
        BLE &ble = BLE::Instance();
        uart_service = new UARTKISSService(ble);
    }


};



#endif /* MBED_CONF_APP_HAS_BLE == 1 */
#endif /* BLE_SERIAL_HPP */