#ifndef BLE_SERIAL_HPP
#define BLE_SERIAL_HPP
#if MBED_CONF_APP_HAS_BLE == 1

#include "mbed.h"
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
#include "gatt_server_process.h"



class BLESerial {
private:

public:
    BLESerial() {
        BLE &ble = BLE::Instance();
    }

    void sendData(const shared_ptr<vector<uint8_t>>& buf) {

    }
};



#endif /* MBED_CONF_APP_HAS_BLE == 1 */
#endif /* BLE_SERIAL_HPP */