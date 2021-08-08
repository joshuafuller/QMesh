#include "mbed.h"
#include "pseudo_serial.hpp"
#include <vector>
#include <utility>
#include <memory>

#if MBED_CONF_APP_HAS_BLE == 1
Mail<pair<ser_port_type_t, shared_ptr<vector<uint8_t>>>, BLE_QUEUE_SIZE> ble_out_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> voice_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> aprs_in_queue;
Mail<shared_ptr<vector<uint8_t>>, BLE_QUEUE_SIZE> dbg_in_queue;
#endif /* MBED_CONF_APP_HAS_BLE == 1 */