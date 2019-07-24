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
#include "MbedJSONValue.h"
#include "serial_data.hpp"
#include <string>
#include "nv_settings.hpp"

extern NVSettings *nv_settings;
extern Mail<std::shared_ptr<string>, 16> tx_ser_queue;
void tx_serial_thread_fn(void);
void rx_serial_thread_fn(void);

class JSONSerial {
protected:
    MbedJSONValue json;
public:
// Creates a JSON-formatted string for a given setting
void settingsToJSON(nv_settings_t &nv_settings, string &json_str);

// Creates a JSON-formatted string for the current status
void statusToJSON(string &status, string &value, string &json_str);

// Creates a JSON-formatted string for a debug printf, with the message being
//  encoded as Base64
void dbgPrintfToJSON(string &dbg_msg, string &json_str);

// Loads a JSON-formatted string into the internal data structures
void loadJSONStr(string &json_str);

// Returns the type of the message
void getType(string &type_str);

// Loads a setting from the JSON string
void getSettings(nv_settings_t &nv_setting);

// Get the JSON object. Needed to initialize a Frame.
MbedJSONValue *getJSONObj(void);
};

extern JSONSerial tx_json_ser;

#endif /* JSON_SERIAL_HPP */