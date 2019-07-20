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

class JSONSerial {
protected:
    MbedJSONValue rx_json;
public:
// Converts a frame into JSON, with the frame being converted into Base64
void frameToJSON(Frame &frame, string &json_str) {
    size_t frame_buf_len = frame.getFullPacketSize():
    uint8_t *frame_buf = malloc(frame_buf_len);
    MBED_ASSERT(frame.serialize(frame_buf) > 0);

    MbedJSONValue json_val;
    size_t b64_len;
    int base64_size = mbedtls_base64_encode(NULL, 0, &b64_len, frame_buf, frame_buf_len);
    unsigned char b64_buf = malloc(b64_len);
    json_val["Type"] = "Frame";
    json_val["Data"] = b64_buf;
    json_str = json_val.serialize();
        
    free(buf);
    free(b64_buf);
}

// Creates a JSON-formatted string for a given setting
void settingToJSON(string &setting, string &value, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Setting";
    json_val["Setting"] = setting;
    json_val["Value"] = value;
    json_str = json_val.serialize();
}

// Creates a JSON-formatted string for a debug printf
void dbgPrintfToJSON(string &dbg_msg, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Debug Msg";
    json_val["Message"] = dbg_msg;
    json_str = json_val.serialize();
}

// Loads a JSON-formatted string into the internal data structures
void loadJSONStr(string &rx_json_str) {
    parse(rx_json, rx_json_str);
}

// Returns the type of the message
void getType(string &type_str) {
    type_str = rx_json["Type"].get<string>();
}

// Loads a Frame from the JSON string
PKT_STATUS_ENUM getFrame(Frame &frame) {
    MBED_ASSERT(getType() == "Frame");
    size_t b64_len;
    string b64_str = rx_json["Data"].get<string>();
    MBED_ASSERT(mbedtls_base64_decode(NULL, 0, &b64_len, b64_str.c_str(), b64_str.size()) == 0);
    uint8_t *frame_data = malloc(b64_len);
    MBED_ASSERT(mbedtls_base64_decode(frame_data, b64_len, &b64_len, b64_str.c_str(), b64_str.size()) == 0);
    PKT_STATUS_ENUM ret_val = frame.deserialize(frame_data, b64_len);

    free(frame_data);

    return ret_val;
}

// Loads a setting from the JSON string
void getSetting(string &setting, string &value) {
    MBED_ASSERT(getType() == "Setting");
    setting = rx_json["Setting"].get<string>();
    value = rx_json["Value"].get<string>();
}

};

#endif /* JSON_SERIAL_HPP */