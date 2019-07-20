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
// Converts a frame into JSON, with the serialized frame being converted into Base64
void frameToJSON(Frame &frame, string &json_str) {
    size_t frame_buf_len = frame.getFullPacketSize():
    uint8_t *frame_buf = new uint8_t[frame_buf_len];
    MBED_ASSERT(frame.serialize(frame_buf) > 0);

    MbedJSONValue json_val;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, frame_buf, frame_buf_len) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, frame_buf, frame_buf_len) == 0);
    json_val["Type"] = "Frame";
    json_val["Data"] = b64_buf;
    json_str = json_val.serialize();

    delete [] frame_buf;
    delete [] b64_buf;
}

// Creates a JSON-formatted string for a given setting
void settingToJSON(string &setting, string &value, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Setting";
    json_val["Setting"] = setting;
    json_val["Value"] = value;
    json_str = json_val.serialize();
}

// Creates a JSON-formatted string for the current status
void statusToJSON(string &status, string &value, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Status";
    json_val["Status"] = status;
    json_val["Value"] = value;
    json_str = json_val.serialize();
}

// Creates a JSON-formatted string for a debug printf, with the message being
//  encoded as Base64
void dbgPrintfToJSON(string &dbg_msg, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Debug Msg";

    MbedJSONValue json_val;
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, dbg_msg.c_str(), dbg_msg.size()) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, dbg_msg.c_str(), dbg_msg.size()) == 0);
    json_val["Message"] = b64_buf;
    json_str = json_val.serialize();

    delete [] b64_buf;
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
    string b64_str = rx_json["Data"].get<string>();
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_decode(NULL, 0, &b64_len, b64_str.c_str(), b64_str.size()) == 0);
    uint8_t *frame_data = new uint8_t[b64_len];
    MBED_ASSERT(mbedtls_base64_decode(frame_data, b64_len, &b64_len, b64_str.c_str(), b64_str.size()) == 0);
    PKT_STATUS_ENUM ret_val = frame.deserialize(frame_data, b64_len);

    delete [] frame_data;

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