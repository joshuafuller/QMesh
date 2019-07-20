#include "json_serial.hpp"
#include "mbed.h"
#include "MbedJSONValue.h"
#include <string>
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"

Queue<string, 16> tx_ser_queue;
void tx_serial_thread_fn(void) {
    for(;;) {
        osEvent evt = tx_ser_queue.get();
        if(evt.status == osEventMessage) {
            string *str = (string *) evt.value.p;
            printf("%s\r\n", str->c_str());
            delete str;
        }
    }
}

static char rx_str[2048];
void rx_serial_thread_fn(void) {
    for(;;) {
        scanf("%s\r\n", rx_str);
    }
    string *rx_string = new string(rx_str);
    rx_json_ser.loadJSONStr(*rx_string);
#warning Need to actually do stuff here
    string *type_string = new string();
    rx_json_ser.getType(*type_string);
    if(*type_string == "Setting") {

    }
    else if(*type_string == "Status") {

    }
    else if(*type_string == "Debug Msg") {
        MBED_ASSERT(false);
    }
    else if(*type_string == "Frame") {
        Frame *frame = new Frame();
        frame->loadFromJSON(*rx_json_ser.getJSONObj());
    }
    else {
        MBED_ASSERT(false);
    }

    delete type_string;
    delete rx_string;
}


// Creates a JSON-formatted string for a given setting
void JSONSerial::settingToJSON(string &setting, string &value, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Setting";
    json_val["Setting"] = setting;
    json_val["Value"] = value;
    json_str = json_val.serialize();
}

// Creates a JSON-formatted string for the current status
void JSONSerial::statusToJSON(string &status, string &value, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Status";
    json_val["Status"] = status;
    json_val["Value"] = value;
    json_str = json_val.serialize();
}

// Creates a JSON-formatted string for a debug printf, with the message being
//  encoded as Base64
void JSONSerial::dbgPrintfToJSON(string &dbg_msg, string &json_str) {
    MbedJSONValue json_val;
    json_val["Type"] = "Debug Msg";
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);
    json_val["Message"] = b64_buf;
    json_str = json_val.serialize();

    delete [] b64_buf;
}

// Loads a JSON-formatted string into the internal data structures
void JSONSerial::loadJSONStr(string &json_str) {
    parse(json, json_str.c_str());
}

// Returns the type of the message
void JSONSerial::getType(string &type_str) {
    type_str = json["Type"].get<string>();
}

// Loads a setting from the JSON string
void JSONSerial::getSetting(string &setting, string &value) {
    MBED_ASSERT(json["Type"].get<string>() == "Setting");
    setting = json["Setting"].get<string>();
    value = json["Value"].get<string>();
}

// Get the JSON object. Needed to initialize a Frame.
MbedJSONValue *JSONSerial::getJSONObj(void) {
    return &json;
}

