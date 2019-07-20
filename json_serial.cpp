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
        if(scanf("%s\r\n", rx_str) != 0) {
            debug_printf(DBG_WARN, "scanf() in Rx thread returned with error %d\r\n");
            continue;
        }
        string *rx_string = new string(rx_str);
        JSONSerial *json_ser = new JSONSerial();
        json_ser->loadJSONStr(*rx_string);
        string *type_string = new string();
        json_ser->getType(*type_string);
        if(*type_string == "Get Settings") {
            nv_settings_t nv_settings_struct = nv_settings->getNVSettings();
            JSONSerial *tx_json_ser = new JSONSerial();
            string *json_str = new string();
            tx_json_ser->settingsToJSON(nv_settings_struct, *json_str);
            tx_ser_queue.put(json_str);
            delete tx_json_ser;
        }
        else if(*type_string == "Put Settings") {
            string *json_str = new string();
            nv_settings_t nv_settings_struct;
            json_ser->getSettings(nv_settings_struct);
            nv_settings->putNVSettings(nv_settings_struct);
        }
        else if(*type_string == "Status") {

        }
        else if(*type_string == "Debug Msg") {
            MBED_ASSERT(false);
        }
        else if(*type_string == "Frame") {
            Frame *frame = new Frame();
            frame->loadFromJSON(*(json_ser->getJSONObj()));
        }
        else {
            MBED_ASSERT(false);
        }

        delete json_ser;
        delete type_string;
        delete rx_string;
    }
}


// Creates a JSON-formatted string for a given setting
void JSONSerial::settingsToJSON(nv_settings_t &nv_settings, string &json_str) {
    json["Type"] = "Settings";
    json["Freq"] = (int) nv_settings.freq;
    json["SF"] = (int) nv_settings.sf;
    json["BW"] = (int) nv_settings.bw;
    json["CR"] = (int) nv_settings.cr;
    if(nv_settings.mode == MESH_MODE_NORMAL) {
        json["Mode"] = "MESH_MODE_NORMAL";
    }
    else if(nv_settings.mode == MESH_MODE_BEACON) {
        json["Mode"] = "MESH_MODE_BEACON";
    }
    else {
        MBED_ASSERT(false);
    }
    json_str = json.serialize();
}

// Creates a JSON-formatted string for the current status
void JSONSerial::statusToJSON(string &status, string &value, string &json_str) {
    json["Type"] = "Status";
    json["Status"] = status;
    json["Value"] = value;
    json_str = json.serialize();
}

// Creates a JSON-formatted string for a debug printf, with the message being
//  encoded as Base64
void JSONSerial::dbgPrintfToJSON(string &dbg_msg, string &json_str) {
    json["Type"] = "Debug Msg";
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);
    json["Message"] = b64_buf;
    json_str = json.serialize();

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
void JSONSerial::getSettings(nv_settings_t &nv_setting) {
    nv_setting.freq = json["Freq"].get<int>();
    nv_setting.sf = json["SF"].get<int>();
    nv_setting.bw = json["BW"].get<int>();
    nv_setting.cr = json["CR"].get<int>();
    string mode = json["Mode"].get<string>();
    if(mode == "MESH_MODE_NORMAL") {
        nv_setting.mode = MESH_MODE_NORMAL;
    }
    else if(mode == "MESH_MODE_BEACON") {
        nv_setting.mode = MESH_MODE_BEACON;
    }
    else {
        MBED_ASSERT(false);
    }
}

// Get the JSON object. Needed to initialize a Frame.
MbedJSONValue *JSONSerial::getJSONObj(void) {
    return &json;
}

