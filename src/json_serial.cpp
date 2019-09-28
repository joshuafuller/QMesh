/*
 * Copyright (c) 2019, Daniel R. Fay.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "json_serial.hpp"
#include "mbed.h"
#include "MbedJSONValue.h"
#include <string>
#include <memory>
#include <fstream>
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include "mesh_protocol.hpp"


Mail<std::shared_ptr<string>, 16> tx_ser_queue;
void tx_serial_thread_fn(void) {
    for(;;) {
        auto str_sptr = dequeue_mail<std::shared_ptr<string>>(tx_ser_queue);
        printf("%s\r\n", str_sptr->c_str());
    }
}

static char rx_str[2048];
#if 0
void rx_serial_thread_fn(void) {
    for(;;) {
        if(scanf("%s\r\n", rx_str) != 0) {
            debug_printf(DBG_WARN, "scanf() in Rx thread returned with error %d\r\n");
            continue;
        }
        string rx_string(rx_str);
        JSONSerial json_ser;
        json_ser.loadJSONStr(rx_string);
        string type_string;
        json_ser.getType(type_string);
        if(type_string == "Get Settings") {
            JSONSerial tx_json_ser;
            auto json_str = make_shared<string>();
            tx_json_ser.settingsToJSON(radio_cb, *json_str);
            enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);
        }
        else if(type_string == "Put Settings") {
            string json_str;
            json_ser.getSettings(radio_cb);
        }
        else if(type_string == "Status") {

        }
        else if(type_string == "Debug Msg") {
            MBED_ASSERT(false);
        }
        else if(type_string == "Frame") {
            Frame frame;
            frame.loadFromJSON(*(json_ser.getJSONObj()));
        }
        else {
            MBED_ASSERT(false);
        }
    }
}
#else
static void send_status(void);
static void send_status(void) {
    MbedJSONValue status_json;
    status_json["Type"] = "Status";
    if(current_mode == BOOTING) {
        status_json["Status"] = "BOOTING";        
    }
    else if(current_mode == MANAGEMENT) {
        status_json["Status"] = "MANAGEMENT";        
    }
    else if(current_mode == RUNNING) {
        status_json["Status"] = "RUNNING";  
    }
    else {
        MBED_ASSERT(false);
    }
    if(tx_frame_mail.full()) {
        status_json["Tx Frame Queue Full"] = "True";
    }
    else {
        status_json["Tx Frame Queue Full"] = "False";
    }
    auto json_str = make_shared<string>(status_json.serialize());
    enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);       
}  

void rx_serial_thread_fn(void) {
    for(;;) {
        if(scanf("%s\r\n", rx_str) != 0) {
            debug_printf(DBG_WARN, "scanf() in Rx thread returned with error %d\r\n");
            continue;
        }
        MbedJSONValue rx_json;
        parse(rx_json, rx_str);
        string type_str(rx_json["Type"].get<string>()); 
        if(type_str == "Get Settings") {
            MbedJSONValue settings_json = radio_cb;
            settings_json["Type"] = "Settings";
            auto json_str = make_shared<string>(settings_json.serialize());
            enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);
        }
        else if(type_str == "Put Setting") {
            string setting = rx_json["Setting"].get<string>();
            if(setting == "Mode") {
                radio_cb["Mode"] = rx_json["Mode"].get<string>();
            }
            else {
                radio_cb[setting] = rx_json[setting].get<int>();
            }
            MbedJSONValue settings_json = radio_cb;
            save_settings_to_flash();
            settings_json["Type"] = "Settings";
            auto json_str = make_shared<string>(settings_json.serialize());
            enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);
        }
        else if(type_str == "Get Status") {
            send_status();      
        }
        else if(type_str == "Debug Msg") {
            MBED_ASSERT(false);
        }
        else if(type_str == "Send Frame") {
            auto frame = make_shared<Frame>();
            frame->loadFromJSON(rx_json);
            enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, frame);
            send_status();           
        }
        else if(type_str == "Reboot") {
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(type_str == "Erase Log") {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("/fs/logfile.json");
            }
            send_status();
            reboot_system();
        }
        else if(type_str == "Read Log") {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fstream f;
                f.open("/fs/logfile.json", ios_base::in);
                string line;
                while(getline(f, line)) {
                    MbedJSONValue log_json;
                    parse(log_json, line.c_str());
                    log_json["Type"] = "Log Entry";
                    auto json_str = make_shared<string>(log_json.serialize());
                    enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);
                }  
            }
            send_status();
            reboot_system();
        }
        else {
            MBED_ASSERT(false);
        }
    }
}
#endif


// Creates a JSON-formatted string for a given setting
void JSONSerial::settingsToJSON(MbedJSONValue &my_nv_settings, string &json_str) {
    json["Type"] = "Settings";
    json["Freq"] = my_nv_settings["Freq"].get<int>();
    json["SF"] = my_nv_settings["SF"].get<int>();
    json["BW"] = my_nv_settings["BW"].get<int>();
    json["CR"] = my_nv_settings["CR"].get<int>();
    json["Mode"] = my_nv_settings["Mode"].get<string>();
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
    json["Timestamp"] = (int) time(NULL);
    size_t b64_len;
    mbedtls_base64_encode(NULL, 0, &b64_len, (unsigned char *) dbg_msg.c_str(), dbg_msg.size());
    vector<unsigned char> b64_buf(b64_len);
    MBED_ASSERT(mbedtls_base64_encode(b64_buf.data(), b64_len, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);  
    json["Message"] = (char *) b64_buf.data();
    json_str = json.serialize();
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
void JSONSerial::getSettings(MbedJSONValue &nv_setting) {
    nv_setting["Freq"] = json["Freq"].get<int>();
    nv_setting["SF"] = json["SF"].get<int>();
    nv_setting["BW"] = json["BW"].get<int>();
    nv_setting["CR"] = json["CR"].get<int>();
    nv_setting["Mode"] = json["Mode"].get<string>();
}

// Get the JSON object. Needed to initialize a Frame.
MbedJSONValue *JSONSerial::getJSONObj(void) {
    return &json;
}

