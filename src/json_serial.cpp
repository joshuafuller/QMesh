/*
QMesh
Copyright (C) 2019 Daniel R. Fay

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

extern RawSerial pc, pc2;

void print_memory_info();
void tx_serial_thread_fn(void) {
    for(;;) {
        //print_memory_info();
        auto str_sptr = dequeue_mail<std::shared_ptr<string>>(tx_ser_queue);
        str_sptr->push_back('\r');
        str_sptr->push_back('\n');
        for(int i = 0; i < str_sptr->length(); i++) {
            while(!pc.writeable());
            pc.putc(str_sptr->c_str()[i]);
        }
    }
}

/**
 * Sends the current status.
 */
void send_status(void);
void send_status(void) {
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
    status_json["Time"] = to_string(time(NULL));
    auto json_str = make_shared<string>(status_json.serialize());
    enqueue_mail<std::shared_ptr<string>>(tx_ser_queue, json_str);       
}  

static void ser_rx_isr(void);
static char rx_bufs[4][2048];
static Mail<int, 16> rx_data_rdy;
static bool led2_val, led3_val = false;
static int my_buf = 0;
static int my_buf_idx = 0;
static void ser_rx_isr(void) {
    if(led2_val) {
        led2.LEDSolid();
        led2_val = false;
    }
    else {
        led2.LEDOff();
        led2_val = true;
    }
    while(pc2.readable()) {
        char my_char = pc2.getc();
        rx_bufs[my_buf][my_buf_idx++] = my_char;
        if(my_char == '\n' || my_char == '\r') {
            if(led3_val) {
                led3.LEDSolid();
                led3_val = false;
            }
            else {
                led3.LEDOff();
                led3_val = true;
            }
            rx_bufs[my_buf][my_buf_idx++] = '\0';
            enqueue_mail_nonblocking<int>(rx_data_rdy, my_buf);
            my_buf += 1;
            my_buf_idx = 0;
            my_buf %= 4;
        }
    } 
}


void rx_serial_thread_fn(void) {
    pc2.attach(ser_rx_isr, mbed::SerialBase::RxIrq);
    for(;;) {
        int rx_buf_idx = dequeue_mail<int>(rx_data_rdy);
        string rx_str(rx_bufs[rx_buf_idx]);
        debug_printf(DBG_INFO, "Received a string: %s\r\n", rx_str.c_str());
        MbedJSONValue rx_json;
        parse(rx_json, rx_str.c_str());
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
            else if(setting == "FEC Algorithm") {
                 radio_cb["FEC Algorithm"] = rx_json["FEC Algorithm"].get<string>();               
            }
            else {
                radio_cb[setting] = rx_json[setting].get<int>();
            }
            save_settings_to_flash();
			send_status();
        }
        else if(type_str == "Get Status") {
            ThisThread::sleep_for(100);
            send_status();      
        }
        else if(type_str == "Set Time") {
            string new_time = rx_json["Time"].get<string>();
            set_time((unsigned int) stoul(new_time));
            ThisThread::sleep_for(100);
            send_status();
        }
        else if(type_str == "Stay in Management") {
            stay_in_management = true;
            ThisThread::sleep_for(100);            
            send_status();
        }
        else if(type_str == "Debug Msg") {
            //MBED_ASSERT(false);
        }
        else if(type_str == "Send Frame") {
            auto frame = make_shared<Frame>();
            frame->loadFromJSON(rx_json);
            enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, frame);
            send_status();           
        }
        else if(type_str == "Reboot") {
            send_status();
            reboot_system();
        }
        else if(type_str == "Erase Log File") {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("logfile.json");
            }
            ThisThread::sleep_for(100);
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            ThisThread::sleep_for(1000);
            reboot_system();
        }
        else if(type_str == "Erase Cfg File") {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("settings.json");
            }
            ThisThread::sleep_for(100);
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            ThisThread::sleep_for(1000);
            reboot_system();
        }
        else if(type_str == "Read Log") {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                std::fstream f;
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
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            ThisThread::sleep_for(1000);
            reboot_system();
        }
        else {
            continue;
        }
    }
}

void JSONSerial::settingsToJSON(MbedJSONValue &my_nv_settings, string &json_str) {
    json["Type"] = "Settings";
    json["Frequency"] = my_nv_settings["Frequency"].get<int>();
    json["SF"] = my_nv_settings["SF"].get<int>();
    json["BW"] = my_nv_settings["BW"].get<int>();
    json["CR"] = my_nv_settings["CR"].get<int>();
    json["Mode"] = my_nv_settings["Mode"].get<string>();
    json_str = json.serialize();
}

void JSONSerial::statusToJSON(string &status, string &value, string &json_str) {
    json["Type"] = "Status";
    json["Status"] = status;
    json["Value"] = value;
    json_str = json.serialize();
}

void JSONSerial::dbgPrintfToJSON(string &dbg_msg, string &json_str) {
    json["Type"] = "Debug Msg";
    json["Timestamp"] = 0;
    size_t b64_len;
    mbedtls_base64_encode(NULL, 0, &b64_len, (unsigned char *) dbg_msg.c_str(), dbg_msg.size());
    vector<unsigned char> b64_buf(b64_len);
    MBED_ASSERT(mbedtls_base64_encode(b64_buf.data(), b64_len, &b64_len, 
            (unsigned char *) dbg_msg.c_str(), dbg_msg.size()) == 0);  
    json["Message"] = (char *) b64_buf.data();
    json_str = json.serialize();
}

void JSONSerial::loadJSONStr(string &json_str) {
    parse(json, json_str.c_str());
}

void JSONSerial::getType(string &type_str) {
    type_str = json["Type"].get<string>();
}

#warning Needs more work, need to add more settings
void JSONSerial::getSettings(MbedJSONValue &nv_setting) {
    nv_setting["Freq"] = json["Freq"].get<int>();
    nv_setting["SF"] = json["SF"].get<int>();
    nv_setting["BW"] = json["BW"].get<int>();
    nv_setting["CR"] = json["CR"].get<int>();
    nv_setting["Mode"] = json["Mode"].get<string>();
}


