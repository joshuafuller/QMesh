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

#include "mbed.h"
#include "peripherals.hpp"
#include "FlashIAPBlockDevice.h"
#include "LittleFileSystem.h"
#include "serial_data.hpp"
#include "nv_settings.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "MbedJSONValue.h"


//BlockDevice *bd = new FlashIAPBlockDevice();
HeapBlockDevice bd(16384, 512);
LittleFileSystem fs("fs");

void init_filesystem(void) {
    debug_printf(DBG_INFO, "Now mounting the block device\r\n");
    bd.init();
    fs.reformat(&bd);
    debug_printf(DBG_INFO, "Now mounting the filesystem...\r\n");
    int err = fs.mount(&bd);
    debug_printf(DBG_WARN, "%s\r\n", (err ? "Fail :(" : "OK"));
    if(err) {
        debug_printf(DBG_WARN, "No filesystem found, reformatting...\r\n");
        err = fs.reformat(&bd);
        debug_printf(DBG_WARN, "%s\r\n", (err ? "Fail :(" : "OK"));
        MBED_ASSERT(!err);
    }
}

void load_settings_from_flash(void) {
    debug_printf(DBG_INFO, "Opening settings.json...\r\n");
    fstream f;
    f.open("settings.json", ios_base::in);
    if(!f.is_open()) {
        debug_printf(DBG_WARN, "Unable to open settings.json. Creating new file with default settings\r\n");
        f.open("settings.json", ios_base::out);
        radio_cb.mode = MESH_MODE_NORMAL;
        radio_cb.freq = RADIO_FREQUENCY;
        radio_cb.bw = RADIO_BANDWIDTH;
        radio_cb.cr = RADIO_CODERATE;
        radio_cb.sf = RADIO_SF;
        MbedJSONValue radio_settings;
        if(radio_cb.mode == MESH_MODE_NORMAL) {
            radio_settings["Mode"] = "Mesh Normal";
        }
        else if(radio_cb.mode == MESH_MODE_BEACON) {
            radio_settings["Mode"] = "Mesh Beacon";
        }
        else {
            MBED_ASSERT(false);
        }
        radio_settings["Frequency"] = (int) radio_cb.freq;
        radio_settings["Bandwidth"] = (int) radio_cb.bw;
        radio_settings["Coding Rate"] = (int) radio_cb.cr;
        radio_settings["Spreading Factor"] = (int) radio_cb.sf;
        radio_settings["Preamble Length"] = (int) radio_cb.pre_len;

        string settings_str = radio_settings.serialize();
        f.write(settings_str.c_str(), settings_str.size());
        f.close();
    }
    // Settings file exists, so read it
    else {
        string settings_str;
        string line;
        while(std::getline(f, line)) {
            settings_str.append(line);
        }
        MbedJSONValue radio_settings(settings_str);
        radio_cb.freq = radio_settings["Frequency"].get<int>();
        radio_cb.bw = radio_settings["Bandwidth"].get<int>();
        radio_cb.cr = radio_settings["Coding Rate"].get<int>();
        radio_cb.sf = radio_settings["Spreading Factor"].get<int>();
        radio_cb.pre_len = radio_settings["Preamble Length"].get<int>();
    }
    f.close();
}

void saveSettingsToFlash(void) {
    debug_printf(DBG_INFO, "Opening settings.json...\r\n");
    fstream f;
    f.open("settings.json", ios_base::out); 
    MBED_ASSERT(f.is_open());
    MbedJSONValue radio_settings;
    if(radio_cb.mode == MESH_MODE_NORMAL) {
        radio_settings["Mode"] = "Mesh Normal";
    }
    else if(radio_cb.mode == MESH_MODE_BEACON) {
        radio_settings["Mode"] = "Mesh Beacon";
    }
    else {
        MBED_ASSERT(false);
    }
    radio_settings["Frequency"] = (int) radio_cb.freq;
    radio_settings["Bandwidth"] = (int) radio_cb.bw;
    radio_settings["Coding Rate"] = (int) radio_cb.cr;
    radio_settings["Spreading Factor"] = (int) radio_cb.sf;
    radio_settings["Preamble Length"] = (int) radio_cb.pre_len;

    string settings_str = radio_settings.serialize();
    f.write(settings_str.c_str(), settings_str.size());
    f.close();
}

void nv_log_fn(void) {
    uint16_t session_nonce = rand() % 65536;
    stringstream file_name;
    file_name << "session_" << session_nonce << ".log";
    fstream f;
    f.open(file_name.str(), ios_base::out);
    MBED_ASSERT(f.is_open());
    for(;;) {        
        auto log_frame = dequeue_mail(nv_logger_mail);  
        MbedJSONValue log_json;
        int16_t rssi;
        uint16_t rx_size;
        int8_t snr;
        log_frame->getRxStats(&rssi, &snr, &rx_size);
        log_json["RSSI"] = (int) rssi;
        log_json["SNR"] = (int) snr;
        log_json["RX Size"] = (int) rx_size;
        log_json["HDR Computed CRC"] = log_frame->calculateHeaderCrc();
        log_json["HDR CRC"] = log_frame->getHeaderCrc();
        log_json["PLD Computed CRC"] = log_frame->calculatePayloadCrc();
        log_json["PLD CRC"] = log_frame->getPayloadCrc();
        string log_json_str = log_json.serialize();
        f.write(log_json_str.c_str(), log_json_str.size());
        f.flush();
    }
}