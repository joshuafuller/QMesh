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

#include "mbed.h"
#include "peripherals.hpp"
#include "LittleFileSystem.h"
#include "serial_data.hpp"
#include "nv_settings.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "MbedJSONValue.h"

DigitalOut flash_io2(MBED_CONF_APP_FLASH_IO2);
DigitalOut flash_io3(MBED_CONF_APP_FLASH_IO3);
#ifdef HEAP_FS
HeapBlockDevice bd(16384, 512);
#else
SPIFBlockDevice bd(MBED_CONF_APP_FLASH_SPI_MOSI, 
                    MBED_CONF_APP_FLASH_SPI_MISO, 
                    MBED_CONF_APP_FLASH_SPI_SCLK, 
                    MBED_CONF_APP_FLASH_SPI_CS);
#endif
LittleFileSystem fs("fs");


void init_filesystem(void) {
    flash_io2 = 0;
    flash_io3 = 1;
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
        int err = fs.mount(&bd);
        MBED_ASSERT(!err);
    }
}

void load_settings_from_flash(void) {
    debug_printf(DBG_INFO, "Opening settings.json...\r\n");
    std::fstream f;
    f.open("/fs/settings.json", 'r');
    if(!f.is_open()) {
        debug_printf(DBG_WARN, "Unable to open settings.json. Creating new file with default settings\r\n");
        f.open("/fs/settings.json", 'w');
        radio_cb["Mode"] = "Mesh Normal";
        radio_cb["Frequency"] = RADIO_FREQUENCY;
        radio_cb["BW"] = RADIO_BANDWIDTH;
        radio_cb["CR"] = RADIO_CODERATE;
        radio_cb["SF"] = RADIO_SF;
        radio_cb["Preamble Length"] = RADIO_PREAMBLE_LEN;
        radio_cb["Preamble Slots"] = RADIO_PREAMBLE_SLOTS;
        radio_cb["Beacon Message"] = RADIO_BEACON_MSG;
        radio_cb["Beacon Interval"] = RADIO_BEACON_INTERVAL;
        radio_cb["Payload Length"] = FRAME_PAYLOAD_LEN;
        string settings_str = radio_cb.serialize();
        f.write(settings_str.c_str(), settings_str.size());
        f.flush();
        f.close();
        f.open("/fs/settings.json", 'r');
    }
    // Settings file exists, so read it
    else {
        string settings_str;
        string line;
        while(std::getline(f, line)) {
            settings_str.append(line);
        }
        parse(radio_cb, settings_str.c_str());
    }
    f.close();
}

void save_settings_to_flash(void) {
    debug_printf(DBG_INFO, "Opening settings.json...\r\n");
    std::fstream f;
    f.open("/fs/settings.json", ios_base::out); 
    MBED_ASSERT(f.is_open());
    string settings_str = radio_cb.serialize();
    f.write(settings_str.c_str(), settings_str.size());
    f.close();
}

void nv_log_fn(void) {
    uint16_t session_nonce = rand() % 65536;
    std::fstream f;
    f.open("/fs/logfile.json", ios_base::app);
    MBED_ASSERT(f.is_open());
    for(;;) {
        // Write the latest frame to disk
        auto log_frame = dequeue_mail(nv_logger_mail);  
        MbedJSONValue log_json;
        int16_t rssi;
        uint16_t rx_size;
        int8_t snr;
        log_frame->getRxStats(&rssi, &snr, &rx_size);
        log_json["Timestamp"] = (int) time(NULL);
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
        // Check whether we've filled up the SPI flash chip. If so,
        //  delete the file and reopen it as an empty one.
        struct stat st;
        stat("/fs/logfile.json", &st);
        if(st.st_size > 12000000) {
            debug_printf(DBG_INFO, "Log file is >12MB. Deleting and reopening...\r\n");
            f.close();
            f.open("/fs/logfile.json", ios_base::out);
            MBED_ASSERT(f.is_open());
        }
    }
}