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
#include "QSPIFBlockDevice.h"
#include "peripherals.hpp"
#include "LittleFileSystem.h"
#include "serial_data.hpp"
#include "nv_settings.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include "MbedJSONValue.hpp"


QSPIFBlockDevice bd(MBED_CONF_APP_QSPI_FLASH_IO0, MBED_CONF_APP_QSPI_FLASH_IO1,
                    MBED_CONF_APP_QSPI_FLASH_IO2, MBED_CONF_APP_QSPI_FLASH_IO3, 
                    MBED_CONF_APP_QSPI_FLASH_SCK, MBED_CONF_APP_QSPI_FLASH_CSN,
                    0, 40000000);
LittleFileSystem fs("fs");
//extern UARTSerial gps_serial;
extern Adafruit_SSD1306_I2c *oled;


void rescue_filesystem(void) {
    int err = bd.init();
	err = fs.reformat(&bd);
    ThisThread::sleep_for(2000);
    reboot_system();
}

static void print_dir(string &base_str);
static void print_dir(string &base_str) {
    DIR *d = opendir(base_str.c_str());
	for(;;) {
		struct dirent *dir_val = readdir(d);
		if(dir_val == NULL) {
			break;
		}
        stringstream fname;
        fname << base_str << "/" << dir_val->d_name;
		struct stat file_stat;
        if(string(dir_val->d_name) != "." && string(dir_val->d_name) != "..") {
            string scrubbed_fs_name(fname.str());
            scrubbed_fs_name.erase(0, 3);
		    MBED_ASSERT(fs.stat(scrubbed_fs_name.c_str(), &file_stat) == 0);
		    debug_printf(DBG_INFO, "%s/%s, Size: %d\r\n", base_str.c_str(), dir_val->d_name, 
                            file_stat.st_size);
        }
        if(dir_val->d_type == DT_DIR) {
            if(string(dir_val->d_name) == "." || string(dir_val->d_name) == "..") {
                continue;
            }
            stringstream dir_path;
            dir_path << base_str << "/" << dir_val->d_name;
            string new_dir_path(dir_path.str());
            print_dir(new_dir_path);
        }
	}
}

void init_filesystem(void) {
    debug_printf(DBG_INFO, "Now mounting the block device\r\n");
    int err = bd.init();
    debug_printf(DBG_INFO, "bd.init -> %d  \r\n", err);
    debug_printf(DBG_INFO, "bd size: %llu\n",         bd.size());
    debug_printf(DBG_INFO, "bd read size: %llu\n",    bd.get_read_size());
    debug_printf(DBG_INFO, "bd program size: %llu\n", bd.get_program_size());
    debug_printf(DBG_INFO, "bd erase size: %llu\n",   bd.get_erase_size());
    debug_printf(DBG_INFO, "Now mounting the filesystem...\r\n");
    err = fs.mount(&bd);
    debug_printf(DBG_WARN, "%s\r\n", (err ? "Fail :(" : "OK"));
    if(err) {
        debug_printf(DBG_WARN, "No filesystem found, reformatting...\r\n");
        err = fs.reformat(&bd);
        debug_printf(DBG_WARN, "%s\r\n", (err ? "Fail :(" : "OK"));
        MBED_ASSERT(!err);
        int err = fs.mount(&bd);
        MBED_ASSERT(!err);
    }
	// Display the root directory
    debug_printf(DBG_INFO, "Opening the root directory... \r\n");
    fflush(stdout);
    string base_str("/fs");
    DIR *d = opendir(base_str.c_str());
    debug_printf(DBG_INFO, "%s\n", (!d ? "Fail :(\r\n" : "OK\r\n"));
    if (!d) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }
    print_dir(base_str);
}

extern Thread rx_serial_thread;
void load_settings_from_flash(void) {
    debug_printf(DBG_INFO, "Stats on settings.json\r\n");
    FILE *f;    
    f = fopen("/fs/settings.json", "r");
    if(!f) {
        debug_printf(DBG_WARN, "Unable to open settings.json. Creating new file with default settings\r\n");
        f = fopen("/fs/settings.json", "w");
        radio_cb["Mode"] = "Mesh Normal";
        radio_cb["Address"] = DEFAULT_ADDRESS;
        radio_cb["Frequency"] = RADIO_FREQUENCY;
        radio_cb["Frequencies"][0] = RADIO_FREQUENCY;
        radio_cb["BW"] = RADIO_BANDWIDTH;
        radio_cb["CR"] = RADIO_CODERATE;
        radio_cb["SF"] = RADIO_SF;
        radio_cb["Preamble Length"] = RADIO_PREAMBLE_LEN;
        radio_cb["Preamble Slots"] = RADIO_PREAMBLE_SLOTS;
        radio_cb["Beacon Message"] = RADIO_BEACON_MSG;
        radio_cb["Beacon Interval"] = RADIO_BEACON_INTERVAL;
        radio_cb["Payload Length"] = FRAME_PAYLOAD_LEN;
        radio_cb["FEC Algorithm"] = FEC_ALGORITHM;
        radio_cb["Conv Rate"] = FEC_CONV_RATE;
        radio_cb["Conv Order"] = FEC_CONV_ORDER;
        radio_cb["Reed-Solomon Number Roots"] = FEC_RS_NUM_ROOTS;
	    radio_cb["TX Power"] = RADIO_POWER;
        radio_cb["CW Test Mode"] = 0;
        radio_cb["Preamble Test Mode"] = 0;
        radio_cb["Test FEC"] = 0;
        radio_cb["Number Offsets"] = 0;
        radio_cb["Has GPS"] = 0;
        radio_cb["POCSAG Frequency"] = 439987500;
        radio_cb["POCSAG Beacon Interval"] = 600;
        string settings_str = radio_cb.serialize();
        debug_printf(DBG_INFO, "%s %d\r\n", settings_str.c_str(), settings_str.size());
        fwrite(settings_str.c_str(), 1, settings_str.size(), f);   
        fflush(f);
        fclose(f); 
        f = fopen("/fs/settings.json", "r");
        MBED_ASSERT(f);
    }
    struct stat file_stat;
    stat("/fs/settings.json", &file_stat);
    debug_printf(DBG_INFO, "Size is %d\r\n", file_stat.st_size);
	vector<char> linebuf(file_stat.st_size);
	fread(linebuf.data(), 1, file_stat.st_size, f);
	debug_printf(DBG_INFO, "Read from file: %s\r\n", linebuf.data());
    ThisThread::sleep_for(1000);
    MbedJSONValue tmp_json;
    parse(tmp_json, linebuf.data());
    radio_cb = tmp_json;
    fflush(f);
    fclose(f);
    debug_printf(DBG_INFO, "Mode: %s\r\n", radio_cb["Mode"].get<string>().c_str());
    debug_printf(DBG_INFO, "Address: %d\r\n", radio_cb["Address"].get<int>());
    for(int i = 0; i < radio_cb["Frequencies"].size(); i++) {
        debug_printf(DBG_INFO, "Frequency %d: %d\r\n", i, radio_cb["Frequencies"][i].get<int>());        
    }
    debug_printf(DBG_INFO, "BW: %d\r\n", radio_cb["BW"].get<int>());
    debug_printf(DBG_INFO, "CR: %d\r\n", radio_cb["CR"].get<int>());
    debug_printf(DBG_INFO, "SF: %d\r\n", radio_cb["SF"].get<int>());
    debug_printf(DBG_INFO, "Preamble Length: %d\r\n", radio_cb["Preamble Length"].get<int>());
    debug_printf(DBG_INFO, "Preamble Slots: %d\r\n", radio_cb["Preamble Slots"].get<int>());
    debug_printf(DBG_INFO, "Beacon Message: %s\r\n", radio_cb["Beacon Message"].get<string>().c_str());
    debug_printf(DBG_INFO, "Beacon Interval: %d\r\n", radio_cb["Beacon Interval"].get<int>());
    debug_printf(DBG_INFO, "Payload Length: %d\r\n", radio_cb["Payload Length"].get<int>());
    debug_printf(DBG_INFO, "Number of timing offset increments: %d\r\n", 
                radio_cb["Number Offsets"].get<int>());
    debug_printf(DBG_INFO, "Has a GPS: %d\r\n", radio_cb["Has GPS"].get<int>());
    debug_printf(DBG_INFO, "POCSAG frequency %d\r\n", radio_cb["POCSAG Frequency"].get<int>());
    debug_printf(DBG_INFO, "POCSAG Beacon Interval %d\r\n", radio_cb["POCSAG Beacon Interval"].get<int>());
    // Since really only 1/2 rate, constraint length=7 convolutional code works, we want to block 
    //  anything else from occurring and leading to weird crashes
    MBED_ASSERT(radio_cb["Conv Rate"].get<int>() == 2);
    MBED_ASSERT(radio_cb["Conv Order"].get<int>() == 7);
    // Check if low-power mode is set. If so, delete the UART
    rx_serial_thread.start(rx_serial_thread_fn);
    FILE *low_power_fh = fopen("/fs/low_power.mode", "r");
    if(low_power_fh) {
        rx_serial_thread.terminate();
        oled->displayOff();
        mbed_file_handle(STDIN_FILENO)->enable_input(false); 
        //gps_serial.enable_input(false); 
        fclose(low_power_fh);
    }
}

void save_settings_to_flash(void) {
    debug_printf(DBG_INFO, "Opening settings.json...\r\n");
    FILE *f = fopen("/fs/settings.json", "w"); 
    MBED_ASSERT(f != 0);
    string settings_str = radio_cb.serialize();
    fwrite(settings_str.c_str(), 1, settings_str.size(), f);
    fclose(f);  
}

void log_boot(void) {
    FILE *f = fopen("/fs/boot_log.json", "a+");
    MBED_ASSERT(f);
    MbedJSONValue log_json;
    time_t my_time = time(NULL);
    char *time_str = ctime(&my_time);
    log_json["Time String"] = string(time_str);
    string log_json_str = log_json.serialize();
	log_json_str.push_back('\n');
	debug_printf(DBG_INFO, "Wrote %s\r\n", log_json_str.c_str());
    fwrite(log_json_str.c_str(), 1, log_json_str.size(), f);
    fflush(f);
	fclose(f);
}


FILE *open_logfile(void) {
    // Step one: get the size of the current logfile. If current logfile is too big,
    //  move it down the "logfile stack".
    debug_printf(DBG_INFO, "opening the logfile\r\n");
    FILE *f = fopen("/fs/log/logfile.json", "r");
    if(!f) {
        debug_printf(DBG_INFO, "Need to create the logfile\r\n");
        f = fopen("/fs/log/logfile.json", "w");
        MBED_ASSERT(f);
    }
    fclose(f);
    struct stat logfile_statbuf;
    fs.stat("log/logfile.json", &logfile_statbuf);
    debug_printf(DBG_INFO, "logfile size is %d\r\n", logfile_statbuf.st_size);
    if(logfile_statbuf.st_size > LOGFILE_SIZE) {
        for(int i = 11; i >= 0; i--) {
            stringstream logfile_name, logfile_name_plusone;
            logfile_name << "log/logfile" << setw(3) << setfill('0') << i << ".json";
            debug_printf(DBG_INFO, "Now moving %s\r\n", logfile_name.str().c_str());
            logfile_name_plusone << "log/logfile" << setw(3) << setfill('0') << i+1 << ".json";
            fs.rename(logfile_name.str().c_str(), logfile_name_plusone.str().c_str());
        }
        fs.rename("log/logfile.json", "log/logfile000.json");
    }
    f = fopen("/fs/log/logfile.json", "a+");
    MBED_ASSERT(f != NULL);
    return f;
}


extern time_t boot_timestamp;
void nv_log_fn(void) {
    DIR *log_dir = opendir("/fs/log");
    if(!log_dir && errno == ENOENT) {
        debug_printf(DBG_INFO, "Log directory does not exist. Creating...\r\n");
        if(mkdir("/fs/log", 777)) {
            MBED_ASSERT(false);
        }
        log_dir = opendir("/fs/log");
        if(!log_dir) {
            switch(errno) {
                case EACCES:  debug_printf(DBG_INFO, "EACCES\r\n"); break;
                case EBADF:   debug_printf(DBG_INFO, "EBADF\r\n"); break;
                case EMFILE:  debug_printf(DBG_INFO, "EMFILE\r\n"); break;
                case ENFILE:  debug_printf(DBG_INFO, "ENFILE\r\n"); break;
                case ENOENT:  debug_printf(DBG_INFO, "ENOENT\r\n"); break;
                case ENOMEM:  debug_printf(DBG_INFO, "ENOMEM\r\n"); break;
                case ENOTDIR: debug_printf(DBG_INFO, "ENOTDIR\r\n"); break;
                default: break;
            }
            MBED_ASSERT(false);
        }
    }
    debug_printf(DBG_INFO, "Now opening the logfile\r\n");
    debug_printf(DBG_INFO, "First set\r\n");
    FILE *f = open_logfile();
    string comb_str;
    for(;;) {
        // Write the latest frame to disk
        auto log_frame = dequeue_mail(nv_logger_mail);  
        MbedJSONValue log_json;
        int16_t rssi;
        uint16_t rx_size;
        int8_t snr;
        log_frame->getRxStats(rssi, snr, rx_size);
		time_t my_time = time(NULL);
		log_json["Timestamp"] = ctime(&my_time);
        log_json["Sender"] = (int) log_frame->getSender();
        log_json["TTL"] = (int) log_frame->getTTL();
        log_json["Stream ID"] = (int) log_frame->getStreamID();
        log_json["RSSI"] = (int) rssi;
        log_json["SNR"] = (int) snr;
        log_json["RX Size"] = (int) rx_size;
        log_json["Computed CRC"] = log_frame->calcCRC();
        log_json["CRC"] = log_frame->getCRC();
        float gps_lat, gps_lon;
        bool gps_valid = gpsd_get_coordinates(gps_lat, gps_lon);
        log_json["GPS Valid"] = gps_valid;
        log_json["GPS Lat"] = to_string(gps_lat);
        log_json["GPS Lon"] = to_string(gps_lon);
        time_t cur_time;
        time(&cur_time);
        time_t uptime = cur_time - boot_timestamp;
        stringstream msg;
        int uptime_rem = uptime;
        int uptime_d = uptime_rem / 86400;
        uptime_rem %= 86400;
        int uptime_h = uptime_rem / 3600;
        uptime_rem %= 3600;
        int uptime_m = uptime_rem / 60;
        uptime_rem %= 60;
        int uptime_s = uptime_rem;
        msg << uptime_d << "d:" << uptime_h << \
            "h:" << uptime_m << "m:" << uptime_s << "s";
        log_json["Uptime"] = msg.str();
        string log_json_str = log_json.serialize();
		log_json_str.push_back('\n');
        comb_str.append(log_json_str);
        fwrite(comb_str.c_str(), 1, comb_str.size(), f);
        comb_str.clear();
        fclose(f);
        f = open_logfile();
    }
}