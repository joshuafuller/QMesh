/*
QMesh
Copyright (C) 2022 Daniel R. Fay

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

#include "os_portability.hpp"
#include "peripherals.hpp"
#include "serial_data.hpp"
#include "nv_settings.hpp"
#include "voice_msg.hpp"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <exception>
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial_msg.hpp"
#include "pseudo_serial.hpp"
#include "Adafruit_SSD1306.h"
#include <lwip/inet.h>


static constexpr int SIX_SECONDS = 6;
static constexpr int TWO_SEC = 2000;
static constexpr int FREQ_40_MHZ = 40000000;
static constexpr int NUM_LOGFILES = 11;
static constexpr int MAX_LOGFILE_SIZE = 1e6;

portability::BlockDevice *bd;
portability::FileSystem *fs;

void create_nv_settings_objects() {
    bd = new portability::BlockDevice(QSPI_FLASH_IO0, QSPI_FLASH_IO1, QSPI_FLASH_IO2, QSPI_FLASH_IO3, 
                        QSPI_FLASH_SCK, QSPI_FLASH_CSN, 0, FREQ_40_MHZ);
    fs = new portability::FileSystem("fs");
}

//extern UARTSerial gps_serial;
extern shared_ptr<Adafruit_SSD1306_I2c> oled;

class ParamException : public exception {
private:
    string except_name;
public:
    explicit ParamException(const string &my_name) {
        except_name = my_name;
    }
     auto what() const noexcept -> const char* override {
        return except_name.c_str();
    }
};


void rescue_filesystem() {
    bd->init();
	fs->reformat(bd);
    portability::sleep(TWO_SEC);
    reboot_system();
}


static void print_dir(const string &base_str);
static void print_dir(const string &base_str) {
    DIR *d = opendir(base_str.c_str());
	for(;;) {
		struct dirent *dir_val = readdir(d);
		if(dir_val == nullptr) {
			break;
		}
        stringstream fname;
        fname << base_str << "/" << dir_val->d_name;
		struct stat file_stat{};
        if(string(dir_val->d_name) != "." && string(dir_val->d_name) != "..") {
            string scrubbed_fs_name(fname.str());
            scrubbed_fs_name.erase(0, 3);
		    PORTABLE_ASSERT(fs->stat(scrubbed_fs_name.c_str(), &file_stat) == 0);
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

void init_filesystem() {
    debug_printf(DBG_INFO, "Now mounting the block device\r\n");
    PORTABLE_ASSERT(bd);
    int err = bd->init();
    debug_printf(DBG_INFO, "bd.init -> %d  \r\n", err);
    debug_printf(DBG_INFO, "bd size: %llu\n",         bd->size());
    debug_printf(DBG_INFO, "bd read size: %llu\n",    bd->get_read_size());
    debug_printf(DBG_INFO, "bd program size: %llu\n", bd->get_program_size());
    debug_printf(DBG_INFO, "bd erase size: %llu\n",   bd->get_erase_size());
    debug_printf(DBG_INFO, "Now mounting the filesystem...\r\n");
    PORTABLE_ASSERT(fs);
    err = fs->mount(bd);
    debug_printf(DBG_WARN, "%s\r\n", (err != 0 ? "Fail :(" : "OK"));
    if(err != 0) {
        debug_printf(DBG_WARN, "No filesystem found, reformatting...\r\n");
        err = fs->reformat(bd);
        debug_printf(DBG_WARN, "%s\r\n", (err != 0 ? "Fail :(" : "OK"));
        PORTABLE_ASSERT(!err);
        int err = fs->mount(bd);
        PORTABLE_ASSERT(!err);
    }
	// Display the root directory
    debug_printf(DBG_INFO, "Opening the root directory... \r\n");
    fflush(stdout);
    string base_str("/fs");
    DIR *d = opendir(base_str.c_str());
    debug_printf(DBG_INFO, "%s\n", (d == nullptr ? "Fail :(\r\n" : "OK\r\n"));
    if (d == nullptr) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }
    print_dir(base_str);
    // Remove the boot failure file
    fs->remove("boot.fail");
}


static void write_default_cfg();
static void write_default_cfg() {
    auto *f = fopen("/fs/settings.bin", "we");
    PORTABLE_ASSERT(f);
    SysCfgMsg sys_cfg_msg_zero = SysCfgMsg_init_zero;
    radio_cb = sys_cfg_msg_zero;
    radio_cb.mode = SysCfgMsg_Mode_NORMAL;
    radio_cb.address = DEFAULT_ADDRESS;

    radio_cb.has_radio_cfg = true;
    RadioCfg radio_cfg_zero = RadioCfg_init_zero;
    radio_cb.radio_cfg = radio_cfg_zero;
    radio_cb.radio_cfg.type = RadioCfg_Type_LORA;
    //radio_cb.radio_cfg.frequency = RADIO_FREQUENCY;
    radio_cb.radio_cfg.frequencies_count = 1;
    radio_cb.radio_cfg.frequencies[0] = RADIO_FREQUENCY;
    radio_cb.radio_cfg.tx_power = RADIO_POWER;
    constexpr float TWO_MS_US = 2000.F;
    radio_cb.radio_cfg.tcxo_time_us = TWO_MS_US;

    radio_cb.radio_cfg.has_lora_cfg = true;
    LoraCfg lora_cfg_zero = LoraCfg_init_zero;
    radio_cb.radio_cfg.lora_cfg = lora_cfg_zero;
    radio_cb.radio_cfg.lora_cfg.bw = RADIO_BANDWIDTH;
    radio_cb.radio_cfg.lora_cfg.cr = RADIO_CODERATE;
    radio_cb.radio_cfg.lora_cfg.sf = RADIO_SF;
    radio_cb.radio_cfg.lora_cfg.preamble_length = RADIO_PREAMBLE_LEN;

    radio_cb.has_net_cfg = true;
    NetCfg net_cfg_zero = NetCfg_init_zero;
    radio_cb.net_cfg = net_cfg_zero;
    string def_msg = "KG5VBY Default Message";
    memcpy(radio_cb.net_cfg.beacon_msg, def_msg.c_str(), def_msg.size());
    radio_cb.net_cfg.beacon_interval = SIX_SECONDS;
    radio_cb.net_cfg.pld_len = FRAME_PAYLOAD_LEN;
    radio_cb.net_cfg.walsh_codes = false;
    radio_cb.net_cfg.invert_bits = false;
    radio_cb.net_cfg.voice_frames_per_frame = DEFAULT_VOICE_FRAMES_PER_FRAME;
    radio_cb.net_cfg.codec2_bitrate = DEFAULT_CODEC2_BITRATE;

    radio_cb.has_fec_cfg = true;
    FECCfg FECCfg_zero = FECCfg_init_zero;
    radio_cb.fec_cfg = FECCfg_zero;
    radio_cb.fec_cfg.type = FECCfg_Type_RSVGOLAY;
    radio_cb.fec_cfg.conv_order = FEC_CONV_ORDER;
    radio_cb.fec_cfg.conv_rate = FEC_CONV_RATE;
    radio_cb.fec_cfg.rs_num_roots = FEC_RS_NUM_ROOTS;

    radio_cb.has_test_cfg = true;
    TestCfg test_cfg_zero = TestCfg_init_zero;
    radio_cb.test_cfg = test_cfg_zero;
    radio_cb.test_cfg.cw_test_mode = false;
    radio_cb.test_cfg.preamble_test_mode = false;
    radio_cb.test_cfg.test_fec = false;

    radio_cb.has_esp_cfg_msg = true;
    radio_cb.esp_cfg_msg.has_esp0 = true;
    radio_cb.esp_cfg_msg.esp0.isBT = true;
    radio_cb.esp_cfg_msg.esp0.isAP = false;
    static constexpr int ESP_CFG_STR_MAX_LEN = 32;
    strncpy(radio_cb.esp_cfg_msg.esp0.ser_name, "QMesh-Serial0", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.bt_name, "QMesh-BT0", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.bt_pin, "0000", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.ip_addr, "192.168.10.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.gateway_addr, "192.168.10.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.subnet_addr, "192.168.10.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.dhcp_range_lo, "192.168.10.30", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.dhcp_range_hi, "192.168.10.40", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.ssid, "QMesh-AP0", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.pass, "password", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.multicast_addr, "224.0.0.0", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.local_port, "1002", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.remote_port, "1000", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp0.wifi_chan, "6", ESP_CFG_STR_MAX_LEN);
    radio_cb.esp_cfg_msg.has_esp1 = true;
    radio_cb.esp_cfg_msg.esp1.isBT = true;
    radio_cb.esp_cfg_msg.esp1.isAP = false;
    strncpy(radio_cb.esp_cfg_msg.esp1.ser_name, "QMesh-Serial1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.bt_name, "QMesh-BT1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.bt_pin, "0000", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.ip_addr, "192.168.20.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.gateway_addr, "192.168.20.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.subnet_addr, "192.168.20.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.dhcp_range_lo, "192.168.20.30", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.dhcp_range_hi, "192.168.20.40", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.ssid, "QMesh-AP1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.pass, "password", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.multicast_addr, "224.0.0.1", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.local_port, "1002", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.remote_port, "1000", ESP_CFG_STR_MAX_LEN);
    strncpy(radio_cb.esp_cfg_msg.esp1.wifi_chan, "6", ESP_CFG_STR_MAX_LEN);

    radio_cb.gps_en = false;

    radio_cb.watchdog_timer_en = false;

    radio_cb.valid = true;

    auto ser_msg = make_shared<SerMsg>();
    ser_msg->type(SerialMsg_Type_CONFIG);
    ser_msg->sys_cfg() = radio_cb;
    auto f_ps = make_shared<FilePseudoSerial>(f);
    PORTABLE_ASSERT(save_SerMsg(*ser_msg, *f_ps) == WRITE_SUCCESS);
    fflush(f);
    fclose(f); 
    f = fopen("/fs/settings.bin", "re");
    PORTABLE_ASSERT(f);
    fclose(f);
}


//extern Thread rx_serial_thread;
void load_settings_from_flash() {
    debug_printf(DBG_INFO, "Stats on settings.bin\r\n");  
    auto *f = fopen("/fs/settings.bin", "re");
    if(f == nullptr) {
        debug_printf(DBG_WARN, "Unable to open settings.bin. Creating new file with default settings\r\n");
        write_default_cfg();
        f = fopen("/fs/settings.bin", "re");
        PORTABLE_ASSERT(f);     
    }
    struct stat file_stat{};
    stat("/fs/settings.bin", &file_stat);
    debug_printf(DBG_INFO, "Size is %d\r\n", file_stat.st_size);
    PORTABLE_ASSERT(file_stat.st_size < 1024);
    auto ser_msg = make_shared<SerMsg>();
    auto f_ps = make_shared<FilePseudoSerial>(f);
    if(load_SerMsg(*ser_msg, *f_ps) != READ_SUCCESS) {
        fclose(f);
        debug_printf(DBG_WARN, "Invalid settings.bin. Creating new file with default settings\r\n");
        write_default_cfg();
        f = fopen("/fs/settings.bin", "re");
        PORTABLE_ASSERT(f);
    }
    PORTABLE_ASSERT(ser_msg->type() == SerialMsg_Type_CONFIG);
    PORTABLE_ASSERT(ser_msg->has_sys_cfg());
    radio_cb = ser_msg->sys_cfg();
    debug_printf(DBG_INFO, "Mode: %d\r\n", radio_cb.mode);
    debug_printf(DBG_INFO, "Address: %d\r\n", radio_cb.address);
    constexpr int MAX_ADDR = 31;
    PORTABLE_ASSERT(radio_cb.address <= MAX_ADDR);
    PORTABLE_ASSERT(radio_cb.has_radio_cfg);
    constexpr int MAX_FREQUENCY_COUNT = 8;
    PORTABLE_ASSERT(radio_cb.radio_cfg.frequencies_count > 0);
    PORTABLE_ASSERT(radio_cb.radio_cfg.frequencies_count <= MAX_FREQUENCY_COUNT);
    for(int i = 0; i < radio_cb.radio_cfg.frequencies_count; i++) {
        debug_printf(DBG_INFO, "Frequency %d: %d\r\n", i, radio_cb.radio_cfg.frequencies[i]); //NOLINT
        constexpr int MIN_FREQ = 400e6;
        constexpr int MAX_FREQ = 500e6; 
        PORTABLE_ASSERT(radio_cb.radio_cfg.frequencies[i] >= MIN_FREQ); //NOLINT
        PORTABLE_ASSERT(radio_cb.radio_cfg.frequencies[i] <= MAX_FREQ); //NOLINT
    }
    PORTABLE_ASSERT(radio_cb.radio_cfg.has_lora_cfg);
    debug_printf(DBG_INFO, "BW: %d\r\n", radio_cb.radio_cfg.lora_cfg.bw);
    uint32_t lora_bw = radio_cb.radio_cfg.lora_cfg.bw;
    constexpr uint32_t BW_125KHZ = 0;
    constexpr uint32_t BW_250KHZ = 1;
    constexpr uint32_t BW_500KHZ = 2;
    PORTABLE_ASSERT(lora_bw == BW_125KHZ || lora_bw == BW_250KHZ || lora_bw == BW_500KHZ);
    debug_printf(DBG_INFO, "CR: %d\r\n", radio_cb.radio_cfg.lora_cfg.cr);
    uint32_t lora_cr = radio_cb.radio_cfg.lora_cfg.cr;
    PORTABLE_ASSERT(lora_cr == 0 || lora_cr == 1 || lora_cr == 2 || lora_cr == 3 || lora_cr != 4);
    uint32_t lora_sf = radio_cb.radio_cfg.lora_cfg.sf;
    constexpr uint32_t SF_5 = 5;
    constexpr uint32_t SF_6 = 6;
    constexpr uint32_t SF_7 = 7;
    constexpr uint32_t SF_8 = 8;
    constexpr uint32_t SF_9 = 9;
    constexpr uint32_t SF_10 = 10;
    constexpr uint32_t SF_11 = 11;
    constexpr uint32_t SF_12 = 12;
    PORTABLE_ASSERT(lora_sf == SF_5 || lora_sf == SF_6 || lora_sf == SF_7 || lora_sf == SF_8 
            || lora_sf == SF_9 || lora_sf == SF_10 || lora_sf == SF_11 || lora_sf == SF_12);
    debug_printf(DBG_INFO, "SF: %d\r\n", radio_cb.radio_cfg.lora_cfg.sf);
    constexpr uint32_t MAX_PREAMBLE_LEN = 128;
    PORTABLE_ASSERT(radio_cb.radio_cfg.lora_cfg.preamble_length <= MAX_PREAMBLE_LEN);
    debug_printf(DBG_INFO, "Preamble Length: %d\r\n", radio_cb.radio_cfg.lora_cfg.preamble_length);
    PORTABLE_ASSERT(radio_cb.has_net_cfg);
    debug_printf(DBG_INFO, "Beacon Message: %s\r\n", radio_cb.net_cfg.beacon_msg);
    constexpr uint32_t MAX_BEACON_MSG_LEN = 64;
    PORTABLE_ASSERT(strlen(radio_cb.net_cfg.beacon_msg) <= MAX_BEACON_MSG_LEN);
    PORTABLE_ASSERT(strlen(radio_cb.net_cfg.beacon_msg) >= 1);
    constexpr uint32_t ONE_HOUR = 3600;
    debug_printf(DBG_INFO, "Beacon Interval: %d\r\n", radio_cb.net_cfg.beacon_interval);
    PORTABLE_ASSERT(radio_cb.net_cfg.beacon_interval <= ONE_HOUR);
    PORTABLE_ASSERT(radio_cb.net_cfg.beacon_interval >= 1);
    constexpr uint32_t MAX_TIMING_OFFSETS = 16;
    debug_printf(DBG_INFO, "Number of timing offset increments: %d\r\n", radio_cb.net_cfg.num_offsets);
    PORTABLE_ASSERT(radio_cb.net_cfg.num_offsets <= MAX_TIMING_OFFSETS);
    debug_printf(DBG_INFO, "Has a GPS: %d\r\n", static_cast<int>(radio_cb.gps_en));
    // Since really only 1/2 rate, constraint length=7 convolutional code works, we want to block 
    //  anything else from occurring and leading to weird crashes
    PORTABLE_ASSERT(radio_cb.has_fec_cfg);
    constexpr uint32_t ALLOWED_CONV_RATE = 2;
    PORTABLE_ASSERT(radio_cb.fec_cfg.conv_rate == ALLOWED_CONV_RATE);
    constexpr uint32_t ALLOWED_CONV_ORDER = 7;
    PORTABLE_ASSERT(radio_cb.fec_cfg.conv_order == ALLOWED_CONV_ORDER);
    // Voice parameters
    PORTABLE_ASSERT(VoiceMsgProcessor::valid_bitrate(radio_cb.net_cfg.codec2_bitrate));
    PORTABLE_ASSERT(VoiceMsgProcessor::valid_bitrate(radio_cb.net_cfg.codec2_bitrate));
    PORTABLE_ASSERT(radio_cb.net_cfg.voice_frames_per_frame == 4);
    constexpr int32_t MAX_RS_ROOTS = 16;
    PORTABLE_ASSERT(radio_cb.fec_cfg.rs_num_roots >= 4);
    PORTABLE_ASSERT(radio_cb.fec_cfg.rs_num_roots <= MAX_RS_ROOTS);
    constexpr int MAX_TX_POWER_DBM = 22;
    PORTABLE_ASSERT(radio_cb.radio_cfg.tx_power <= MAX_TX_POWER_DBM);
    PORTABLE_ASSERT(radio_cb.radio_cfg.tx_power >= 0);
    radio_cb.net_cfg.pld_len = VoiceMsgProcessor::size();
    constexpr uint32_t MAX_PLD_LEN = 128;
    debug_printf(DBG_INFO, "Payload Length: %d\r\n", radio_cb.net_cfg.pld_len);
    PORTABLE_ASSERT(radio_cb.net_cfg.pld_len <= MAX_PLD_LEN);
    PORTABLE_ASSERT(radio_cb.net_cfg.pld_len >= 1);

    PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.ser_name).empty());
    if(radio_cb.esp_cfg_msg.esp0.isBT) {
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.bt_name).empty());
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.bt_pin).empty());
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.bt_pin).size() <= 8); 
    } else {
        struct in_addr inp{};
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.ip_addr, &inp) == 1); 
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.gateway_addr, &inp) == 1); 
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.subnet_addr, &inp) == 1); 
        if(radio_cb.esp_cfg_msg.esp0.isAP) {
            PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.dhcp_range_lo, &inp) == 1); 
            PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.dhcp_range_hi, &inp) == 1); 
        }
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.ssid).empty());    
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp0.pass).empty());
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp0.multicast_addr, &inp) == 1); 
        string test_port_addr0(radio_cb.esp_cfg_msg.esp0.multicast_addr);
        test_port_addr0.append(":");
        test_port_addr0.append(radio_cb.esp_cfg_msg.esp0.remote_port);
        PORTABLE_ASSERT(inet_aton(test_port_addr0.c_str(), &inp) == 1); 
        string test_port_addr1(radio_cb.esp_cfg_msg.esp0.multicast_addr);
        test_port_addr1.append(":");
        test_port_addr1.append(radio_cb.esp_cfg_msg.esp0.local_port);
        PORTABLE_ASSERT(inet_aton(test_port_addr1.c_str(), &inp) == 1); 
    }
    PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.ser_name).empty());
    if(radio_cb.esp_cfg_msg.esp1.isBT) {
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.bt_name).empty());
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.bt_pin).empty());
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.bt_pin).size() <= 8); 
    } else {
        struct in_addr inp{};
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.ip_addr, &inp) == 1); 
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.gateway_addr, &inp) == 1);
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.subnet_addr, &inp) == 1);
        if(radio_cb.esp_cfg_msg.esp0.isAP) {
            PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.dhcp_range_lo, &inp) == 1);
            PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.dhcp_range_hi, &inp) == 1);
        }
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.ssid).empty());    
        PORTABLE_ASSERT(string(radio_cb.esp_cfg_msg.esp1.pass).empty());
        PORTABLE_ASSERT(inet_aton(radio_cb.esp_cfg_msg.esp1.multicast_addr, &inp) == 1);
        string test_port_addr0(radio_cb.esp_cfg_msg.esp1.multicast_addr);
        test_port_addr0.append(":");
        test_port_addr0.append(radio_cb.esp_cfg_msg.esp1.remote_port);
        PORTABLE_ASSERT(inet_aton(test_port_addr0.c_str(), &inp) == 1);
        string test_port_addr1(radio_cb.esp_cfg_msg.esp1.multicast_addr);
        test_port_addr1.append(":");
        test_port_addr1.append(radio_cb.esp_cfg_msg.esp1.local_port);
        PORTABLE_ASSERT(inet_aton(test_port_addr1.c_str(), &inp) == 1);   
    }

    radio_cb.valid = true;

#if 0
    // Check if low-power mode is set. If so, delete the UART
    //rx_serial_thread.start(rx_serial_thread_fn);
    FILE *low_power_fh = fopen("/fs/low_power.mode", "r");
    if(low_power_fh) {
        //rx_serial_thread.terminate();
        oled->displayOff();
        mbed_file_handle(STDIN_FILENO)->enable_input(false); 
        //gps_serial.enable_input(false); 
        fclose(low_power_fh);
    }
#endif
}


void save_settings_to_flash() {
    debug_printf(DBG_INFO, "Opening settings.bin...\r\n");
    auto *f = fopen("/fs/settings.bin", "we"); 
    PORTABLE_ASSERT(f);
    auto ser_msg = make_shared<SerMsg>();
    ser_msg->type(SerialMsg_Type_CONFIG);
    PORTABLE_ASSERT(radio_cb.valid);
    ser_msg->sys_cfg() = radio_cb;
    auto f_ps = make_shared<FilePseudoSerial>(f);
    PORTABLE_ASSERT(!save_SerMsg(*ser_msg, *f_ps));
    fclose(f);  
}


void log_boot() {
    auto ser_msg = make_shared<SerMsg>();
    ser_msg->type(SerialMsg_Type_BOOT_LOG);
    time_t my_time = time(nullptr);
    ser_msg->boot_log_msg().boot_time = my_time;
    ser_msg->boot_log_msg().count = 0;
    ser_msg->boot_log_msg().valid = true;

    auto *f = fopen("/fs/boot_log.bin", "a+e");
    PORTABLE_ASSERT(f);
    auto f_ps = make_shared<FilePseudoSerial>(f);
    save_SerMsg(*ser_msg, *f_ps);
    fclose(f);      
}


auto open_logfile() -> FILE * {
    // Step one: get the size of the current logfile. If current logfile is too big,
    //  move it down the "logfile stack".
    auto *f = fopen("/fs/log/logfile.bin", "re");
    if(f == nullptr) {
        debug_printf(DBG_INFO, "Need to create the logfile\r\n");
        f = fopen("/fs/log/logfile.bin", "we");
        PORTABLE_ASSERT(f);
    }
    fclose(f);
    struct stat logfile_statbuf{};
    fs->stat("log/logfile.bin", &logfile_statbuf);
    if(logfile_statbuf.st_size > MAX_LOGFILE_SIZE) {
        for(int i = NUM_LOGFILES; i >= 0; i--) {
            stringstream logfile_name;
            stringstream logfile_name_plusone;
            logfile_name << "log/logfile" << setw(3) << setfill('0') << i << ".json";
            debug_printf(DBG_INFO, "Now moving %s\r\n", logfile_name.str().c_str());
            logfile_name_plusone << "log/logfile" << setw(3) << setfill('0') << i+1 << ".json";
            fs->rename(logfile_name.str().c_str(), logfile_name_plusone.str().c_str());
        }
        fs->rename("log/logfile.bin", "log/logfile000.bin");
    }
    f = fopen("/fs/log/logfile.bin", "a+e");
    PORTABLE_ASSERT(f != nullptr);
    return f;
}


extern time_t boot_timestamp;
void nv_log_fn() {
    auto *log_dir = opendir("/fs/log");
    if((log_dir == nullptr) && errno == ENOENT) {
        debug_printf(DBG_INFO, "Log directory does not exist. Creating...\r\n");
        constexpr int UGO_RWX = 777;
        if(mkdir("/fs/log", UGO_RWX) != 0) {
            PORTABLE_ASSERT(false);
        }
        log_dir = opendir("/fs/log");
        if(log_dir == nullptr) {
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
            PORTABLE_ASSERT(false);
        }
    }
    debug_printf(DBG_INFO, "Now opening the logfile\r\n");
    debug_printf(DBG_INFO, "First set\r\n");
    auto *f = open_logfile();
    for(;;) {
        // Write the latest frame to disk
        auto log_frame = nv_logger_mail->dequeue_mail();  
        int16_t rssi = 0;
        uint16_t rx_size = 0;
        int8_t snr = 0;
        log_frame->getRxStats(rssi, snr, rx_size);
        LogMsg log_msg;
		time_t my_time = time(nullptr);
		log_msg.timestamp = my_time;
        log_msg.sender = log_frame->getSender();
        log_msg.ttl = log_frame->getTTL();
        log_msg.stream_id = log_frame->getStreamID();
        log_msg.rssi = static_cast<int>(rssi);
        log_msg.snr = static_cast<int>(snr); //NOLINT
        log_msg.rx_size = rx_size;
        log_msg.comp_crc = log_frame->calcCRC();
        log_msg.crc = log_frame->getCRC();
#if 0
        if(radio_cb.gps_en) {
            log_msg.has_gps_msg = true;
            float gps_lat, gps_lon;
            gpsd_get_coordinates(gps_lat, gps_lon);
            log_msg.gps_msg.valid = true;
            log_msg.gps_msg.lat = gps_lat;
            log_msg.gps_msg.lon = gps_lon;
        } 
#endif
        time_t cur_time = 0;
        time(&cur_time);
        time_t uptime = cur_time - boot_timestamp;
        log_msg.uptime = uptime;

        auto ser_msg = make_shared<SerMsg>();
        ser_msg->type(SerialMsg_Type_LOG);
        ser_msg->log_msg() = log_msg;
        auto f_ps = make_shared<FilePseudoSerial>(f);
        PORTABLE_ASSERT(!save_SerMsg(*ser_msg, *f_ps));
        fclose(f);
        f = open_logfile();
    }
}