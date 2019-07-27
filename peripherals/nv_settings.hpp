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

 #ifndef NV_SETTINGS_HPP
 #define NV_SETTINGS_HPP

#include "mbed.h"
#include "params.hpp"

void nv_log_fn(void);

class EEPROM {
protected:
    I2C *i2c_dev;
public:
    EEPROM(I2C *my_i2c_dev) {
        i2c_dev = my_i2c_dev;
    }

    // Read a byte from the EEPROM
    uint8_t readEEPROMByte(uint32_t addr);

    // Write a byte to the EEPROM
    void writeEEPROMByte(uint32_t addr, uint8_t val);

    // Read multiple bytes from the EEPROM into a buffer
    void readEEPROMBlock(uint32_t addr, uint32_t size, void *buf);

    // Write multiple bytes from a supplied buffer into the EEPROM
    void writeEEPROMBlock(uint32_t addr, uint32_t size, void *buf);

    // Function that performs some tests on the EEPROM, mainly as a way
    //  of checking out a newly-built EEPROM board.
    void testEEPROM(void);
};

typedef enum {
    MESH_MODE_BEACON,
    MESH_MODE_NORMAL,
} mesh_mode_t;


typedef struct {
    uint32_t magic;
    mesh_mode_t mode;
    uint32_t freq;
    uint8_t sf;
    uint8_t bw;
    uint8_t cr;
} nv_settings_t;

typedef struct {
    uint8_t session_nonce;
    uint16_t pld_crc;
    uint16_t pld_comp_crc;
    uint16_t hdr_crc;
    uint16_t hdr_comp_crc;
    int16_t rssi;
    int8_t snr;
    uint16_t rx_size;
} log_val_t;

class NVSettings {
protected:
    EEPROM *eeprom;
    size_t log_baseaddr;
public:
    nv_settings_t nv_settings;

    NVSettings(EEPROM *my_eeprom) {
        log_baseaddr = LOG_BASEADDR;
        eeprom = my_eeprom;
        loadEEPROM();
    }

    void loadEEPROM(void); 

    void saveEEPROM(void) {
        eeprom->writeEEPROMBlock(0, sizeof(nv_settings), (void *) &nv_settings);
    }

    nv_settings_t getNVSettings(void) {
        return nv_settings;
    }

    void putNVSettings(nv_settings_t &settings) {
        nv_settings = settings;
        saveEEPROM();
        loadEEPROM();
    }

    mesh_mode_t getMode(void) {
        loadEEPROM();
        return nv_settings.mode;
    }

    void setMode(mesh_mode_t my_mode) {
        loadEEPROM();
        nv_settings.mode = my_mode;
        saveEEPROM();
    }

    uint32_t getFrequency(void) {
        loadEEPROM();
        return nv_settings.freq;
    }

    void setFrequency(uint32_t frequency) {
        loadEEPROM();
        nv_settings.freq = frequency;
        saveEEPROM();
    }

    uint8_t getBW(void) {
        loadEEPROM();
        return nv_settings.bw;
    }

    void setBW(uint8_t bandwidth) {
        loadEEPROM();
        nv_settings.bw = bandwidth;
        saveEEPROM();
    }

    uint8_t getSF(void) {
        loadEEPROM();
        return nv_settings.sf;
    }

    void setSF(uint8_t spreading_factor) {
        loadEEPROM();
        nv_settings.sf = spreading_factor;
        saveEEPROM();
    }

    uint8_t getCR(void) {
        loadEEPROM();
        return nv_settings.cr;
    }

    void setCR(uint8_t coding_rate) {
        loadEEPROM();
        nv_settings.cr = coding_rate;
        saveEEPROM();
    }

    void writeLogVal(log_val_t log_val);
};

extern NVSettings nv_settings;

#endif /* NV_SETTINGS_HPP */

