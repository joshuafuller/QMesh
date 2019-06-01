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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "I2C.h"
#include "serial_data.hpp"
#include "nv_settings.hpp"

#define NUM_TEST_ITERS 10

// Read a byte from the EEPROM
uint8_t EEPROM::readEEPROMByte(uint32_t addr) {
    // Set the address
    uint8_t addr_bytes[2];
    uint8_t top_addr_bit = (addr >> 16 & 0x01);
    uint8_t dev_addr = 0xA0 | (top_addr_bit << 1);
    addr_bytes[0] = (uint8_t) ((addr >> 8) & 0x000000FF);
    addr_bytes[1] = (uint8_t) (addr & 0x000000FF);
    MBED_ASSERT(i2c_dev->write(dev_addr, (char *) addr_bytes, sizeof(addr_bytes), true) == 0);
    // Read out the byte
    dev_addr = 0xA0;
    uint8_t ret_val = 0xFF;
    MBED_ASSERT(i2c_dev->read(dev_addr, (char *) &ret_val, 1, false) == 0);
    return ret_val;
}


// Write a byte to the EEPROM
void EEPROM::writeEEPROMByte(uint32_t addr, uint8_t val) {
    uint8_t data_bytes[3];
    uint8_t top_addr_bit = (addr >> 16 & 0x01);
    uint8_t dev_addr = 0xA0 | (top_addr_bit << 1);
    data_bytes[0] = (uint8_t) ((addr >> 8) & 0x000000FF);
    data_bytes[1] = (uint8_t) (addr & 0x000000FF);
    data_bytes[2] = val;
    MBED_ASSERT(i2c_dev->write(dev_addr, (char *) data_bytes, 3, false) == 0);
    // Poll the device until the write operation is complete
    while(true) {
        char test_val = 0;
        i2c_dev->write(dev_addr, (char *) data_bytes, 2, false);
        if(!i2c_dev->read(dev_addr, &test_val, 1, false)) {
            break;
        }
    }
}


// Read multiple bytes from the EEPROM into a buffer
void EEPROM::readEEPROMBlock(uint32_t addr, uint32_t size, void *buf) {
    for(uint32_t idx = addr; idx < addr+size; idx++) {
        *((uint8_t *)buf+idx) = readEEPROMByte(idx);
    }
}


// Write multiple bytes from a supplied buffer into the EEPROM
void EEPROM::writeEEPROMBlock(uint32_t addr, uint32_t size, void *buf) {
    for(uint32_t idx = addr; idx < addr+size; idx++) {
        writeEEPROMByte(idx, *((uint8_t *)buf+idx));
    }
}


// Function that performs some tests on the EEPROM, mainly as a way
//  of checking out a newly-built EEPROM board.
void EEPROM::testEEPROM(void) {
    // Test results variables
    int num_correct_walking_bits = 0;
    int num_incorrect_walking_bits = 0;
    int num_correct_seq = 0;
    int num_incorrect_seq = 0;
    debug_printf(DBG_INFO, "EEPROM TESTING\r\n");
    debug_printf(DBG_INFO, "----------\r\n");
    debug_printf(DBG_INFO, "Performing walking bits test...\r\n");
    for(int i = 0; i < NUM_TEST_ITERS; i++) {
        debug_printf(DBG_INFO, "Writing walking bits iteration %d/%d\r\n", i+1, NUM_TEST_ITERS);
        uint8_t write_val = i & 0x1 ? 0xAA : 0x55;
        for(int j = 0; j < (1 << 17); j++) {          
            if((j & 0x00000FFF) == 0) {
                debug_printf(DBG_INFO, "Byte %d\r\n", j);
            }
            uint8_t test_val = write_val;
            if(j & 0x100) {
                test_val ^= 0xFF;
            }
            writeEEPROMByte(j, test_val);
        }
        debug_printf(DBG_INFO, "Reading walking bits iteration %d/%d\r\n", i+1, NUM_TEST_ITERS);        
        for(int j = 0; j < (1 << 17); j++) {
            uint8_t test_val = write_val;
            if(j & 0x100) {
                test_val ^ 0xFF;
            }            
            if(readEEPROMByte(j) == test_val) {
                num_correct_walking_bits += 1;
            }
            else {
                num_incorrect_walking_bits += 1;
            }        
            if((j & 0x00000FFF) == 0) {
                debug_printf(DBG_INFO, "Byte %d\r\n", j);
            }
        }
        debug_printf(DBG_INFO, "Iteration %d complete, %d correct, %d incorrect\r\n", 
                i, num_correct_walking_bits, num_incorrect_walking_bits);
    }
    debug_printf(DBG_INFO, "----------\r\n");
    debug_printf(DBG_INFO, "Performing sequence test...\r\n");    
    for(int i = 0; i < NUM_TEST_ITERS; i++) {
        debug_printf(DBG_INFO, "Writing sequence iteration %d/%d\r\n", i+1, NUM_TEST_ITERS);
        for(int j = 0; j < (1 << 17); j++) {           
            if((j & 0x00000FFF) == 0) {
                debug_printf(DBG_INFO, "Byte %d\r\n", j);
            }
            writeEEPROMByte(j, (j ^ (j>>8 & 0xFF)) & 0xFF);
        }
        debug_printf(DBG_INFO, "Reading sequence iteration %d/%d\r\n", i+1, NUM_TEST_ITERS);
        for(int j = 0; j < (1 << 17); j++) {         
            if((j & 0x00000FFF) == 0) {
                debug_printf(DBG_INFO, "Byte %d\r\n", j);
            }       
            if(readEEPROMByte(j) == ((j ^ (j>>8 & 0xFF)) & 0xFF)) {
                num_correct_seq += 1;
            }
            else {
                num_incorrect_seq += 1;
            }
        }
        debug_printf(DBG_INFO, "Iteration %d complete, %d correct, %d incorrect\r\n", 
                i, num_correct_seq, num_incorrect_seq);
    }
    debug_printf(DBG_INFO, "----------\r\n");   
    debug_printf(DBG_INFO, "EEPROM test complete!\r\n");
    debug_printf(DBG_INFO, "\t Walking bits test: %d correct, %d incorrect \r\n", 
            num_correct_walking_bits, num_incorrect_walking_bits);
    debug_printf(DBG_INFO, "\t Sequence test: %d correct, %d incorrect \r\n",
            num_correct_seq, num_incorrect_seq);

    // Just spin
    while(true);
}


void NVSettings::loadEEPROM(void) {
    debug_printf(DBG_INFO, "Reading EEPROM...\r\n");
    eeprom->readEEPROMBlock(0, sizeof(nv_settings), (void *) &nv_settings);
    if(nv_settings.magic != (0xFEEDFACE ^ sizeof(nv_settings))) {  // Invalid settings block
        debug_printf(DBG_INFO, "NOTE: No valid configuration stored in EEPROM.\r\n");
        debug_printf(DBG_INFO, "Saving default configuration to EEPROM.\r\n");
        nv_settings.magic = 0xFEEDFACE ^ sizeof(nv_settings);
        nv_settings.mode = MESH_MODE_NORMAL;
        nv_settings.freq = 915000000;
        nv_settings.sf = 9;
        nv_settings.bw = 7;
        nv_settings.cr = 4;
        saveEEPROM();
    }
    debug_printf(DBG_INFO, "----------\r\n"); 
    bool node_mode = nv_settings.mode;
    debug_printf(DBG_INFO, "Mode: %s\r\n", node_mode == MESH_MODE_NORMAL ? "NORMAL":"BEACON");
    debug_printf(DBG_INFO, "Frequency: %d\r\n", nv_settings.freq);
    debug_printf(DBG_INFO, "SF: %d\r\n", nv_settings.sf);
    debug_printf(DBG_INFO, "BW: %d\r\n", nv_settings.bw);
    debug_printf(DBG_INFO, "CR: %d\r\n", nv_settings.cr);
    debug_printf(DBG_INFO, "----------\r\n");
    debug_printf(DBG_INFO, "Configuration loading complete.\r\n");
}

////kljlkj

