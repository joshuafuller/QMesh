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
#include "nvstore.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "I2C.h"
#include "serial_data.hpp"

// Read a byte from the EEPROM
uint8_t EEPROM::readEEPROMByte(uint32_t addr) {
    // Set the address
    uint8_t addr_bytes[2];
    uint8_t top_addr_bit = (addr >> 16 & 0x01);
    uint8_t dev_addr = 0xA0 | (top_addr_bit << 1);
    addr_bytes[0] = (uint8_t) ((addr >> 8) & 0x000000FF);
    addr_bytes[1] = (uint8_t) (addr & 0x000000FF);
    MBED_ASSERT(i2c_dev->write(dev_addr, (char *) addr_bytes, sizeof(addr_bytes), true));
    // Read out the byte
    dev_addr = 0xA0;
    uint8_t ret_val = 0xFF;
    MBED_ASSERT(i2c_dev->read(dev_addr, (char *) &ret_val, 1, false));
    return ret_val;
}


// Write a byte to the EEPROM
void EEPROM::writeEEPROMByte(uint32_t addr, uint8_t val) {
    uint8_t addr_bytes[2];
    uint8_t top_addr_bit = (addr >> 16 & 0x01);
    uint8_t dev_addr = 0xA0 | (top_addr_bit << 1);
    MBED_ASSERT(i2c_dev->write(dev_addr, (char *) &val, 1, true));
}


// Read multiple bytes from the EEPROM into a buffer
void EEPROM::readEEPROMBlock(uint32_t addr, uint32_t size, void *buf) {
    for(uint32_t idx = addr; addr < addr+size; addr++) {
        *((uint8_t *)buf+idx) = readEEPROMByte(idx);
    }
}


// Write multiple bytes from a supplied buffer into the EEPROM
void EEPROM::writeEEPROMBlock(uint32_t addr, uint32_t size, void *buf) {
    for(uint32_t idx = addr; addr < addr+size; addr++) {
        writeEEPROMByte(idx, *((uint8_t *)buf+idx));
    }
}


// Function that performs some tests on the EEPROM, mainly as a way
//  of checking out a newly-built EEPROM board.
void EEPROM::testEEPROM(void) {
    // Test results variables
    size_t num_correct_walking_bits = 0;
    size_t num_incorrect_walking_bits = 0;
    size_t num_correct_seq = 0;
    size_t num_incorrect_seq = 0;
    debug_printf(DBG_INFO, "EEPROM TESTING\r\n");
    debug_printf(DBG_INFO, "----------\r\n");
    debug_printf(DBG_INFO, "Performing walking bits test...\r\n");
    for(int i = 0; i < 1000; i++) {
        debug_printf(DBG_INFO, "Writing walking bits iteration %d\r\n", i);
        uint8_t write_val = i & 0x1 ? 0xAA : 0x55;
        for(int j = 0; j < (1 << 19); j++) {
            writeEEPROMByte(j, write_val);
        }
        debug_printf(DBG_INFO, "Reading walking bits iteration %d\r\n", i);        
        for(int j = 0; i < (1 << 19); j++) {
            if(readEEPROMByte(j) == write_val) {
                num_correct_walking_bits += 1;
            }
            else {
                num_incorrect_walking_bits += 1;
            }
        }
    }
    debug_printf(DBG_INFO, "----------\r\n");
    debug_printf(DBG_INFO, "Performing sequence test...\r\n");    
    for(int i = 0; i < 1000; i++) {
        debug_printf(DBG_INFO, "Writing sequence iteration %d\r\n", i);
        for(int j = 0; j < (1 << 19); j++) {
            writeEEPROMByte(j, j);
        }
        debug_printf(DBG_INFO, "Reading sequence iteration %d\r\n", i);
        for(int j = 0; j < (1 << 19); j++) {
            if(readEEPROMByte(j) == j) {
                num_correct_seq += 1;
            }
            else {
                num_incorrect_seq += 1;
            }
        }
    }
    debug_printf(DBG_INFO, "----------\r\n");   
    debug_printf(DBG_INFO, "EEPROM test complete!\r\n");
    debug_printf(DBG_INFO, "\t Walking bits test: %d correct, %d incorrect \r\n", 
            num_correct_walking_bits, num_incorrect_walking_bits);
    debug_printf(DBG_INFO, "\t Sequence test: %d correct, %d incorrect \r\n",
            num_correct_seq, num_incorrect_seq);
}

