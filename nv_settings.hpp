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

// Load the settings saved into the NV storage
void loadNVSettings(void);

// Save the current settings to the NV storage
void saveNVSettings(void);

// Clear the NV MESH_SETTINGS_KEY
void clearNVSettings(void);


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

#endif /* NV_SETTINGS_HPP */

