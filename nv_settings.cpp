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

#if 0
#define RADIO_SETTINGS_KEY 0
#define MESH_SETTINGS_KEY 1

// Non-volatile settings
extern union nv_radio_settings_union nv_radio_settings;
extern bool mesh_enable;


// Load the settings saved into the NV storage
void loadNVSettings(void) {
    uint16_t actual_len_bytes = 0;
    NVStore &nvstore = NVStore::get_instance();
    // Load the radio settings
    nvstore.get(RADIO_SETTINGS_KEY, sizeof(nv_radio_settings.buf), nv_radio_settings.buf, actual_len_bytes);
    // Load the mesh settings
    nvstore.get(MESH_SETTINGS_KEY, sizeof(bool), &mesh_enable, actual_len_bytes);
}


// Save the current settings to the NV storage
void saveNVSettings(void) {
    NVStore &nvstore = NVStore::get_instance();
    // Save the radio settings
    nvstore.set(RADIO_SETTINGS_KEY, sizeof(nv_radio_settings.buf), nv_radio_settings.buf);
    // Save the mesh settings
    nvstore.set(MESH_SETTINGS_KEY, sizeof(bool), &mesh_enable);
}


// Clear the NV MESH_SETTINGS_KEY
void clearNVSettings(void) {
    NVStore &nvstore = NVStore::get_instance();
    nvstore.reset();
}
#endif