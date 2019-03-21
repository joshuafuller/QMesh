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

 #endif /* NV_SETTINGS_HPP */