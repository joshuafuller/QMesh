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
#include "buttons.hpp"
#include "serial_data.hpp"
#include "LittleFileSystem.h"

void reboot_system(void) {
    debug_printf(DBG_INFO, "Unmounting the filesystem...\r\n");
    int err = fs.unmount();
    debug_printf(DBG_INFO, "%s\n", (err ? "Fail :(" : "OK"));
    bd.sync();
    debug_printf(DBG_INFO, "Rebooting the system in 1s...\r\n");
    wait(1.0);
    debug_printf(DBG_INFO, "Now rebooting the system...\r\n");
    NVIC_SystemReset();
}

// Pin name for the button is USER_BUTTON
PushButton button(USER_BUTTON);

PushButton::PushButton(PinName button) {
    was_pressed = false;
    btn = new InterruptIn(button);
    btn->rise(callback(this, &PushButton::btnInterrupt));
}

void PushButton::btnInterrupt(void) {
    was_pressed = true;
    reboot_system();
}
    
bool PushButton::getPressed(void) {
    bool pressed_val = was_pressed;
    was_pressed = false;
    return pressed_val;
}

PushButton::~PushButton() {
    delete btn;
}