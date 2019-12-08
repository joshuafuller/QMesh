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
#include "buttons.hpp"
#include "serial_data.hpp"
#include "LittleFileSystem.h"

void reboot_system(void) {
    debug_printf(DBG_INFO, "Unmounting the filesystem...\r\n");
    int err = fs.unmount();
    debug_printf(DBG_INFO, "%s\n", (err ? "Fail :(" : "OK"));
    bd.sync();
    debug_printf(DBG_INFO, "Now rebooting the system...\r\n");
    ThisThread::sleep_for(3000);
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