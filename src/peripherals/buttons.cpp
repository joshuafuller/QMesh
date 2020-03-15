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

extern Thread tx_serial_thread, rx_serial_thread;
extern Thread mesh_protocol_thread;
extern Thread beacon_thread;
extern Thread nv_log_thread;
extern rtos::Thread *lora_irq_thread;

volatile bool rebooting = false;

void reboot_system(void) {
	rebooting = true;
    if(mesh_protocol_thread.get_state() != Thread::Deleted) {
	    mesh_protocol_thread.join();
    }
    if(beacon_thread.get_state() != Thread::Deleted) {
	    beacon_thread.join();
    }
    if(lora_irq_thread->get_state() != Thread::Deleted) {
        lora_irq_thread->terminate();
    }
    debug_printf(DBG_INFO, "Unmounting the filesystem...\r\n");
    int err = fs.unmount();
    debug_printf(DBG_INFO, "%s\n", (err ? "Fail :(\r\n" : "OK\r\n"));
    bd.sync();
    debug_printf(DBG_INFO, "Now rebooting the system...\r\n");
    ThisThread::sleep_for(500);
    NVIC_SystemReset();
}

Semaphore btn_pressed_irq(0);
void button_thread_fn(void) {
    btn_pressed_irq.acquire();
    debug_printf(DBG_WARN, "Soft reset button pressed\r\n");
    reboot_system();
}

PushButton::PushButton(PinName button) {
    was_pressed = false;
    btn = new InterruptIn(button);
    btn->rise(callback(this, &PushButton::btnInterrupt));
}

void PushButton::btnInterrupt(void) {
    was_pressed = true;
    btn_pressed_irq.release();
}
    
bool PushButton::getPressed(void) {
    bool pressed_val = was_pressed;
    was_pressed = false;
    return pressed_val;
}

PushButton::~PushButton() {
    delete btn;
}