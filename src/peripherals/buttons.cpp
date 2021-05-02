/*
QMesh
Copyright (C) 2020 Daniel R. Fay

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
#include "LittleFileSystem.h"
#include "Adafruit_SSD1306.h"

//extern Thread tx_serial_thread, rx_serial_thread;
extern Thread mesh_protocol_thread;
extern Thread beacon_thread;
extern Thread nv_log_thread;
extern Thread oled_mon_thread;
extern rtos::Thread *lora_irq_thread;
extern Thread btn_evt_thread;

extern Adafruit_SSD1306_I2c *oled;
//extern UARTSerial gps_serial;

volatile bool rebooting = false;

void button_thread_fn(void);

PushButton::PushButton(PinName button) {
    was_pressed = false;
    btn = new InterruptIn(button);
}

void PushButton::SetQueue(EventQueue &evt_queue) {
    //btn->rise(evt_queue.event(button_fn));
}
    
bool PushButton::getPressed(void) {
    bool pressed_val = was_pressed;
    was_pressed = false;
    return pressed_val;
}

PushButton::~PushButton() {
    delete btn;
}