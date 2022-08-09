/*
QMesh
Copyright (C) 2022 Daniel R. Fay

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
#include "Adafruit_SSD1306.h"

//extern Thread tx_serial_thread, rx_serial_thread;
extern portability::Thread *mesh_protocol_thread;
extern portability::Thread *beacon_thread;
extern portability::Thread *nv_log_thread;
extern portability::Thread *oled_mon_thread;
extern portability::Thread *lora_irq_thread;
extern portability::Thread *btn_evt_thread;

extern shared_ptr<Adafruit_SSD1306_I2c> oled;
//extern UARTSerial gps_serial;


void button_thread_fn();
void button_fn() {
    FILE *f = fopen("/fs/low_power.mode", "re");
    if(f != nullptr) {
        fclose(f);
        fs->remove("low_power.mode");
        mbed_file_handle(STDIN_FILENO)->enable_input(true);
        //gps_serial.enable_input(true);
        oled->displayOn();
        debug_printf(DBG_WARN, "Rebooting in 1s so serial port will work...\r\n");
        portability::sleep(ONE_SECOND);
        reboot_system();
    }
    else {
        FILE *fw = fopen("/fs/low_power.mode", "we");
        if(fw != nullptr) {
            fprintf(fw, "In low power mode\r\n");
            fclose(fw);
            mbed_file_handle(STDIN_FILENO)->enable_input(false);   
            //rx_serial_thread.terminate();
            //gps_serial.enable_input(false);
            oled->displayOff();
        }
    }
} 

PushButton::PushButton(PinName button) {
    was_pressed = false;
    btn = new portability::InterruptIn(button);
}

void PushButton::SetQueue(portability::EventQueue &evt_queue) {
    btn->rise(evt_queue.event(button_fn));
}
    
auto PushButton::getPressed() -> bool {
    bool pressed_val = was_pressed;
    was_pressed = false;
    return pressed_val;
}

PushButton::~PushButton() {
    delete btn;
}