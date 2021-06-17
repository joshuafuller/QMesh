/*
QMesh
Copyright (C) 2021 Daniel R. Fay

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
#include "peripherals.hpp"
#include "params.hpp"
#include "mem_trace.hpp"
#include "Adafruit_SSD1306.h"
#include "SoftI2C.h"
#include "USBSerial.h"
#include "fw_update.hpp"
#include <memory>


constexpr uint32_t THREAD_STACK_SIZE = 4096;
constexpr uint32_t I2C_FREQ = 400000;
constexpr uint32_t SLEEP_TIME_1S = 1000;
enum BUTTON_PRESS {
    NOT_PRESSED = 0,
    PRESSED = 1
};
time_t boot_timestamp;

EventQueue background_queue; /// Event Queue
Thread background_thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "BG"); /// Background thread

// Devices
DigitalIn user_button(USER_BUTTON); /// Button. When held down, will revert to the golden firmware
SoftI2C oled_i2c(PB_8, PB_9); /// OLED display's I2C
unique_ptr<Adafruit_SSD1306_I2c> oled; /// OLED display


auto main() -> int
{
    // Set up the LEDs
    led1.setEvtQueue(&background_queue);
    led2.setEvtQueue(&background_queue);
    led3.setEvtQueue(&background_queue);  

    oled_i2c.frequency(I2C_FREQ);
    oled_i2c.start();
    
    constexpr uint32_t SSD1306_INIT_1 = 0x78;
    constexpr uint32_t SSD1306_NUM_LINES = 32;
    constexpr uint32_t SSD1306_NUM_COLS = 128;
    oled = make_unique<Adafruit_SSD1306_I2c>(oled_i2c, PD_13, SSD1306_INIT_1, 
                        SSD1306_NUM_LINES, SSD1306_NUM_COLS);

    oled->printf("Welcome to QMesh\r\n");
    oled->display();

    led1.LEDBlink();
    oled->printf("In bootloader...\r\n");
    oled->display();
    if(start_fs()) {
        // First, check for the update file that the program is supposed to remove.
        FILE *boot_fail = fopen("/fs/boot.fail", "r");
        if(boot_fail != nullptr) {
            oled->printf("Boot fail!\r\n");
            oled->display();
            // Don't want to end up in an infinite loop where the golden firmware
            //  keeps getting unsuccessfully loaded
            fclose(boot_fail);
            fs.remove("boot.fail");
            // Load the golden firmware, if it exists
            string fname("golden.bin");
            if(check_for_update(fname)) {
                apply_update(fname, true);
            }
        } else if(user_button == PRESSED) {
            oled->printf("Button pressed!\r\n");
            oled->display();
            fclose(boot_fail);
            fs.remove("boot.fail");
            boot_fail = fopen("/fs/boot.fail", "w");
            fclose(boot_fail);
            // Load the golden firmware, if it exists
            string fname("golden.bin");
            if(check_for_update(fname)) {
                apply_update(fname, true);
            }
        } else {
            fclose(boot_fail);
            fs.remove("boot.fail");
            boot_fail = fopen("/fs/boot.fail", "w");
            fclose(boot_fail);
            string fname("update.bin");
            if(check_for_update(fname)) {
                apply_update(fname, true);
            }
        }
        ThisThread::sleep_for(SLEEP_TIME_1S);
    }
    mbed_start_application(POST_APPLICATION_ADDR);
}

