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


extern IndicatorLED led1, led2, led3;

EventQueue background_queue;
Thread background_thread(osPriorityNormal, 4096, NULL, "BG"); /// Background thread

time_t boot_timestamp;


#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

void send_status(void);

DigitalIn user_button(USER_BUTTON);

SoftI2C oled_i2c(PB_8, PB_9);
Adafruit_SSD1306_I2c *oled;

void print_stats()
{
    {
    mbed_stats_cpu_t stats;
    mbed_stats_cpu_get(&stats);

    printf("Uptime: %-20lld", stats.uptime);
    printf("Idle time: %-20lld", stats.idle_time);
    printf("Sleep time: %-20lld", stats.sleep_time);
    printf("Deep sleep time: %-2F0lld\n", stats.deep_sleep_time);
    }
#if 0
    {
    mbed_stats_thread_t *stats = new mbed_stats_thread_t[20];
    int count = mbed_stats_thread_get_each(stats, 20);
    
    for(int i = 0; i < count; i++) {
        printf("ID: 0x%x \n", stats[i].id);
        printf("Name: %s \n", stats[i].name);
        printf("State: %d \n", stats[i].state);
        printf("Priority: %d \n", stats[i].priority);
        printf("Stack Size: %d \n", stats[i].stack_size);
        printf("Stack Space: %d \n", stats[i].stack_space);
        printf("\n");
    }
    }
#endif
}


// main() runs in its own thread in the OS

static int dummy = printf("Starting all the things\r\n"); /// Strawman call to see if object initialization occurred.
int main()
{

    // Set up the LEDs
    led1.evt_queue = &background_queue;
    led2.evt_queue = &background_queue;
    led3.evt_queue = &background_queue;  

    oled_i2c.frequency(400000);
    oled_i2c.start();
    
    auto oled = new Adafruit_SSD1306_I2c(oled_i2c, PD_13, 0x78, 32, 128);

    oled->printf("Welcome to QMesh\r\n");
    oled->display();

    led1.LEDBlink();
    oled->printf("In bootloader...\r\n");
    oled->display();
	ThisThread::sleep_for(1000);
    //init_filesystem();

    
}

