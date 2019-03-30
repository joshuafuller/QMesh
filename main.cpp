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
#include "lora_radio_helper.h"
#include "radio.hpp"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
Thread led2_thread;
Thread led3_thread;
Thread radio_thread;

void led2_thread_fn() {
    while (true) {
        led2 = !led2;
        wait(0.333);
    }
}

void led3_thread_fn() {
    while (true) {
        led3 = !led3;
        wait(0.1);
    }
}

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

// main() runs in its own thread in the OS
int main()
{
    // Set up the radio
    init_radio();

    // Start a thread for blinking LEDs
    led2_thread.start(led2_thread_fn);
    led3_thread.start(led3_thread_fn);

    // Start a thread for the radio
    radio_thread.start(test_radio);

    int count = 0;
    while (true) {
        // Blink LED and wait 0.5 seconds
        led1 = !led1;
        wait_ms(SLEEP_TIME);
    }
}

