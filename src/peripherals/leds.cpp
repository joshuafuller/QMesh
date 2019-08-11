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

#include "leds.hpp"


IndicatorLED led1(LED1);
IndicatorLED led2(LED2);
IndicatorLED led3(LED3);


void IndicatorLED::blinkFn(void) {
    while(true) {
        if(led_state == LED_BLINK) {
            *pin = !*pin;
        }
        wait(0.5);
    }
}

IndicatorLED::IndicatorLED(PinName led_pin_name) {
    led_state = LED_OFF;
    blink_led = false;
    pin = new DigitalOut(led_pin_name);
    *pin = 0;
    thread.start(callback(this, &IndicatorLED::blinkFn));
}

void IndicatorLED::LEDSolid(void) {
    led_state = LED_SOLID;
    blink_led = false;
    *pin = 1;
}

void IndicatorLED::LEDOff(void) {
    led_state = LED_OFF;
    blink_led = false;
    *pin = 0;
}

void IndicatorLED::LEDBlink(void) {
    led_state = LED_BLINK;
    blink_led = true;
}

IndicatorLED::~IndicatorLED() {
    delete pin;
}