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

#include "leds.hpp"

IndicatorLED *led1;
IndicatorLED *led2;
IndicatorLED *led3;

static constexpr int QUARTER_SECOND = 250;
static constexpr int TWENTIETH_SECOND = 50;

void create_led_objects() {
    led1 = new IndicatorLED(LED_1);
    led2 = new IndicatorLED(LED_2);
    led3 = new IndicatorLED(LED_3);   
}

void IndicatorLED::blinkFn() {
    if(led_state == LED_BLINK) {
        *pin = *pin == 0 ? 1 : 0;
        evt_queue->call_in(blink_period, callback(this, &IndicatorLED::blinkFn));
    } else {
        *pin = 0;
    }
}

IndicatorLED::IndicatorLED(PinName led_pin_name) {
    evt_queue = nullptr;
    led_state = LED_OFF;
    blink_led = false;
    blink_period = QUARTER_SECOND;
    pin = new DigitalOut_portable(led_pin_name);
    *pin = 0;
}

void IndicatorLED::LEDSolid() {
    led_state = LED_SOLID;
    blink_led = false;
    *pin = 1;
}

void IndicatorLED::LEDOff() {
    led_state = LED_OFF;
    blink_led = false;
    *pin = 0;
}   

void IndicatorLED::LEDBlink() {
    blink_period = QUARTER_SECOND;
    blink_led = true;
    if(led_state != LED_BLINK) {
        led_state = LED_BLINK;
        evt_queue->call_in(blink_period, callback(this, &IndicatorLED::blinkFn));
    }
}

void IndicatorLED::LEDFastBlink() {
    blink_period = TWENTIETH_SECOND;
    blink_led = true;
    if(led_state != LED_BLINK) {
        led_state = LED_BLINK;
        evt_queue->call_in(blink_period, callback(this, &IndicatorLED::blinkFn));
    }
}

IndicatorLED::~IndicatorLED() {
    delete pin;
}