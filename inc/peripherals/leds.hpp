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

#ifndef LEDS_HPP
#define LEDS_HPP

#include "mbed.h"

class IndicatorLED {
protected:    
    enum {
        LED_OFF,
        LED_SOLID,
        LED_BLINK
    } led_state;
    DigitalOut *pin;
    Thread thread;
    bool blink_led;

public:
    void blinkFn(void);
    IndicatorLED(PinName led_pin_name);
    void LEDSolid(void);
    void LEDOff(void);
    void LEDBlink(void);
    ~IndicatorLED();
};


extern IndicatorLED led1;
extern IndicatorLED led2;
extern IndicatorLED led3;

#endif /* LEDS_HPP */