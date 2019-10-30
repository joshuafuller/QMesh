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
    float blink_period;

public:
    void blinkFn(void);
    IndicatorLED(PinName led_pin_name);
    void LEDSolid(void);
    void LEDOff(void);
    void LEDBlink(void);
    void LEDFastBlink(void);
    ~IndicatorLED();
};


extern IndicatorLED led1;
extern IndicatorLED led2;
extern IndicatorLED led3;

#endif /* LEDS_HPP */