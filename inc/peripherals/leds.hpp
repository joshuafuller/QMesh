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

/**
 * Class that wraps the board's LEDs, and provides basic features
 * for turning LEDs on, off, and for blinking them at different 
 * rates.
 */
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
    int blink_period;

public:
    /// Function resonsible for blinking the LED.
    void blinkFn(void);
    /**
     * Constructor.
     * @param led_pin_name Pin that controls the LED.
     */
    IndicatorLED(PinName led_pin_name);
    /// Set the LED to solid
    void LEDSolid(void);
    /// Turn the LED off
    void LEDOff(void);
    /// Blink the LED
    void LEDBlink(void);
    /// Blink the LED quickly
    void LEDFastBlink(void);
    /// Destructor
    ~IndicatorLED();
};


extern IndicatorLED led1;
extern IndicatorLED led2;
extern IndicatorLED led3;

#endif /* LEDS_HPP */