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

#ifndef LEDS_HPP
#define LEDS_HPP

#include "mbed.h"

/**
 * Class that wraps the board's LEDs, and provides basic features
 * for turning LEDs on, off, and for blinking them at different 
 * rates.
 */
class IndicatorLED {
private:    
    enum {
        LED_OFF,
        LED_SOLID,
        LED_BLINK
    } led_state;
    DigitalOut *pin;
    bool blink_led;
    int blink_period;
    EventQueue *evt_queue;

public:
    /// Function resonsible for blinking the LED.
    void blinkFn();
    /**
     * Constructor.
     * @param led_pin_name Pin that controls the LED.
     */
    explicit IndicatorLED(PinName led_pin_name);

    /// Since you can't really share pins, the copy constructor and
    /// copy assignment operators don't make any sense.
    IndicatorLED(const IndicatorLED &old) = delete;
    auto operator= (const IndicatorLED &) -> IndicatorLED & = delete;

    /// Move constructor and move assignment operators do make sense, however.
    auto operator= (IndicatorLED &&) -> IndicatorLED & = default;	
    IndicatorLED(IndicatorLED&& other) = default;

    /// Set the event queue
    void setEvtQueue(EventQueue *q) {
        evt_queue = q;
    }
    /// Set the LED to solid
    void LEDSolid();
    /// Turn the LED off
    void LEDOff();
    /// Blink the LED
    void LEDBlink();
    /// Blink the LED quickly
    void LEDFastBlink();
    /// Destructor
    ~IndicatorLED();
};


extern IndicatorLED *led1;
extern IndicatorLED *led2;
extern IndicatorLED *led3;

#endif /* LEDS_HPP */