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