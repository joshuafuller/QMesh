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