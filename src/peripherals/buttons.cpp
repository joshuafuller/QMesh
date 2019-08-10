#include "mbed.h"
#include "buttons.hpp"

// Pin name for the button is USER_BUTTON
PushButton button(USER_BUTTON);

PushButton::PushButton(PinName button) {
    was_pressed = false;
    btn = new InterruptIn(button);
    btn->rise(callback(this, &PushButton::btnInterrupt));
}

void PushButton::btnInterrupt(void) {
    was_pressed = true;
}
    
bool PushButton::getPressed(void) {
    bool pressed_val = was_pressed;
    was_pressed = false;
    return pressed_val;
}

PushButton::~PushButton() {
    delete btn;
}