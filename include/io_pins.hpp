#ifndef IO_PINS_HPP
#define IO_PINS_HPP

#include "asserts.hpp"

#define ESP_IDF

#if defined(MBED_OS)
#include "mbed.h"
namespace portability {

using DigitalIn = mbed::DigitalIn;
using DigitalOut = mbed::DigitalOut;
using DigitalInOut = mbed::DigitalInOut;
using InterruptIn = mbed::InterruptIn;

} // namespace portability
#elif defined(ESP_IDF)
#include "Arduino.h"
namespace portability {

typedef enum PinMode_ENUM { 

} PinMode;

typedef enum PinDirection_ENUM {
    INPUT,
    OUTPUT,
    INOUT
} PinDirection;

class DigitalOut_portable {
private:
    int num;
    int val;
public:
    DigitalOut(const int pin_num, PinMode mode) { // Ignore modes
        num = pin_num;
        val = -1;
        pinMode(pin_num, OUTPUT);
    }
    DigitalOut() = delete;
    DigitalOut(const DigitalOut &obj) = delete;
    DigitalOut(const DigitalOut &&obj) = delete;
    auto operator=(const DigitalOut &obj) -> DigitalOut & = delete;
    auto operator=(const DigitalOut &&obj) -> DigitalOut & = delete;   
    operator int() const {
        return read();
    }

    auto read() -> int {
        return val;
    }

    void output() {
        return;
    }

    void input() {
        PORTABLE_ASSERT(false);
        return;
    }

    void write(int my_val) {
        digitalWrite(num, my_val);
        val = my_val;
    }

    void mode(PinMode mode) { }
};


class DigitalIn {
private:
    int num;
    int val;
public:
    DigitalIn(const int pin_num, PinMode mode) { // Ignore modes
        num = pin_num;
        pinMode(pin_num, OUTPUT);
    }
    DigitalIn() = delete;
    DigitalIn(const DigitalIn &obj) = delete;
    DigitalIn(const DigitalIn &&obj) = delete;
    auto operator=(const DigitalIn &obj) -> DigitalIn & = delete;
    auto operator=(const DigitalIn &&obj) -> DigitalIn & = delete;   
    operator int() const {
        return read();
    }

    auto read() -> int {
        digitalRead(pin_num);
    }

    void output() {
        PORTABLE_ASSERT(false);
        return;
    }

    void input() {
        return;
    }

    void write(int my_val) {
        PORTABLE_ASSERT(false);
    }

    void mode(PinMode mode) { }
};

} // namespace portability
#else
#error Need to define either MBED_OS or ESP_IDF
#endif

#endif /* IO_PINS_HPP */
