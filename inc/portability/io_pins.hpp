#ifndef IO_PINS_HPP
#define IO_PINS_HPP

#include "portability/asserts.hpp"

#define ESP_IDF

#if defined(MBED_OS)
#include "mbed.h"
using DigitalIn_portable = DigitalIn;
using DigitalOut_portable = DigitalOut;
using DigitalInOut_portable = DigitalInOut;
using InterruptIn_portable = InterruptIn;

#elif defined(ESP_IDF)
#include "Arduino.h"
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
    DigitalOut_portable(const int pin_num, PinMode mode) { // Ignore modes
        num = pin_num;
        val = -1;
        pinMode(pin_num, OUTPUT);
    }
    DigitalOut_portable() = delete;
    DigitalOut_portable(const DigitalOut_portable &obj) = delete;
    DigitalOut_portable(const DigitalOut_portable &&obj) = delete;
    auto operator=(const DigitalOut_portable &obj) -> DigitalOut_portable & = delete;
    auto operator=(const DigitalOut_portable &&obj) -> DigitalOut_portable & = delete;   
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


class DigitalIn_portable {
private:
    int num;
    int val;
public:
    DigitalIn_portable(const int pin_num, PinMode mode) { // Ignore modes
        num = pin_num;
        pinMode(pin_num, OUTPUT);
    }
    DigitalIn_portable() = delete;
    DigitalIn_portable(const DigitalIn_portable &obj) = delete;
    DigitalIn_portable(const DigitalIn_portable &&obj) = delete;
    auto operator=(const DigitalIn_portable &obj) -> DigitalIn_portable & = delete;
    auto operator=(const DigitalIn_portable &&obj) -> DigitalIn_portable & = delete;   
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
#else
#error Need to define either MBED_OS or ESP_IDF
#endif

#endif /* IO_PINS_HPP */