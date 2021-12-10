#ifndef OS_PORTABILITY_HPP
#define OS_PORTABILITY_HPP

#include "mbed.h"
#include "SoftI2C.h"
#include <cstdint>

#define PORTABLE_ASSERT MBED_ASSERT

using mutex_portable = Mutex;
using timer_portable = Timer;
using ticker_portable = Ticker;
using I2C_portable = SoftI2C;
using Watchdog_portable = Watchdog;
using Thread_portable = Thread;
using DigitalIn_portable = DigitalIn;

void sleep_portable(uint32_t duration_ms);

#endif /* OS_PORTABILITY_HPP */