#ifndef TIMER_HPP
#define TIMER_HPP

#if defined(MBED_OS)
#include "mbed.h"
using Timer_portable = Timer;
using LowPowerTimer_portable = LowPowerTimer;

#elif defined(ESP_IDF)
#include "esp_timer.h"
class Timer_portable  {
private:
    int64_t start_time_us{-1};
    int64_t stop_time_us{-1};
    int64_t duration_time_us{0};
    bool running{false};

    auto get_cur_time_us(void) -> int64_t {
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    }

public:
    void start() {
        start_time_us = get_cur_time_us();
        running = true;
    }

    void stop() {
        stop_time_us = get_cur_time_us();
        duration_time_us += stop_time_us - start_time_us;
        running = false;
    }

    void reset() {
        start_time_us = get_cur_time_us();
        stop_time_us = -1;
        duration_time_us = 0;
        running = false;
    }

    auto read_us(void) -> int32_t {
        MBED_ASSERT(start_time_us >= 0);
        if(running) {
            return get_cur_time_us() - start_time_us + duration_time_us;
        else {
            return stop_time_us-start_time_us + duration_time_us;
        }
    }

    auto read_ms(void) -> int32_t {
        return read_us()/1000;
    }

    auto read_s(void) -> int32_t {
        return read_us()/1000000;
    }
};

using LowPowerTimer_portable = Timer_portable;

#endif

#endif /* TIMER_HPP */