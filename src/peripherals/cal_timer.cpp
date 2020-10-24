#include "mbed.h"
#include <list>
#include <numeric>
#include <atomic>
#include "cal_timer.hpp"

#define BINS_PER_SEC 16

static Timer tmr;
static LowPowerTicker lpticker;
extern EventQueue background_queue;
atomic<float> tmr_factor;
static volatile bool cal_running = false;

static void cal_handler(void);
static void cal_subhandler(const int tmr_val);


void start_cal(void) {
    tmr_factor.store(1.0f);
    lpticker.attach_us(cal_handler, 1e6/BINS_PER_SEC);
    cal_running = true;
}


static void cal_handler(void) {
    CriticalSectionLock lock;
    int tmr_val = tmr.read_us();
    background_queue.call(cal_subhandler, tmr_val);
}
 
static void cal_subhandler(const int tmr_val) {
    static list<int> tmr_vals;
    tmr_vals.push_back(tmr_val);
    while(tmr_vals.size() > BINS_PER_SEC) {
        tmr_vals.pop_front();
    }
    float avg = (float) accumulate(tmr_vals.begin(), tmr_vals.end(), 0);
    avg /= (float) tmr_vals.size();
    float tmr_fact = avg / (1e6f * (float) BINS_PER_SEC);
    tmr_factor.store(tmr_fact);
}


int32_t CalTimer::read_us(void) {
    if(cal_running) {
        float tmr_fact = tmr_factor.load()*Timer::read_us();
        return (int32_t) tmr_fact;
    }
    else {
        return Timer::read_us();
    }
}

int32_t CalTimer::read_ms(void) {
    return CalTimer::read_us()/1000;
}

int32_t CalTimer::read_s(void) {
    return CalTimer::read_us()/1e6;
}