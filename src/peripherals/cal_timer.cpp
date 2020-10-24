#include "mbed.h"
#include <list>
#include <numeric>
#include <atomic>
#include "cal_timer.hpp"

#define BINS_PER_SEC 1

static Timer tmr;
static LowPowerTicker lpticker;
extern EventQueue background_queue;
static atomic<float> tmr_factor;
static us_timestamp_t last_tmr_val = 0;
static volatile bool cal_running = false;

static void cal_handler(void);
static void cal_subhandler(const us_timestamp_t tmr_val);


void start_cal(void) {
    tmr_factor.store(1.0f);
    tmr.start();
    lpticker.attach_us(cal_handler, 1e6/BINS_PER_SEC);
}


static void cal_handler(void) {
    CriticalSectionLock lock;
    us_timestamp_t tmr_val = tmr.read_high_resolution_us();
    background_queue.call(cal_subhandler, tmr_val);
}
 

static void cal_subhandler(const us_timestamp_t tmr_val) {
    static list<us_timestamp_t> tmr_vals;
    tmr_vals.push_back(tmr_val-last_tmr_val);
    last_tmr_val = tmr_val;
    while(tmr_vals.size() > BINS_PER_SEC) {
        tmr_vals.pop_front();
    }
    float avg = (float) accumulate(tmr_vals.begin(), tmr_vals.end(), 0);
    avg /= (float) tmr_vals.size();
    float tmr_fact = (avg * (float) BINS_PER_SEC) / 1e6f;
    tmr_factor.store(tmr_fact);
    cal_running = true;
}


int32_t CalTimer::read_us(void) {
    if(cal_running) {
        float tmr_fact = (float) Timer::read_us()/tmr_factor.load();
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


void CalTimeout::attach_us(Callback< void()> func, us_timestamp_t t) {
    us_timestamp_t cal_t = tmr_factor.load()*(float) t;
    Timeout::attach_us(func, cal_t);
}