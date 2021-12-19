#include "mbed.h"
#include <list>
#include <numeric>
#include <atomic>
#include "cal_timer.hpp"
#include "serial_data.hpp"


#ifndef USE_LPTIMER

#define BINS_PER_SEC (1.f/64.f)
#define NUM_SECS 256

static Timer_portable tmr;
static lpticker_portable lpticker;
extern EventQueue_portable *background_queue;
static atomic<float> tmr_factor, tmr_factor_inv;
static us_timestamp_t last_tmr_val = 0;
static volatile bool cal_running = false;

static void cal_handler(void);
static void cal_subhandler(const us_timestamp_t tmr_val);


void start_cal(void) {
    tmr_factor.store(1.0f);
    tmr.start();
    lpticker.attach_us(cal_handler, (float) 1e6/BINS_PER_SEC);
}


static void cal_handler(void) {
    CriticalSectionLock_portable lock;
    us_timestamp_t tmr_val = tmr.read_high_resolution_us();
    background_queue.call(cal_subhandler, tmr_val);
}
 

static void cal_subhandler(const us_timestamp_t tmr_val) {
    static list<us_timestamp_t> tmr_vals;
    tmr_vals.push_back(tmr_val-last_tmr_val);
    last_tmr_val = tmr_val;
    while(tmr_vals.size() > (BINS_PER_SEC*NUM_SECS)) {
        tmr_vals.pop_front();
    }
    float avg = (float) accumulate(tmr_vals.begin(), tmr_vals.end(), 0);
    avg /= (float) tmr_vals.size();
    float tmr_fact = (avg * ((float) BINS_PER_SEC*NUM_SECS)) / (1e6f*NUM_SECS);
    //debug_printf(DBG_INFO, "Timer factor is %f\r\n", tmr_fact);
    tmr_factor.store(tmr_fact);
    tmr_factor_inv.store(1.f/tmr_fact);
    cal_running = true;
}


int32_t CalTimer::read_us(void) {
    if(cal_running) {
        float tmr_fact = (float) Timer::read_us()*tmr_factor_inv.load();
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

#else
void start_cal() {
    
}
#endif