#ifndef CAL_TIMER_HPP
#define CAL_TIMER_HPP

#include "mbed.h"

void start_cal(void);

class CalTimer : public Timer {
    int32_t read_us(void); 

    int32_t read_ms(void); 

    int32_t read_s(void); 
};

#endif /* CAL_TIMER_HPP */