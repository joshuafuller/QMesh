#ifndef CAL_TIMER_HPP
#define CAL_TIMER_HPP

#include "mbed.h"

void start_cal();

#define USE_LPTIMER

#ifndef USE_LPTIMER
class CalTimer : public Timer {
    int32_t read_us(void); 

    int32_t read_ms(void); 

    int32_t read_s(void); 
};


class CalTimeout : public Timeout {
public:
	void attach_us(Callback< void()> func, us_timestamp_t t);
};

#else

class CalTimer : public LowPowerTimer {};
class CalTimeout : public LowPowerTimeout {};

#endif

#endif /* CAL_TIMER_HPP */