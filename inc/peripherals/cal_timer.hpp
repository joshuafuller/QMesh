#ifndef CAL_TIMER_HPP
#define CAL_TIMER_HPP

#include "os_portability.hpp"

void start_cal();

#define USE_LPTIMER

#ifndef USE_LPTIMER
class CalTimer : public Timer_portable {
    int32_t read_us(void); 

    int32_t read_ms(void); 

    int32_t read_s(void); 
};


class CalTimeout : public Timeout_portable {
public:
	void attach_us(Callback< void()> func, us_timestamp_t t);
};

#else

class CalTimer : public LowPowerTimer_portable {};
class CalTimeout : public LowPowerTimeout_portable {};

#endif

#endif /* CAL_TIMER_HPP */