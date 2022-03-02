#ifndef CAL_TIMER_HPP
#define CAL_TIMER_HPP

#include "os_portability.hpp"

void start_cal();

#define USE_LPTIMER

#ifndef USE_LPTIMER
class CalTimer : public portability::Timer {
    int32_t read_us(void); 

    int32_t read_ms(void); 

    int32_t read_s(void); 
};


class CalTimeout : public portability::Timeout_portable {
public:
	void attach_us(Callback< void()> func, us_timestamp_t t);
};

#else

class CalTimer : public portability::LowPowerTimer {};
class CalTimeout : public portability::LowPowerTimeout {};

#endif

#endif /* CAL_TIMER_HPP */