#ifndef OS_PORTABILITY_HPP
#define OS_PORTABILITY_HPP

#include "mbed.h"
#include <cstdint>
#include "asserts.hpp"
#include "Callback.hpp"
#include "locking.hpp"
#include "io_pins.hpp"
#include "pins.hpp"
#include "spi.hpp"
#include "i2c.hpp"
#include "timer.hpp"
#include "thread.hpp"


#if defined(MBED_OS)
using Ticker_portable = Ticker;
using LowPowerTicker_portable = LowPowerTicker;
using LowPowerTimer_portable = LowPowerTimer;
using LowPowerTimeout_portable = LowPowerTimeout;
using Watchdog_portable = Watchdog;
using EventQueue_portable = EventQueue;

void special_init();

template <typename T, int queue_len> 
class Mail_portable
{
private:
    Mail<T, queue_len> *my_mail;

public:
    Mail_portable() {
        my_mail = new Mail<T, queue_len>();
    }

    ~Mail_portable() {
        delete my_mail;
    }    

    Mail_portable(const Mail_portable &obj) = delete;
    Mail_portable(const Mail_portable &&obj) = delete;
    auto operator=(const Mail_portable &obj) -> Mail_portable & = delete;
    auto operator=(const Mail_portable &&obj) -> Mail_portable & = delete;    

    auto full() -> bool {
        return my_mail->full();
    }

    auto empty() -> bool {
        return my_mail->empty();
    }

    auto alloc_for(uint32_t millisec = 0) -> T * {
        return my_mail->alloc_for(millisec);
    }

    auto alloc(uint32_t millisec = 0) -> T * {
        return my_mail->alloc(millisec);
    }

    auto get(uint32_t millisec = osWaitForever) -> osEvent {
        return my_mail->get(millisec);
    }

    auto put(T *mptr) -> osStatus {
        return my_mail->put(mptr);
    }

    auto free(T *mptr) -> osStatus {
        return my_mail->free(mptr);
    }
};
#elif defined(ESP_IDF)

#else
#error Need to define either MBED_OS or ESP_IDF
#endif 

void sleep_portable(uint32_t duration_ms);
void wait_us_portable(uint32_t duration_us);

#endif /* OS_PORTABILITY_HPP */