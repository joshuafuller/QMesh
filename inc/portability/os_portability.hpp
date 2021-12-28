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

namespace portability {
#if defined(MBED_OS)
using Ticker = mbed::Ticker;
using LowPowerTicker = mbed::LowPowerTicker;
using LowPowerTimer = mbed::LowPowerTimer;
using LowPowerTimeout = mbed::LowPowerTimeout;
using Watchdog = mbed::Watchdog;
using EventQueue = EventQueue;

void special_init();

template <typename T, int queue_len> 
class Mail
{
private:
    rtos::Mail<T, queue_len> *my_mail;

public:
    Mail() {
        my_mail = new rtos::Mail<T, queue_len>();
    }

    ~Mail() {
        delete my_mail;
    }    

    Mail(const Mail &obj) = delete;
    Mail(const Mail &&obj) = delete;
    auto operator=(const Mail &obj) -> Mail & = delete;
    auto operator=(const Mail &&obj) -> Mail & = delete;    

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

void sleep(uint32_t duration_ms);
#undef wait_us
void wait_us(uint32_t duration_us);

}  // namespace portability

#endif /* OS_PORTABILITY_HPP */