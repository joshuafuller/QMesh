#ifndef LOCKING_HPP
#define LOCKING_HPP

#include "portability/asserts.hpp"

namespace portability {

#if defined(MBED_OS)
#include "mbed.h"

using mutex = rtos::Mutex;
using CriticalSectionLock = mbed::CriticalSectionLock;

#elif defined(ESP_IDF)
#include "components/freertos/FreeRTOS-Kernel/include/freertos/semphr.h"

class mutex {
private:
    semaphore_handle_t handle;
public:
    mutex() {
        handle = xSemaphoreCreateMutex();
        PORTABLE_ASSERT(handle != nullptr);
    }

    ~mutex() {
        xSemaphoreDelete(handle);
    }

    mutex(const mutex &obj) = delete;
    mutex(const mutex &&obj) = delete;
    auto operator=(const mutex &obj) -> mutex & = delete;
    auto operator=(const mutex &&obj) -> mutex & = delete;    

    void lock() {
        PORTABLE_ASSERT(xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE);
    }

    void unlock() {
        PORTABLE_ASSERT(xSemaphoreGive(handle) == pdTRUE);
    }
};


class CriticalSectionLock {
private:
    static portability::mutex *mtx;

public:
    CriticalSectionLock() {
        if(mtx == nullptr) {
            mtx = new portability::mutex();
        }
        mtx->lock();
    }

    ~CriticalSectionLock() {
        mtx->unlock();
    }
};

#else
#error Need to define either MBED_OS or ESP_IDF
#endif 

}  // namespace portability

#endif /* LOCKING_HPP */