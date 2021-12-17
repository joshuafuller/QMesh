#ifndef LOCKING_HPP
#define LOCKING_HPP

#include "portability/asserts.hpp"

#if defined(MBED_OS)
#include "mbed.h"

using mutex_portable = Mutex;
using CriticalSectionLock_portable = CriticalSectionLock;

#elif defined(ESP_IDF)
#include "components/freertos/FreeRTOS-Kernel/include/freertos/semphr.h"

class mutex_portable {
private:
    semaphore_handle_t handle;
public:
    mutex_portable() {
        handle = xSemaphoreCreateMutex();
        PORTABLE_ASSERT(handle != nullptr);
    }

    ~mutex_portable() {
        xSemaphoreDelete(handle);
    }

    mutex_portable(const mutex_portable &obj) = delete;
    mutex_portable(const mutex_portable &&obj) = delete;
    auto operator=(const mutex_portable &obj) -> mutex_portable & = delete;
    auto operator=(const mutex_portable &&obj) -> mutex_portable & = delete;    

    void lock() {
        PORTABLE_ASSERT(xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE);
    }

    void unlock() {
        PORTABLE_ASSERT(xSemaphoreGive(handle) == pdTRUE);
    }
};


class CriticalSectionLock_portable {
private:
    static Mutex_portable *mtx;

public:
    CriticalSectionLock() {
        if(mtx == nullptr) {
            mtx = new Mutex_portable();
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

#endif /* LOCKING_HPP */