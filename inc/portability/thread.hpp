#ifndef THREAD_HPP
#define THREAD_HPP

#if defined(MBED_OS)
#include "mbed.h"
using Thread_portable = rtos::Thread;


#elif defined(ESP_IDF)
#include "pthread.h"
#include "esp_pthread.h"

class Thread_portable {
private:
    pthread_t *my_pthread;
public:

    ~Thread_portable {
        my_pthread->join();
        delete my_pthread;
    }
};

#endif
//	Thread (osPriority priority=osPriorityNormal, uint32_t stack_size=OS_STACK_SIZE, unsigned char *stack_mem=nullptr, const char *name=nullptr)



#endif /* THREAD_HPP */