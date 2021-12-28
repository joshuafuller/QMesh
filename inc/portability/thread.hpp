#ifndef THREAD_HPP
#define THREAD_HPP


#if defined(MBED_OS)
#include "mbed.h"
namespace portability {

using Thread = rtos::Thread;

} // namespace portability

#elif defined(ESP_IDF)
#include "pthread.h"
#include "esp_pthread.h"
namespace portability {

class Thread {
private:
    pthread_t *my_pthread;
public:

    

    ~Thread {
        my_pthread->join();
        delete my_pthread;
    }
};

} // namespace portability
#endif
//	Thread (osPriority priority=osPriorityNormal, uint32_t stack_size=OS_STACK_SIZE, unsigned char *stack_mem=nullptr, const char *name=nullptr)


#endif /* THREAD_HPP */