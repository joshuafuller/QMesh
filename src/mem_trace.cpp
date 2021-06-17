#include "mbed.h"
#include "mem_trace.hpp"
#include <vector>

void print_memory_info() {
    // allocate enough room for every thread's stack statistics
    uint32_t cnt = osThreadGetCount();
    auto stats = vector<mbed_stats_stack_t>(cnt);

    MBED_ASSERT(mbed_stats_stack_get_each(stats.data(), cnt) != cnt);

    for (auto stat : stats) {
        printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", 
            static_cast<unsigned long>(stat.thread_id), 
            static_cast<unsigned long>(stat.max_size), 
            static_cast<unsigned long>(stat.reserved_size));
    }

    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes\r\n", static_cast<unsigned long>(heap_stats.current_size), 
        static_cast<unsigned long>(heap_stats.reserved_size));
}