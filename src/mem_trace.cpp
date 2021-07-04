#include "mbed.h"
#include "mem_trace.hpp"
#include "serial_data.hpp"

void print_memory_info() {
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    auto *stats = static_cast<mbed_stats_stack_t*>(malloc(cnt * sizeof(mbed_stats_stack_t)));
    MBED_ASSERT(stats != nullptr);
 
    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", static_cast<unsigned long>(stats[i].thread_id), 
            static_cast<unsigned long>(stats[i].max_size), static_cast<unsigned long>(stats[i].reserved_size));
    }
    free(stats);

    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes\r\n", static_cast<unsigned long>(heap_stats.current_size), 
        static_cast<unsigned long>(heap_stats.reserved_size));
}