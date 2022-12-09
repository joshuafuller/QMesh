#include "os_portability.hpp"
#include "mem_trace.hpp"
#include "serial_data.hpp"

void print_memory_info() {
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    auto *stats = static_cast<mbed_stats_stack_t*>(malloc(cnt * sizeof(mbed_stats_stack_t))); //NOLINT
    PORTABLE_ASSERT(stats != nullptr);
 
    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", static_cast<unsigned long>(stats[i].thread_id), //NOLINT
            static_cast<unsigned long>(stats[i].max_size), static_cast<unsigned long>(stats[i].reserved_size)); //NOLINT
    }
    free(stats); //NOLINT

    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes\r\n", static_cast<unsigned long>(heap_stats.current_size), 
        static_cast<unsigned long>(heap_stats.reserved_size));
}


//extern portability::EventQueue background_queue;

static atomic<size_t> max_stack_usage;
static atomic<size_t> max_heap_usage;
static void track_memory_info();
static void track_memory_info() {
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    auto stats = vector<mbed_stats_stack_t>(cnt);
    cnt = mbed_stats_stack_get_each(stats.data(), cnt);
    size_t stack_usage = 0;
    for (int i = 0; i < cnt; i++) {
        stack_usage += stats[i].max_size;
    }
    if(stack_usage > max_stack_usage) {
        max_stack_usage = stack_usage;
    }
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    if(heap_stats.current_size > max_heap_usage) {
        max_heap_usage = heap_stats.current_size;
    }
}


static void print_max_memory_usage();
static void print_max_memory_usage() {
    debug_printf(DBG_INFO, "Current total memory usage is %d\r\n", max_stack_usage+max_heap_usage);
}


auto get_max_memory_usage() -> uint32_t {
    return max_stack_usage+max_heap_usage;
}


void start_max_memory_usage() {
    max_stack_usage = 0;
    max_heap_usage = 0;
    static constexpr int TENTH_SECOND = 100;
    static constexpr int TEN_SECONDS = 10000;
    //#warning FIX THIS
    //background_queue.call_every(TENTH_SECOND, track_memory_info);
    //background_queue.call_every(TEN_SECONDS, print_max_memory_usage);
}