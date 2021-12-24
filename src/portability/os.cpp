#include "os_portability.hpp"
#ifdef MBED_OS
void special_init() { }

void sleep_portable(const uint32_t duration_ms) {
    ThisThread::sleep_for(duration_ms);
}


void wait_us_portable(const uint32_t duration_us) {
    wait_us(duration_us);
}

#elif ESP_IDF
#include "rom/ets_sys.h"

void special_init(void) { 
    // Do any special initialization that the ESP-IDF needs
}


void sleep_portable(const uint32_t duration_ms) {
    const TickType_t xDelay = duration_ms / portTICK_PERIOD_MS
    vTaskDelay(xDelay);
}


void wait_us_portable(const uint32_t duration_us) {
    ets_delay_us(duration_us);
}
#else
#error Need to pick either MBED_OS or ESP_IDF
#endif