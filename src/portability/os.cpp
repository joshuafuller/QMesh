#include "os_portability.hpp"
#ifdef MBED_OS

void wait_us_duration(uint32_t duration_us);
void wait_us_duration(const uint32_t duration_us) {
    ::wait_us(duration_us);
}

namespace portability {

void special_init() { }

void sleep(const uint32_t duration_ms) {
    ThisThread::sleep_for(duration_ms);
}


#undef wait_us
void wait_us(const uint32_t duration_us) {
    wait_us_duration(duration_us);
}

}  // namespace portability

#elif ESP_IDF
#include "rom/ets_sys.h"
namespace portability {

void special_init(void) { 
    // Do any special initialization that the ESP-IDF needs
}


void sleep(const uint32_t duration_ms) {
    const TickType_t xDelay = duration_ms / portTICK_PERIOD_MS
    vTaskDelay(xDelay);
}


void wait_us(const uint32_t duration_us) {
    ets_delay_us(duration_us);
}

} // namespace portability
#else
#error Need to pick either MBED_OS or ESP_IDF
#endif