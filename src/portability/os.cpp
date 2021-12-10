#include "os_portability.hpp"

void sleep_portable(const uint32_t duration_ms) {
    ThisThread::sleep_for(duration_ms);
}