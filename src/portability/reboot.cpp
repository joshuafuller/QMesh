#include "os_portability.hpp"
#include "serial_data.hpp"

volatile bool rebooting = false;
static constexpr int HALF_SECOND = 500;

#if defined(MBED_OS)
void reboot_system() {
	rebooting = true;
    debug_printf(DBG_INFO, "Now rebooting the system...\r\n");
    portability::sleep(HALF_SECOND);
    NVIC_SystemReset();
}


#elif defined(ESP_IDF)
#include "components/esp_system/include/esp_system.h"
void reboot_system() {
	rebooting = true;
    debug_printf(DBG_INFO, "Now rebooting the system...\r\n");
    portability::sleep(HALF_SECOND);
    esp_restart();
}


#else
#error Need to define either MBED_OS or ESP_IDF

#endif