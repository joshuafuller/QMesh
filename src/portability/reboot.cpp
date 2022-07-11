/*
QMesh
Copyright (C) 2022 Daniel R. Fay

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "os_portability.hpp"
#include "serial_data.hpp"

volatile bool rebooting = false;

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