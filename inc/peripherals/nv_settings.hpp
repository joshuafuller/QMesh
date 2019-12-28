/*
QMesh
Copyright (C) 2019 Daniel R. Fay

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

 #ifndef NV_SETTINGS_HPP
 #define NV_SETTINGS_HPP

#include "mbed.h"
#include "params.hpp"
#include "peripherals.hpp"
#include "LittleFileSystem.h"
#include "SPIFBlockDevice.h"
#include "MbedJSONValue.h"

extern SPIFBlockDevice bd;
extern LittleFileSystem fs;

void nv_log_fn(void);

void log_boot(void);

void rescue_filesystem(void);

void init_filesystem(void);

void load_settings_from_flash(void);

void save_settings_to_flash(void);

#endif /* NV_SETTINGS_HPP */

