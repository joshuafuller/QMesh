#ifndef FW_UPDATE_HPP
#define FW_UPDATE_HPP

#include <string>

auto check_for_update(const std::string &fname) -> bool;

auto update_firmware() -> bool;

void apply_update(const std::string &fname, bool golden);

auto start_fs() -> bool;


#endif /* FW_UPDATE_HPP */