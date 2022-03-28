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

#ifndef SERIAL_MSG_HPP
#define SERIAL_MSG_HPP

#ifndef TEST_FEC
#include "qmesh.pb.h"
#endif /* TEST_FEC */
#include <memory>

template <typename T> 
void check_and_init_assign(T **mine, T *theirs) {
    delete *mine;
    if(theirs != nullptr) {
        *mine = new T();
        **mine = *theirs;
    } 
}

template <typename T> 
void check_and_init(T **mine, const T &zero_val) {
    if(*mine == nullptr) {
        *mine = new T();
        **mine = zero_val;
    } 
}

class SerMsg {
private:
    SerialMsg serial_msg;
public:
    SerMsg();
    SerMsg(const SerMsg &serialmsg);
    auto operator=(const SerMsg &serialmsg) -> SerMsg&;
    ~SerMsg();
    SerMsg(SerMsg &&) = delete;
    auto operator=(SerMsg &&) -> SerMsg& = delete;

    auto size() const -> size_t;
    void clear();
    static auto maxSize() -> size_t;
    auto type() const -> SerialMsg_Type;
    auto retry() const -> bool;
    void type(SerialMsg_Type my_type);
    auto get_ser_msg() -> SerialMsg&;
    auto has_sys_cfg() const -> bool;
    auto sys_cfg() -> SysCfgMsg&;
    auto has_clock_set() const -> bool;
    auto clock_set() -> ClockSetMsg&;
    auto has_status() const -> bool;
    auto status() -> StatusMsg&;
    auto has_dbg_msg() const -> bool;
    auto dbg_msg() -> DbgMsg&;
    auto has_log_msg() const -> bool;
    auto log_msg() -> LogMsg&;
    auto has_boot_log_msg() const -> bool;
    auto boot_log_msg() -> BootLogMsg&;
    auto has_data_msg() const -> bool;
    auto data_msg() -> DataMsg&;
    auto has_error_msg() const -> bool;
    auto error_msg() -> ErrorMsg&;
    auto has_time_msg() const -> bool;
    auto time_msg() -> TimeMsg&;
    auto has_update_msg() const -> bool;
    auto update_msg() -> UpdateMsg&;
    auto has_ver_msg() const -> bool;
    auto ver_msg() -> VersionMsg&;
    auto has_int_params_msg() const -> bool;
    auto int_params_msg() -> IntParamsMsg&;
    auto has_voice_frame_msg() const -> bool;
    auto voice_frame_msg() -> VoiceFrameMsg&;
    auto has_ack_msg() const -> bool;
    auto ack_msg() -> AckMsg&;   
};

#endif /* SERIAL_MSG_HPP */