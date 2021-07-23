#include "serial_msg.hpp"

static constexpr SerialMsg serialmsg_init_zero = SerialMsg_init_zero;
static constexpr SysCfgMsg syscfgmsg_init_zero = SysCfgMsg_init_zero;
static constexpr DbgMsg dbgmsg_init_zero = DbgMsg_init_zero;
static constexpr ErrorMsg errmsg_init_zero = ErrorMsg_init_zero;
static constexpr UpdateMsg updatemsg_init_zero = UpdateMsg_init_zero;
static constexpr VersionMsg versionmsg_init_zero = VersionMsg_init_zero;
SerMsg::SerMsg() : serial_msg(serialmsg_init_zero) { }

auto SerMsg::operator=(const SerialMsg &serialmsg) -> SerMsg& {
    serial_msg = serialmsg;
    return *this;
}

auto SerMsg::type() const -> SerialMsg_Type {
    return serial_msg.type;
}

void SerMsg::type(const SerialMsg_Type my_type) {
    serial_msg.type = my_type;
}

auto SerMsg::get_ser_msg() -> SerialMsg& {
    return serial_msg;
}

auto SerMsg::has_sys_cfg() const -> bool {
    return serial_msg.has_sys_cfg;
}

auto SerMsg::sys_cfg() -> SysCfgMsg& {
    if(!serial_msg.has_sys_cfg) {
        serial_msg.has_sys_cfg = true;
        serial_msg.sys_cfg = syscfgmsg_init_zero;            
    }
    return serial_msg.sys_cfg;
}

auto SerMsg::has_clock_set() const -> bool {
    return serial_msg.has_clock_set;
}

auto SerMsg::clock_set() -> ClockSetMsg& {
    if(!serial_msg.has_clock_set) {
        serial_msg.has_clock_set = true;
        serial_msg.clock_set = ClockSetMsg_init_zero;            
    }
    return serial_msg.clock_set;
}

auto SerMsg::has_status() const -> bool {
    return serial_msg.has_status;
}

auto SerMsg::status() -> StatusMsg& {
    if(!serial_msg.has_status) {
        serial_msg.has_status = true;
        serial_msg.status = StatusMsg_init_zero;            
    }
    return serial_msg.status;
}

auto SerMsg::has_dbg_msg() const -> bool {
    return serial_msg.has_dbg_msg;
}

auto SerMsg::dbg_msg() -> DbgMsg& {
    if(!serial_msg.has_dbg_msg) {
        serial_msg.has_dbg_msg = true;
        serial_msg.dbg_msg = dbgmsg_init_zero;            
    }
    return serial_msg.dbg_msg;
}

auto SerMsg::has_log_msg() const -> bool {
    return serial_msg.has_log_msg;
}

auto SerMsg::log_msg() -> LogMsg& {
    if(!serial_msg.has_log_msg) {
        serial_msg.has_log_msg = true;
        serial_msg.log_msg = LogMsg_init_zero;            
    }
    return serial_msg.log_msg;
}

auto SerMsg::has_boot_log_msg() const -> bool {
    return serial_msg.has_boot_log_msg;
}

auto SerMsg::boot_log_msg() -> BootLogMsg& {
    if(!serial_msg.has_boot_log_msg) {
        serial_msg.has_boot_log_msg = true;
        serial_msg.boot_log_msg = BootLogMsg_init_zero;            
    }
    return serial_msg.boot_log_msg;
}

auto SerMsg::has_data_msg() const -> bool {
    return serial_msg.has_data_msg;
}

auto SerMsg::data_msg() -> DataMsg& {
    if(!serial_msg.has_data_msg) {
        serial_msg.has_data_msg = true;
        serial_msg.data_msg = DataMsg_init_zero;            
    }
    return serial_msg.data_msg;
}

    auto SerMsg::has_error_msg() const -> bool {
        return serial_msg.has_error_msg;
    }

    auto SerMsg::error_msg() -> ErrorMsg& {
        if(!serial_msg.has_error_msg) {
            serial_msg.has_error_msg = true;
            serial_msg.error_msg = errmsg_init_zero;            
        }
        return serial_msg.error_msg;
    }

    auto SerMsg::has_time_msg() const -> bool {
        return serial_msg.has_time_msg;
    }

    auto SerMsg::time_msg() -> TimeMsg& {
        if(!serial_msg.has_time_msg) {
            serial_msg.has_time_msg = true;
            serial_msg.time_msg = TimeMsg_init_zero;            
        }
        return serial_msg.time_msg;
    }


    auto SerMsg::has_update_msg() const -> bool {
        return serial_msg.has_update_msg;
    }

    auto SerMsg::update_msg() -> UpdateMsg& {
        if(!serial_msg.has_update_msg) {
            serial_msg.has_update_msg = true;
            serial_msg.update_msg = updatemsg_init_zero;            
        }
        return serial_msg.update_msg;
    }

    auto SerMsg::has_ver_msg() const -> bool {
        return serial_msg.has_ver_msg;
    }

    auto SerMsg::ver_msg() -> VersionMsg& {
        if(!serial_msg.has_ver_msg) {
            serial_msg.has_ver_msg = true;
            serial_msg.ver_msg = versionmsg_init_zero;            
        }
        return serial_msg.ver_msg;
    }