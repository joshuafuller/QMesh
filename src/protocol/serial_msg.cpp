#include "serial_msg.hpp"

static constexpr SerialMsg serialmsg_init_zero = SerialMsg_init_zero;
static constexpr SysCfgMsg syscfgmsg_init_zero = SysCfgMsg_init_zero;
static constexpr DbgMsg dbgmsg_init_zero = DbgMsg_init_zero;
static constexpr ErrorMsg errmsg_init_zero = ErrorMsg_init_zero;
static constexpr UpdateMsg updatemsg_init_zero = UpdateMsg_init_zero;
static constexpr VersionMsg versionmsg_init_zero = VersionMsg_init_zero;
static constexpr ClockSetMsg clocksetmsg_init_zero = ClockSetMsg_init_zero;
static constexpr StatusMsg statusmsg_init_zero = StatusMsg_init_zero;
static constexpr LogMsg logmsg_init_zero = LogMsg_init_zero;
static constexpr BootLogMsg bootlogmsg_init_zero = BootLogMsg_init_zero;
static constexpr DataMsg datamsg_init_zero = DataMsg_init_zero;
static constexpr TimeMsg timemsg_init_zero = TimeMsg_init_zero;
static constexpr IntParamsMsg intparamsmsg_init_zero = IntParamsMsg_init_zero;
static constexpr VoiceFrameMsg voiceframemsg_init_zero = VoiceFrameMsg_init_zero;
static constexpr AckMsg ackmsg_init_zero = AckMsg_init_zero;

SerMsg::SerMsg() : serial_msg(serialmsg_init_zero) { }

SerMsg::SerMsg(const SerMsg &serialmsg) : serial_msg(serialmsg_init_zero) {
    serial_msg.type = serialmsg.type();
    serial_msg.retry = serialmsg.retry();
    check_and_init_assign(&(serial_msg.boot_log_msg), serialmsg.serial_msg.boot_log_msg);
    check_and_init_assign(&(serial_msg.clock_set), serialmsg.serial_msg.clock_set);
    check_and_init_assign(&(serial_msg.sys_cfg), serialmsg.serial_msg.sys_cfg);
    check_and_init_assign(&(serial_msg.dbg_msg), serialmsg.serial_msg.dbg_msg);
    check_and_init_assign(&(serial_msg.error_msg), serialmsg.serial_msg.error_msg);
    check_and_init_assign(&(serial_msg.update_msg), serialmsg.serial_msg.update_msg);
    check_and_init_assign(&(serial_msg.ver_msg), serialmsg.serial_msg.ver_msg);
    check_and_init_assign(&(serial_msg.status), serialmsg.serial_msg.status);
    check_and_init_assign(&(serial_msg.log_msg), serialmsg.serial_msg.log_msg);
    check_and_init_assign(&(serial_msg.data_msg), serialmsg.serial_msg.data_msg);
    check_and_init_assign(&(serial_msg.time_msg), serialmsg.serial_msg.time_msg);
    check_and_init_assign(&(serial_msg.int_params_msg), serialmsg.serial_msg.int_params_msg);    
    check_and_init_assign(&(serial_msg.voice_frame_msg), serialmsg.serial_msg.voice_frame_msg);    
    check_and_init_assign(&(serial_msg.ack_msg), serialmsg.serial_msg.ack_msg); 
}

auto SerMsg::operator=(const SerMsg &serialmsg) -> SerMsg & {
    if(this != &serialmsg) {
        serial_msg.type = serialmsg.type();
        serial_msg.retry = serialmsg.retry();
        check_and_init_assign(&(serial_msg.boot_log_msg), serialmsg.serial_msg.boot_log_msg);
        check_and_init_assign(&(serial_msg.clock_set), serialmsg.serial_msg.clock_set);
        check_and_init_assign(&(serial_msg.sys_cfg), serialmsg.serial_msg.sys_cfg);
        check_and_init_assign(&(serial_msg.dbg_msg), serialmsg.serial_msg.dbg_msg);
        check_and_init_assign(&(serial_msg.error_msg), serialmsg.serial_msg.error_msg);
        check_and_init_assign(&(serial_msg.update_msg), serialmsg.serial_msg.update_msg);
        check_and_init_assign(&(serial_msg.ver_msg), serialmsg.serial_msg.ver_msg);
        check_and_init_assign(&(serial_msg.status), serialmsg.serial_msg.status);
        check_and_init_assign(&(serial_msg.log_msg), serialmsg.serial_msg.log_msg);
        check_and_init_assign(&(serial_msg.data_msg), serialmsg.serial_msg.data_msg);
        check_and_init_assign(&(serial_msg.time_msg), serialmsg.serial_msg.time_msg);
        check_and_init_assign(&(serial_msg.int_params_msg), serialmsg.serial_msg.int_params_msg);
        check_and_init_assign(&(serial_msg.voice_frame_msg), serialmsg.serial_msg.voice_frame_msg);  
        check_and_init_assign(&(serial_msg.ack_msg), serialmsg.serial_msg.ack_msg);           
    }
    return *this; 
}

void SerMsg::clear() {
    delete serial_msg.sys_cfg;
    delete serial_msg.dbg_msg;
    delete serial_msg.error_msg;
    delete serial_msg.update_msg;
    delete serial_msg.ver_msg;
    delete serial_msg.clock_set;
    delete serial_msg.status;
    delete serial_msg.log_msg;
    delete serial_msg.boot_log_msg;
    delete serial_msg.data_msg;
    delete serial_msg.time_msg;
    delete serial_msg.int_params_msg;
    delete serial_msg.voice_frame_msg;
    delete serial_msg.ack_msg;
    serial_msg = serialmsg_init_zero;
}

SerMsg::~SerMsg() {
    delete serial_msg.sys_cfg;
    delete serial_msg.dbg_msg;
    delete serial_msg.error_msg;
    delete serial_msg.update_msg;
    delete serial_msg.ver_msg;
    delete serial_msg.clock_set;
    delete serial_msg.status;
    delete serial_msg.log_msg;
    delete serial_msg.boot_log_msg;
    delete serial_msg.data_msg;
    delete serial_msg.time_msg;
    delete serial_msg.int_params_msg;
    delete serial_msg.voice_frame_msg;
    delete serial_msg.ack_msg;
}

auto SerMsg::size() const -> size_t {
    size_t acc = 0;
    acc += sizeof(SerialMsg);
    if(serial_msg.sys_cfg != nullptr) {
        acc += SysCfgMsg_size;
    }
    if(serial_msg.dbg_msg != nullptr) {
        acc += DbgMsg_size;
    }
    if(serial_msg.error_msg != nullptr) {
        acc += ErrorMsg_size;
    }
    if(serial_msg.update_msg != nullptr) {
        acc += UpdateMsg_size;
    }
    if(serial_msg.ver_msg != nullptr) {
        acc += VersionMsg_size;
    }
    if(serial_msg.clock_set != nullptr) {
        acc += ClockSetMsg_size;
    }
    if(serial_msg.status != nullptr) {
        acc += StatusMsg_size;
    }
    if(serial_msg.log_msg != nullptr) {
        acc += LogMsg_size;
    }
    if(serial_msg.boot_log_msg != nullptr) {
        acc += BootLogMsg_size;
    }
    if(serial_msg.data_msg != nullptr) {
        acc += DataMsg_size;
    }
    if(serial_msg.time_msg != nullptr) {
        acc += TimeMsg_size;
    }
    if(serial_msg.int_params_msg != nullptr) {
        acc += IntParamsMsg_size;
    }
    if(serial_msg.voice_frame_msg != nullptr) {
        acc += VoiceFrameMsg_size;
    }
    if(serial_msg.ack_msg != nullptr) {
        acc += AckMsg_size;
    }
    return acc;
}

auto SerMsg::maxSize() -> size_t {
    size_t acc = 0;
    acc += sizeof(SerialMsg);
    acc += SysCfgMsg_size;
    acc += DbgMsg_size;
    acc += ErrorMsg_size;
    acc += UpdateMsg_size;
    acc += VersionMsg_size;
    acc += ClockSetMsg_size;
    acc += StatusMsg_size;
    acc += LogMsg_size;
    acc += BootLogMsg_size;
    acc += DataMsg_size;
    acc += TimeMsg_size;
    acc += IntParamsMsg_size;
    acc += VoiceFrameMsg_size;
    acc += AckMsg_size;
    return acc;
}

auto SerMsg::type() const -> SerialMsg_Type {
    return serial_msg.type;
}

auto SerMsg::retry() const -> bool {
    return serial_msg.retry;
}

void SerMsg::type(const SerialMsg_Type my_type) {
    serial_msg.type = my_type;
}

auto SerMsg::get_ser_msg() -> SerialMsg& {
    return serial_msg;
}

auto SerMsg::has_sys_cfg() const -> bool {
    return !(serial_msg.sys_cfg == nullptr);
}

auto SerMsg::sys_cfg() -> SysCfgMsg& {
    check_and_init(&(serial_msg.sys_cfg), syscfgmsg_init_zero);
    return *(serial_msg.sys_cfg);
}

auto SerMsg::has_clock_set() const -> bool {
    return !(serial_msg.clock_set == nullptr);
}

auto SerMsg::clock_set() -> ClockSetMsg& {
    check_and_init(&(serial_msg.clock_set), clocksetmsg_init_zero);
    return *(serial_msg.clock_set);
}

auto SerMsg::has_status() const -> bool {
    return !(serial_msg.status == nullptr);
}

auto SerMsg::status() -> StatusMsg& {
    check_and_init(&(serial_msg.status), statusmsg_init_zero);
    return *(serial_msg.status);
}

auto SerMsg::has_dbg_msg() const -> bool {
    return !(serial_msg.dbg_msg == nullptr);
}

auto SerMsg::dbg_msg() -> DbgMsg& {
    check_and_init(&(serial_msg.dbg_msg), dbgmsg_init_zero);
    return *(serial_msg.dbg_msg);
}

auto SerMsg::has_log_msg() const -> bool {
    return !(serial_msg.log_msg == nullptr);
}

auto SerMsg::log_msg() -> LogMsg& {
    check_and_init(&(serial_msg.log_msg), logmsg_init_zero);
    return *(serial_msg.log_msg);
}

auto SerMsg::has_boot_log_msg() const -> bool {
    return !(serial_msg.boot_log_msg == nullptr);
}

auto SerMsg::boot_log_msg() -> BootLogMsg& {
    check_and_init(&(serial_msg.boot_log_msg), bootlogmsg_init_zero);
    return *(serial_msg.boot_log_msg);
}

auto SerMsg::has_data_msg() const -> bool {
    return !(serial_msg.data_msg == nullptr);
}

auto SerMsg::data_msg() -> DataMsg& {
    check_and_init(&(serial_msg.data_msg), datamsg_init_zero);
    return *(serial_msg.data_msg);
}

auto SerMsg::has_error_msg() const -> bool {
    return !(serial_msg.error_msg == nullptr);
}

auto SerMsg::error_msg() -> ErrorMsg& {
    check_and_init(&(serial_msg.error_msg), errmsg_init_zero);
    return *(serial_msg.error_msg);
}

auto SerMsg::has_time_msg() const -> bool {
    return !(&(serial_msg.time_msg) == nullptr);
}

auto SerMsg::time_msg() -> TimeMsg& {
    check_and_init(&(serial_msg.time_msg), timemsg_init_zero);
    return *(serial_msg.time_msg);
}

auto SerMsg::has_update_msg() const -> bool {
    return !(serial_msg.update_msg == nullptr);
}

auto SerMsg::update_msg() -> UpdateMsg& {
    check_and_init(&(serial_msg.update_msg), updatemsg_init_zero);
    return *(serial_msg.update_msg);
}

auto SerMsg::has_ver_msg() const -> bool {
    return !(serial_msg.ver_msg == nullptr);
}

auto SerMsg::ver_msg() -> VersionMsg& {
    check_and_init(&(serial_msg.ver_msg), versionmsg_init_zero);
    return *(serial_msg.ver_msg);
}

auto SerMsg::has_int_params_msg() const -> bool {
    return !(serial_msg.int_params_msg == nullptr);
}

auto SerMsg::int_params_msg() -> IntParamsMsg& {
    check_and_init(&(serial_msg.int_params_msg), intparamsmsg_init_zero);
    return *(serial_msg.int_params_msg);
}

auto SerMsg::has_voice_frame_msg() const -> bool {
    return !(serial_msg.voice_frame_msg == nullptr);
}

auto SerMsg::voice_frame_msg() -> VoiceFrameMsg& {
    check_and_init(&(serial_msg.voice_frame_msg), voiceframemsg_init_zero);
    return *(serial_msg.voice_frame_msg);
}

auto SerMsg::has_ack_msg() const -> bool {
    return !(serial_msg.ack_msg == nullptr);
}

auto SerMsg::ack_msg() -> AckMsg& {
    check_and_init(&(serial_msg.ack_msg), ackmsg_init_zero);
    return *(serial_msg.ack_msg);
}