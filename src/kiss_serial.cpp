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

#include "kiss_serial.hpp"
#include "os_portability.hpp"
#include <random>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <atomic>
#include <cstdio>
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <cstring>
#include "sha256.h"
#include "pseudo_serial.hpp"
#include "serial_data.hpp"
#include "peripherals.hpp"
#include "mem_trace.hpp"

extern portability::EventQueue *background_queue;

// debug_printf() uses this vector to determine which serial ports to send out
vector<KISSSerial *> kiss_sers;
portability::mutex *kiss_sers_mtx;
portability::mutex *shared_mtx;
void create_kiss_serial_data_objects() {
    kiss_sers_mtx = new portability::mutex();
    PORTABLE_ASSERT(kiss_sers_mtx != nullptr);
    shared_mtx = new portability::mutex();
    PORTABLE_ASSERT(shared_mtx != nullptr);
}

static constexpr uint8_t FEND = 0xC0;
static constexpr uint8_t FESC = 0xDB;
static constexpr uint8_t TFEND = 0xDC;
static constexpr uint8_t TFESC = 0xDD;
static constexpr uint8_t SETHW = 0x06;
static constexpr uint8_t SIGRPT = 0x07;
static constexpr uint8_t REBOOT = 0x08;
static constexpr uint8_t DATAPKT = 0x00;
static constexpr uint8_t VOICEPKT = 0x01;
static constexpr uint8_t DEBUGPKT = 0x02;
//static constexpr uint8_t QMPKT = 0x0A;
static constexpr uint8_t QMPKT = 0x20;
static constexpr uint8_t EXITKISS = 0xFF;
static constexpr uint32_t LOWER_NIBBLE = 0x0FU;
static constexpr uint32_t UPPER_NIBBLE = 0xF0U;
static constexpr uint8_t PORT_DATA = 0x00U;
static constexpr uint8_t PORT_VOICE = 0x01U;
static constexpr uint8_t PORT_DEBUG = 0x02U;
//static constexpr size_t MAX_MSG_SIZE = (SerialMsg_size+sizeof(crc_t))*2;

static auto compare_frame_crc(const vector<uint8_t> &buf) -> bool;
static auto compute_frame_crc(const vector<uint8_t> &buf) -> crc_t; 

auto load_SerMsg(SerMsg &ser_msg, PseudoSerial &ps) -> read_ser_msg_err_t {
    size_t byte_read_count = 0;
    bool kiss_extended = false;
    uint8_t kiss_type = 0;
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = ps.getc();
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        }
        uint32_t cur_byte_u32 = static_cast<uint32_t>(cur_byte) & LOWER_NIBBLE;
        uint32_t port_num = (static_cast<uint32_t>(cur_byte) & UPPER_NIBBLE) >> 4U;
        if(cur_byte_u32 == SETHW) {
            kiss_extended = false;
            kiss_type = SETHW;
            break;
        } 
        if(cur_byte_u32 == DATAPKT) {
            kiss_extended = false;
            if(port_num == PORT_DATA) {
                kiss_type = DATAPKT;
            } else if(port_num == PORT_VOICE) {
                kiss_type = VOICEPKT;
            } else if(port_num == PORT_DEBUG) {
                kiss_type = DEBUGPKT;
                kiss_extended = true;
            } else {
                PORTABLE_ASSERT(false);
            }
            break;
        } 
        if(cur_byte_u32 == QMPKT) {
            kiss_extended = true;
            kiss_type = QMPKT;
            break;
        }
        if(cur_byte_u32 == SIGRPT) {
            kiss_extended = false;
            kiss_type = SIGRPT;
            break;
        }
        if(cur_byte_u32 == REBOOT) {
            portability::sleep(ONE_SECOND);
            reboot_system();
        }
        if(cur_byte_u32 == EXITKISS) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
            return READ_SUCCESS;
        }  
        return READ_INVALID_KISS_ID;  
    }
    // Pull in the main frame
    vector<uint8_t> buf;
    for(;;) {
        int cur_byte = ps.getc();
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } 
        if(cur_byte == FESC) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
            if(cur_byte == TFESC) {
                buf.push_back(FESC);
            } else if(cur_byte == TFEND) {
                buf.push_back(FEND);
            } else {
                return INVALID_CHAR;
            }
        } else {
            buf.push_back(static_cast<uint8_t>(cur_byte));
        }
    }
    if(kiss_extended) {
        // Check the CRC
        if(!compare_frame_crc(buf)) {
            return CRC_ERR;
        }
        // Deserialize it
        ser_msg.clear();
        pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size()-sizeof(crc_t));
        if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
            return DECODE_SER_MSG_ERR;
        }
    } else {
        if(kiss_type == DATAPKT) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_DATA);
            ser_msg.data_msg().type = DataMsg_Type_KISSTX;
            ser_msg.data_msg().payload.size = buf.size();
            memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
        } else if(kiss_type == VOICEPKT) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_VOICE_MSG);
            ser_msg.data_msg().type = DataMsg_Type_KISSTX;
            ser_msg.data_msg().payload.size = buf.size();
            memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());            
        } else if(kiss_type == SETHW) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_SETHW);
            static constexpr int SETHW_SIZE = 17;
            PORTABLE_ASSERT(buf.size() == SETHW_SIZE);
            // KISS SetHardware 6
            struct SetHardware {
                uint32_t freq;
                uint32_t bw;
                uint16_t sf;
                uint16_t cr;
                uint16_t pwr;
                uint16_t sync;
                uint8_t crc;
            } __attribute__((packed)) set_hardware{};
            PORTABLE_ASSERT(sizeof(SetHardware) == SETHW_SIZE);
            memcpy(&set_hardware, buf.data(), buf.size());
            // Values are big-endian, change them to little-endian for the STM32
            ser_msg.sethw_msg().freq = __builtin_bswap32(set_hardware.freq);
            ser_msg.sethw_msg().bw = __builtin_bswap32(set_hardware.bw);
            ser_msg.sethw_msg().sf = __builtin_bswap16(set_hardware.sf);
            ser_msg.sethw_msg().cr = __builtin_bswap16(set_hardware.cr);
            ser_msg.sethw_msg().pwr = __builtin_bswap16(set_hardware.pwr);
            ser_msg.sethw_msg().sync = __builtin_bswap16(set_hardware.sync);
            ser_msg.sethw_msg().crc = set_hardware.crc;
        } else if(kiss_type == SIGRPT) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_SIGRPT);
            static constexpr int SIGRPT_SIZE = 4;
            PORTABLE_ASSERT(buf.size() == SIGRPT_SIZE);
            // KISS command 7
            struct SignalReport {
                int16_t rssi;
                int16_t snr;  // snr * 100
            } __attribute__((packed)) signal_report{};
            PORTABLE_ASSERT(sizeof(SignalReport) == SIGRPT_SIZE);
            memcpy(&signal_report, buf.data(), buf.size());
            // Values are big-endian, change them to little-endian for the STM32
            ser_msg.sigrpt_msg().rssi = __builtin_bswap16(signal_report.rssi);
            ser_msg.sigrpt_msg().snr = __builtin_bswap16(signal_report.snr);
        } else {
            PORTABLE_ASSERT(false);
        }
    }
    return READ_SUCCESS;
}


auto save_SerMsg(SerMsg &ser_msg, PseudoSerial &ps, const bool kiss_data_msg) -> write_ser_msg_err_t {
    vector<uint8_t> comb_buf; //NOTE: we can probably just have a buf, not a buf and comb_buf
    // If we're not doing KISS, we want to send the whole serialized protobuf message.
    // OTOH, if we're doing KISS, we just want to send back the payload.
    if(!kiss_data_msg) {
        if(ser_msg.type() == SerialMsg_Type_SETHW) {
            // KISS SetHardware 6
            struct SetHardware {
                uint32_t freq;
                uint32_t bw;
                uint16_t sf;
                uint16_t cr;
                uint16_t pwr;
                uint16_t sync;
                uint8_t crc;
            } __attribute__((packed)) set_hardware{};
            vector<uint8_t> buf(sizeof(SetHardware));
            // Values are big-endian, change them from little-endian for the STM32
            set_hardware.freq = __builtin_bswap32(ser_msg.sethw_msg().freq);
            set_hardware.bw = __builtin_bswap32(ser_msg.sethw_msg().bw);
            set_hardware.sf = __builtin_bswap16(ser_msg.sethw_msg().sf);
            set_hardware.cr = __builtin_bswap16(ser_msg.sethw_msg().cr);
            set_hardware.pwr = __builtin_bswap16(ser_msg.sethw_msg().pwr);
            set_hardware.sync = __builtin_bswap16(ser_msg.sethw_msg().sync);
            set_hardware.crc = ser_msg.sethw_msg().crc;
            memcpy(buf.data(), &set_hardware, buf.size());
        } else if(ser_msg.type() == SerialMsg_Type_SIGRPT) {
            // KISS command 7
            struct SignalReport {
                int16_t rssi;
                int16_t snr;  // snr * 100
            } __attribute__((packed)) signal_report{};
            vector<uint8_t> buf(sizeof(SignalReport));
            // Values are big-endian, change them from little-endian for the STM32
            signal_report.rssi = __builtin_bswap16(ser_msg.sigrpt_msg().rssi);
            signal_report.snr = __builtin_bswap16(ser_msg.sigrpt_msg().snr);
            memcpy(buf.data(), &signal_report, buf.size());            
        } else {
            vector<uint8_t> buf(SerMsg::maxSize()+sizeof(crc_t));
            pb_ostream_t stream = pb_ostream_from_buffer(buf.data(), buf.size()); 
            if(!pb_encode(&stream, SerialMsg_fields, &ser_msg)) {
                return ENCODE_SER_MSG_ERR;
            }
            comb_buf.resize(stream.bytes_written+sizeof(crc_t));
            memcpy(comb_buf.data(), buf.data(), stream.bytes_written);
            crc_t crc = compute_frame_crc(vector<uint8_t>(comb_buf.begin(), comb_buf.begin()+stream.bytes_written));
            memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t)); //NOLINT
        }
    } else {
        comb_buf.resize(ser_msg.data_msg().payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg().payload.bytes, ser_msg.data_msg().payload.size);
    }

    // Make it into a KISS frame and write it out
    if(ps.putc(FEND) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(ser_msg.type() == SerialMsg_Type_SETHW) {
            if(ps.putc(SETHW) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(ser_msg.type() == SerialMsg_Type_SIGRPT) {
            if(ps.putc(SIGRPT) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(ps.putc(QMPKT) == EOF) { return WRITE_SER_MSG_ERR; }
        }
    } else {
        enum kiss_port_type {
            DATA, 
            VOICE,
            NONE
        };
        if(ser_msg.type() == SerialMsg_Type_DATA) {
            if(ps.putc(DATAPKT | (static_cast<uint32_t>(DATA) << 4U)) == EOF) { return WRITE_SER_MSG_ERR; }  
        } else if(ser_msg.type() == SerialMsg_Type_VOICE_MSG) {
            if(ps.putc(DATAPKT | (static_cast<uint32_t>(VOICE) << 4U)) == EOF) { return WRITE_SER_MSG_ERR; }  
        } else {    
            PORTABLE_ASSERT(false);
        }
    }
    for(uint8_t i : comb_buf) {
        if(i == FEND) {
            if(ps.putc(FESC) == EOF) { return WRITE_SER_MSG_ERR; }
            if(ps.putc(TFEND) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(i == FESC) {
            if(ps.putc(FESC) == EOF) { return WRITE_SER_MSG_ERR; }
            if(ps.putc(TFESC) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(ps.putc(i) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(ps.putc(FEND) == EOF) { return WRITE_SER_MSG_ERR; } 
    return WRITE_SUCCESS;
}


static constexpr int FRAME_CRC_SIZE = 16;
static constexpr uint32_t LOWER_16_BITS = 0x0000FFFF;
static auto compute_frame_crc(const vector<uint8_t> &buf) -> crc_t {
    MbedCRC<POLY_16BIT_CCITT, FRAME_CRC_SIZE> ct;
    uint32_t gen_crc = 0;
    ct.compute(buf.data(), buf.size(), &gen_crc);
    return gen_crc & LOWER_16_BITS;
}


static constexpr uint32_t EIGHT_BITS = 8;
static auto compare_frame_crc(const vector<uint8_t> &buf) -> bool {
    PORTABLE_ASSERT(buf.size() >= 2);
    crc_t crc = compute_frame_crc(vector<uint8_t>(buf.begin(), buf.end()-2));
    crc_t crc_actual = 0;
    crc_actual |= *(buf.end()-1);
    crc_actual <<= EIGHT_BITS;
    crc_actual |= *(buf.end()-2);
    return crc == crc_actual;
}


void send_status() {
    kiss_sers_mtx->lock();
    for(auto & kiss_ser : kiss_sers) {
        kiss_ser->send_status();
    }
    kiss_sers_mtx->unlock();
}


auto KISSSerial::save_SerMsg(SerMsg &ser_msg, PseudoSerial &ps, const bool kiss_data_msg) -> write_ser_msg_err_t {
    vector<uint8_t> comb_buf;
    // If we're not doing KISS, we want to send the whole serialized protobuf message.
    // OTOH, if we're doing KISS, we just want to send back the payload.
    if(!kiss_data_msg) {
        vector<uint8_t> buf(SerMsg::maxSize()+sizeof(crc_t));
        pb_ostream_t stream = pb_ostream_from_buffer(buf.data(), buf.size()); 
        if(!pb_encode(&stream, SerialMsg_fields, &ser_msg)) {
            return ENCODE_SER_MSG_ERR;
        }
        comb_buf.resize(stream.bytes_written);
        copy(buf.begin(), buf.begin()+comb_buf.size(), comb_buf.begin());
        crc_t crc = compute_frame_crc(comb_buf);
        constexpr uint16_t LOWER_BYTE = 0x00FF;
        constexpr uint32_t SHIFT_BYTE = 8;
        comb_buf.push_back(crc & LOWER_BYTE);
        comb_buf.push_back((static_cast<uint32_t>(crc) >> SHIFT_BYTE) & LOWER_BYTE);
    } else {
        comb_buf.resize(ser_msg.data_msg().payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg().payload.bytes, comb_buf.size());
    }

    // Make it into a KISS frame and write it out
    if(ps.putc(FEND) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(ps.putc(SETHW) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(ps.putc(DATAPKT) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint8_t i : comb_buf) {
        if(i == FEND) {
            if(ps.putc(FESC) == EOF) { return WRITE_SER_MSG_ERR; }
            if(ps.putc(TFEND) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(i == FESC) {
            if(ps.putc(FESC) == EOF) { return WRITE_SER_MSG_ERR; }
            if(ps.putc(TFESC) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(ps.putc(i) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(ps.putc(FEND) == EOF) { return WRITE_SER_MSG_ERR; } 
    return WRITE_SUCCESS;
}


KISSSerial::KISSSerial(string my_port_name, ser_port_type_t ser_port_type) :
    port_type(ser_port_type),
    port_name(std::move(my_port_name)),
    pser_rd(nullptr),
    pser_wr(nullptr) {
    past_log_msg.clear();
    kiss_extended = true;
    logfile_names = vector<string>();
        
    string rx_ser_name("RX-");
    rx_ser_name.append(portName());
    rx_ser_thread = new portability::Thread(osPriorityNormal, SER_THREAD_STACK_SIZE*2, nullptr, rx_ser_name.c_str());
    string tx_ser_name("TX-");
    tx_ser_name.append(portName());
    tx_ser_thread = new portability::Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, tx_ser_name.c_str());
}


static constexpr uint32_t BT_BAUD_RATE = 115200;

#if MBED_CONF_APP_HAS_BLE == 0
static constexpr ESP32CfgSubMsg ESP32CfgSubMsg_zero = ESP32CfgSubMsg_init_zero;
KISSSerialUART::KISSSerialUART(const string &my_port_name, const ser_port_type_t ser_port_type) :
    KISSSerial(my_port_name, ser_port_type),
    cfg(ESP32CfgSubMsg_zero),
    isESP32(false),
    tx_port(NC),
    rx_port(NC),
    cts_port(NC),
    rts_port(NC),
    esp32_rst(DigitalInOut(NC)), 
    ser(nullptr),
    using_stdio(true),
    flow_control(false) { 
    startThreads();

    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
}


KISSSerialUART::KISSSerialUART(PinName tx, PinName rx, PinName rst, ESP32CfgSubMsg &my_cfg, 
    const ser_port_type_t ser_port_type) :
    KISSSerialUART(tx, rx, rst, NC, NC, my_cfg, ser_port_type) { }


KISSSerialUART::KISSSerialUART(PinName tx, PinName rx, PinName rst, PinName cts, PinName rts,
                                ESP32CfgSubMsg &my_cfg, const ser_port_type_t ser_port_type) :
KISSSerial(cfg.ser_name, ser_port_type), 
cfg(my_cfg),
isESP32(true),
tx_port(tx),
rx_port(rx),
cts_port(cts),
rts_port(rts),
esp32_rst(DigitalInOut(rst)),
ser(make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE)),
flow_control(cts != NC) {
    PORTABLE_ASSERT(ser);
    *pserRd() = make_shared<ESP32PseudoSerial>(tx, rx, rst, cts, rts, my_cfg);
    *pserWr() = *pserRd();
    using_stdio = false;
    printf("starting the threads\r\n");
    startThreads();
    printf("loading the serial ports\r\n");
    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
    printf("loaded the serial ports\r\n");
}


KISSSerialUART::KISSSerialUART(PinName tx, PinName rx, const ser_port_type_t ser_port_type) :
        KISSSerial(cfg.ser_name, ser_port_type), 
        cfg(ESP32CfgSubMsg_zero),
        isESP32(false),
        tx_port(tx),
        rx_port(rx),
        cts_port(NC),
        rts_port(NC),
        esp32_rst(DigitalInOut(NC)),
        ser(make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE)),
        flow_control(false) {
    PORTABLE_ASSERT(ser);
    *pserRd() = make_shared<UARTPseudoSerial>(ser, true);
    *pserWr() = make_shared<UARTPseudoSerial>(ser, false);
    using_stdio = false;

    startThreads();

    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
}


KISSSerialUART::KISSSerialUART(PinName tx, PinName rx, PinName cts, PinName rts, 
            ser_port_type_t ser_port_type) :
        KISSSerial(cfg.ser_name, ser_port_type), 
        cfg(ESP32CfgSubMsg_zero),
        isESP32(false),
        tx_port(tx),
        rx_port(rx),
        cts_port(cts),
        rts_port(rts),
        esp32_rst(DigitalInOut(NC)),
        ser(make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE)),
        flow_control(true) {
    PORTABLE_ASSERT(ser);
    *pserRd() = make_shared<UARTPseudoSerial>(ser, true);
    *pserWr() = make_shared<UARTPseudoSerial>(ser, false);
    using_stdio = false;
    FILE *ser_fh = fdopen(&*ser, "rw");
    PORTABLE_ASSERT(ser_fh != nullptr);
    if(flow_control) {
        set_uart_flow_ctl(ser_fh);
    }
    fclose(ser_fh);
    startThreads();

    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
}

#endif /* #if MBED_CONF_APP_HAS_BLE == 0 */


//static constexpr int BT_NAME_MAX_LEN = 8;
//static constexpr int BT_PIN_MAX_LEN = 4;
void KISSSerialUART::set_uart_flow_ctl(FILE *ser_fh) {
    PORTABLE_ASSERT(false);
    // Set the UART values in the ESP32
    string flow_ctl_cmd("AT+UART_CUR=");
    constexpr int STR_SIZE = 32;
    vector<char> baud_rate_string(STR_SIZE);
    snprintf(baud_rate_string.data(), baud_rate_string.size(), "%8d", static_cast<int32_t>(BT_BAUD_RATE));
    flow_ctl_cmd.append(baud_rate_string.data());
    flow_ctl_cmd.append(",8,1,0,3\r\n");
    fprintf(ser_fh, PseudoSerial::safe_pcts(flow_ctl_cmd).c_str(), flow_ctl_cmd.c_str()); 
    // Set the UART values on the ST board
    ser->set_baud(BT_BAUD_RATE);
    ser->set_flow_control(mbed::SerialBase::RTSCTS, rts_port, cts_port);
}


void KISSSerialUART::sleep() {
//    delete ser;
}


void KISSSerialUART::wake() {
#if 0
    if(ser == nullptr) {
        ser = make_shared<UARTSerial>(tx_port, rx_port, SER_BAUD_RATE);
        PORTABLE_ASSERT(ser);
    }
    if(hc05) {
        configure_hc05();
    }
#endif
}


KISSSerial::~KISSSerial() {
    kiss_sers_mtx->lock();
    kiss_sers.erase(find(kiss_sers.begin(), kiss_sers.end(), this));
    kiss_sers_mtx->unlock();
    rx_ser_thread->join();
    delete rx_ser_thread;
    tx_ser_thread->join();
    delete tx_ser_thread;
}


KISSSerialUART::~KISSSerialUART() {
    if(isESP32) {
        FILE *ser_fh = fdopen(&*ser, "rw");
        portability::sleep(QUARTER_SECOND);
        string bt_leave_passthrough_cmd("+++\r\n");
        if(ser_fh != nullptr) {
            fprintf(ser_fh, PseudoSerial::safe_pcts(bt_leave_passthrough_cmd).c_str(), 
                    bt_leave_passthrough_cmd.c_str());
        } else {
            PORTABLE_ASSERT(false);
        }
        portability::sleep(ONE_SECOND);
    } 
}


void KISSSerial::enqueue_msg(const shared_ptr<SerMsg> &ser_msg_sptr) { 
    if(ser_msg_sptr->type() == SerialMsg_Type_DATA) {
        PORTABLE_ASSERT(ser_msg_sptr->has_data_msg());
        auto out_ser_msg = make_shared<SerMsg>();
        *out_ser_msg = *ser_msg_sptr;
        if(!kiss_extended) {
            if(out_ser_msg->data_msg().type == DataMsg_Type_TX) {
                out_ser_msg->data_msg().type = DataMsg_Type_KISSTX;
            }
            if(out_ser_msg->data_msg().type == DataMsg_Type_RX) {
                out_ser_msg->data_msg().type = DataMsg_Type_KISSRX;
            }
        }
        if(((ser_msg_sptr->data_msg().voice && (port_type == DEBUG_PORT || port_type == VOICE_PORT))) || 
            (!ser_msg_sptr->data_msg().voice && (port_type == DEBUG_PORT || port_type == DATA_PORT))) {
            tx_ser_queue.enqueue_mail(ser_msg_sptr);  
        } else {
            PORTABLE_ASSERT(false);
        }
    } else if(kiss_extended) {
        tx_ser_queue.enqueue_mail(ser_msg_sptr);
    }
}


auto KISSSerial::load_SerMsg(SerMsg &ser_msg, PseudoSerial &ps) -> read_ser_msg_err_t {
    size_t byte_read_count = 0;
    enum {
        DATA, 
        VOICE,
        DEBUG,
        NONE
    } kiss_port_type = NONE;
    // Get past the first delimiter(s)
    for(;;) {
        auto cur_byte = static_cast<uint32_t>(ps.getc());
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        }
        if((cur_byte & LOWER_NIBBLE) == SETHW) {
            kissExtended(false);
            break;
        } 
        if((cur_byte & LOWER_NIBBLE) == DATAPKT) {
            uint32_t kiss_port = ((cur_byte & UPPER_NIBBLE) >> 4U);
            if(kiss_port == DATA) {
                kiss_port_type = DATA;
                kissExtended(false);
            } else if(kiss_port == VOICE) {
                kiss_port_type = VOICE;
                kissExtended(false);
            } else if(kiss_port == DEBUG) {
                kissExtended(true);
            } else {
                PORTABLE_ASSERT(false);
            }
            break;
        } 
        if(cur_byte == EXITKISS) {
            ser_msg.clear();
            ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
            return READ_SUCCESS;
        }  
        return READ_INVALID_KISS_ID;
    }
    // Pull in the main frame
    vector<uint8_t> buf;
    for(;;) {
        int cur_byte = ps.getc();
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } 
        if(cur_byte == FESC) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
            if(cur_byte == TFESC) {
                buf.push_back(FESC);
            } else if(cur_byte == TFEND) {
                buf.push_back(FEND);
            } else {
                return INVALID_CHAR;
            }
        } else {
            buf.push_back(static_cast<uint8_t>(cur_byte));
        }
    }
    if(isKISSExtended()) {
        // Check the CRC
        if(!compare_frame_crc(buf)) {
            return CRC_ERR;
        }
        // Deserialize it
        ser_msg.clear();
        pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size()-sizeof(crc_t));
        if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
            return DECODE_SER_MSG_ERR;
        }
    } else {
        ser_msg.clear();
        if(kiss_port_type == DATA) {
            ser_msg.type(SerialMsg_Type_DATA);
            ser_msg.data_msg().type = DataMsg_Type_KISSTX;
        } else if(kiss_port_type == VOICE) {
            ser_msg.type(SerialMsg_Type_VOICE_MSG);
            ser_msg.data_msg().type = DataMsg_Type_KISSTX;            
        } else {
            PORTABLE_ASSERT(false);
        }
        ser_msg.data_msg().payload.size = buf.size();
        memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


/**
 * Sends the current status.
 */
extern atomic<int> total_rx_pkt;
extern atomic<int> total_rx_corr_pkt;
extern atomic<int> total_tx_pkt;
extern atomic<int> last_rx_rssi;
extern atomic<int> last_rx_snr;

void KISSSerial::send_status() {
    auto ser_msg = make_shared<SerMsg>();
    ser_msg->type(SerialMsg_Type_STATUS);
    ser_msg->status();
    if(current_mode == system_state_t::BOOTING) {
        ser_msg->status().status = StatusMsg_Status_BOOTING;      
    } else if(current_mode == system_state_t::MANAGEMENT) {
        ser_msg->status().status = StatusMsg_Status_MANAGEMENT;     
    } else if(current_mode == system_state_t::RUNNING) {
        ser_msg->status().status = StatusMsg_Status_RUNNING;
    } else {
        PORTABLE_ASSERT(false);
    }
    ser_msg->status().tx_full = tx_frame_mail->full();
    ser_msg->status().time = time(nullptr);
    auto *display_file = fopen("/fs/display.off", "re");
    if(display_file == nullptr) {
        ser_msg->status().oled_on = true;
    } else {
        fclose(display_file);
        ser_msg->status().oled_on = false;
    }
    ser_msg->status().total_rx_pkt = total_rx_pkt;
    ser_msg->status().total_rx_corr_pkt = total_rx_corr_pkt;
    ser_msg->status().total_tx_pkt = total_tx_pkt;
    ser_msg->status().last_rx_rssi = last_rx_rssi;
    ser_msg->status().last_rx_snr = last_rx_snr;
    // Status of the radio queue
    ser_msg->status().radio_out_queue_level = unified_radio_evt_mail->getLevel();
    // Heap size for tracking whether we have memory leak(s)
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    ser_msg->status().heap_size = heap_stats.current_size;
    ser_msg->status().peak_mem_usage = get_max_memory_usage();
    // How many protocol deadlines we've missed
    ser_msg->status().missed_deadlines = RadioTiming::getNumMissedDeadlines();
    ser_msg->status().total_deadlines = RadioTiming::getNumTotalDeadlines();
    tx_ser_queue.enqueue_mail(ser_msg);
}  


void KISSSerial::send_ack() {
    auto ser_msg_sptr = make_shared<SerMsg>();
    ser_msg_sptr->type(SerialMsg_Type_ACK);
    ser_msg_sptr->ack_msg().radio_out_queue_level = unified_radio_evt_mail->getLevel();
    tx_ser_queue.enqueue_mail(ser_msg_sptr);    
}  


void KISSSerial::send_error(const string &err_str) {
    auto ser_msg_sptr = make_shared<SerMsg>();
    ser_msg_sptr->type(SerialMsg_Type_ERR);
    strncpy(ser_msg_sptr->error_msg().msg, err_str.c_str(), sizeof(ser_msg_sptr->error_msg().msg));
    debug_printf(DBG_WARN, PseudoSerial::safe_pcts(err_str).c_str(), err_str.c_str());
    tx_ser_queue.enqueue_mail(ser_msg_sptr);   
}  

