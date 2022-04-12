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
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial_msg.hpp"
#include <cstring>
#include "USBSerial.h"
#include "sha256.h"
#include "pseudo_serial.hpp"
#include "serial_data.hpp"
#include "voice_msg.hpp"
#include "peripherals.hpp"
#include "Adafruit_SSD1306.h"
#include "mem_trace.hpp"

extern portability::EventQueue *background_queue;

static constexpr int ERR_MSG_SIZE = 32;
static constexpr int SHA256_SIZE = 32;

// debug_printf() uses this vector to determine which serial ports to send out
vector<KISSSerial *> kiss_sers;
portability::mutex *kiss_sers_mtx;
static portability::mutex *shared_mtx;
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
static constexpr uint8_t DATAPKT = 0x00;
static constexpr uint8_t EXITKISS = 0xFF;
//static constexpr size_t MAX_MSG_SIZE = (SerialMsg_size+sizeof(crc_t))*2;

static auto compare_frame_crc(const vector<uint8_t> &buf) -> bool;
static auto compute_frame_crc(const vector<uint8_t> &buf) -> crc_t; 

static const DataMsg data_msg_zero = DataMsg_init_zero;

auto load_SerMsg(SerMsg &ser_msg, PseudoSerial &ps) -> read_ser_msg_err_t {
    size_t byte_read_count = 0;
    bool kiss_extended = false;
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = ps.getc();
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        }
        if(cur_byte != FEND) {
            constexpr uint32_t LOWER_NIBBLE = 0x0F;
            uint32_t cur_byte_u32 = static_cast<uint32_t>(cur_byte) & LOWER_NIBBLE;
            if(cur_byte_u32 == SETHW) {
                kiss_extended = true;
                break;
            } 
            if(cur_byte_u32 == DATAPKT) {
                kiss_extended = false;
                break;
            } 
            if(cur_byte_u32 == EXITKISS) {
                ser_msg.clear();
                ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
                return READ_SUCCESS;
            }  
            return READ_INVALID_KISS_ID;  
        }
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
        ser_msg.clear();;
        ser_msg.type(SerialMsg_Type_DATA);
        ser_msg.data_msg().type = DataMsg_Type_KISSTX;
        ser_msg.data_msg().payload.size = buf.size();
        memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


auto save_SerMsg(SerMsg &ser_msg, PseudoSerial &ps, const bool kiss_data_msg) -> write_ser_msg_err_t {
    vector<uint8_t> comb_buf; //NOTE: we can probably just have a buf, not a buf and comb_buf
    // If we're not doing KISS, we want to send the whole serialized protobuf message.
    // OTOH, if we're doing KISS, we just want to send back the payload.
    if(!kiss_data_msg) {
        vector<uint8_t> buf(SerMsg::maxSize()+sizeof(crc_t));
        pb_ostream_t stream = pb_ostream_from_buffer(buf.data(), buf.size()); 
        if(!pb_encode(&stream, SerialMsg_fields, &ser_msg)) {
            return ENCODE_SER_MSG_ERR;
        }
        comb_buf.resize(stream.bytes_written+sizeof(crc_t));
        memcpy(comb_buf.data(), buf.data(), stream.bytes_written);
        crc_t crc = compute_frame_crc(vector<uint8_t>(comb_buf.begin(), comb_buf.begin()+stream.bytes_written));
        memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t)); //NOLINT
    } else {
        comb_buf.resize(ser_msg.data_msg().payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg().payload.bytes, ser_msg.data_msg().payload.size);
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
    port_name(std::move(my_port_name)) {
    pser_rd = nullptr;
    pser_wr = nullptr;
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


static constexpr uint32_t SER_BAUD_RATE = 230400;
static constexpr uint32_t BT_BAUD_RATE = 115200;
KISSSerialUART::KISSSerialUART(PinName tx, PinName rx, PinName rst, ESP32CfgSubMsg &my_cfg, 
    const ser_port_type_t ser_port_type) :
        KISSSerial(cfg.ser_name, ser_port_type), 
        cfg(my_cfg),
        isESP32(true),
        tx_port(tx),
        rx_port(rx),
        cts_port(NC),
        rts_port(NC),
        esp32_rst(DigitalInOut(rst)),
        ser(new UARTSerial(tx_port, rx_port, SER_BAUD_RATE)),
        flow_control(false) {
    PORTABLE_ASSERT(ser);
    *pserRd() = make_shared<UARTPseudoSerial>(ser, true);
    *pserWr() = make_shared<UARTPseudoSerial>(ser, false);
    using_stdio = false;
    if(my_cfg.isBT) {
        configure_esp32_bt();
    } else {
        configure_esp32_wifi();
    }

    startThreads();

    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
}



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
        ser(new UARTSerial(tx_port, rx_port, SER_BAUD_RATE)),
        flow_control(true) {
    PORTABLE_ASSERT(ser);
    *pserRd() = make_shared<UARTPseudoSerial>(ser, true);
    *pserWr() = make_shared<UARTPseudoSerial>(ser, false);
    using_stdio = false;
    if(my_cfg.isBT) {
        configure_esp32_bt();
    } else {
        configure_esp32_wifi();
    }

    startThreads();

    kiss_sers_mtx->lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx->unlock();
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
        ser(new UARTSerial(tx_port, rx_port, SER_BAUD_RATE)),
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


static constexpr int QUARTER_SECOND = 250;
static constexpr int HALF_SECOND = 500;
static constexpr int ONE_SECOND = 1000;
static constexpr int BT_NAME_MAX_LEN = 8;
void KISSSerialUART::set_uart_flow_ctl(FILE *ser_fh) {
    // Set the UART values in the ESP32
    string flow_ctl_cmd("AT+UART_CUR=");
    constexpr int STR_SIZE = 32;
    vector<char> baud_rate_string(STR_SIZE);
    snprintf(baud_rate_string.data(), baud_rate_string.size(), "%d", BT_BAUD_RATE);
    flow_ctl_cmd.append(baud_rate_string.data());
    flow_ctl_cmd.append(",8,1,0,3\r\n");
    fprintf(ser_fh, "%s", flow_ctl_cmd.c_str()); 
    // Set the UART values on the ST board
    ser->set_baud(BT_BAUD_RATE);
    ser->set_flow_control(mbed::SerialBase::RTSCTS, rts_port, cts_port);
}


void KISSSerialUART::configure_esp32_bt() {
    PORTABLE_ASSERT(string(cfg.bt_name).size() <= BT_NAME_MAX_LEN);
    // Reset the ESP32 board
    esp32_rst.mode(OpenDrain);
    esp32_rst.mode(PullNone);
    esp32_rst = 0;
    portability::sleep(HALF_SECOND);
    esp32_rst = 1;
    portability::sleep(ONE_SECOND);
    // Set initial UART baud rate
    ser->set_baud(BT_BAUD_RATE);
    portability::sleep(QUARTER_SECOND);
    FILE *ser_fh = fdopen(&*ser, "rw");
    if(flow_control) {
        set_uart_flow_ctl(ser_fh);
    }
    // Set the ESP32 to Bluetooth Classic, and init the SPP
    string bt_classic_cmd("AT+BTINIT=1\r\n");
    fprintf(ser_fh, "%s", bt_classic_cmd.c_str());
    string bt_spp_cmd("AT+BTSPPINIT=2\r\n");
    fprintf(ser_fh, "%s", bt_spp_cmd.c_str()); 
    // Set the ESP32 name
    string bt_name_cmd("AT+BTNAME=\"");
    bt_name_cmd.append(cfg.bt_name);
    bt_name_cmd.append("\"\r\n");
    fprintf(ser_fh, "%s", bt_name_cmd.c_str());
    // Set connection params
    string bt_scanmode_cmd("AT+BTSCANMODE=2\r\n");
    fprintf(ser_fh, "%s", bt_scanmode_cmd.c_str());
    string bt_sec_cmd("AT+BTSECPARAM=3,1,");
    bt_sec_cmd.append("\"");
    bt_sec_cmd.append(cfg.bt_pin);
    bt_sec_cmd.append("\"");
    bt_sec_cmd.append("\r\n");
    fprintf(ser_fh, "%s", bt_sec_cmd.c_str());
    // Enter passthrough mode
    string bt_enter_spp_cmd("AT+BTSPPSEND\r\n");
    fprintf(ser_fh, "%s", bt_enter_spp_cmd.c_str());    

    fclose(ser_fh);
    printf("Done with configuration\r\n");
}


static constexpr int SSID_MAX_LEN = 8;
static constexpr int PASS_MAX_LEN = 32;
void KISSSerialUART::configure_esp32_wifi() {
    PORTABLE_ASSERT(string(cfg.ssid).size() <= SSID_MAX_LEN);
    PORTABLE_ASSERT(string(cfg.pass).size() <= PASS_MAX_LEN);
    // Reset the ESP32 board
    esp32_rst.mode(OpenDrain);
    esp32_rst.mode(PullNone);
    esp32_rst = 0;
    portability::sleep(HALF_SECOND);
    esp32_rst = 1;
    portability::sleep(ONE_SECOND);
    // Set initial UART baud rate
    ser->set_baud(BT_BAUD_RATE);
    portability::sleep(QUARTER_SECOND);
    FILE *ser_fh = fdopen(&*ser, "rw");
    if(flow_control) {
        set_uart_flow_ctl(ser_fh);
    }
    if(cfg.isAP) {
        // Set the Wi-Fi mode to SoftAP+STA mode
        string wifi_mode_cmd("AT+CWMODE=3\r\n");
        fprintf(ser_fh, "%s", wifi_mode_cmd.c_str());
        // Set the ESP SoftAP params
        string wifi_softap_cmd("AT+CWSAP=");
        wifi_softap_cmd.append(cfg.ssid);
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append(cfg.pass);
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append(cfg.wifi_chan); // Just use Channel 6 for now
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append("0"); // Open; no encryption
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append("1"); // Only allow one station to connect
        wifi_softap_cmd.append(",");
        wifi_softap_cmd.append("0"); // Broadcast SSID
        wifi_softap_cmd.append("\r\n");
        fprintf(ser_fh, "%s", wifi_softap_cmd.c_str());
        // Set the IP configuration of the ESP SoftAP
        string wifi_ip_cmd(R"("192.168.10.1","192.168.10.1","255.255.255.0")");
        wifi_ip_cmd.append("\r\n");
        fprintf(ser_fh, "%s", wifi_ip_cmd.c_str());
        // Setup and enable mDNS
        string wifi_mdns_cmd("AT+MDNS=1,\"QMesh-");
        wifi_mdns_cmd.append(cfg.ssid);
        wifi_mdns_cmd.append("\",\"_iot\",8080\r\n");
        fprintf(ser_fh, "%s", wifi_mdns_cmd.c_str());
        // Setup the DHCP server
        string wifi_dhcp_cmd("AT+CWDHCP=1,2\r\n");
        fprintf(ser_fh, "%s", wifi_dhcp_cmd.c_str());
        string wifi_dhcp_ip_range("AT+CWDHCPS=1,3,");
        wifi_dhcp_ip_range.append("\"");
        wifi_dhcp_ip_range.append(cfg.dhcp_range_lo);
        wifi_dhcp_ip_range.append("\"");
        wifi_dhcp_ip_range.append(",\"");
        wifi_dhcp_ip_range.append(cfg.dhcp_range_hi);
        wifi_dhcp_ip_range.append("\"\r\n");
        fprintf(ser_fh, "%s", wifi_dhcp_ip_range.c_str());
    } else {
        // Enable DHCP
        string wifi_dhcp_cmd("AT+CWDHCP=1,1\r\n");
        fprintf(ser_fh, "%s", wifi_dhcp_cmd.c_str());
        // Setup and enable mDNS
        string wifi_mdns_cmd("AT+MDNS=1,\"QMesh-");
        wifi_mdns_cmd.append(cfg.ssid);
        wifi_mdns_cmd.append("\",\"_iot\",8080\r\n");
        fprintf(ser_fh, "%s", wifi_mdns_cmd.c_str()); 
        // Connect to the AP
        string wifi_conn_ap_cmd("AT+CWJAP=");
        wifi_conn_ap_cmd.append(cfg.ssid);
        wifi_conn_ap_cmd.append(",");
        wifi_conn_ap_cmd.append(cfg.pass);
        wifi_conn_ap_cmd.append("\r\n");
        fprintf(ser_fh, "%s", wifi_conn_ap_cmd.c_str());
    }

    // Setup a UDP server to transmit/receive on a multicast IP address
    string wifi_conn_mux_cmd("AT+CIPMUX=0\r\n");
    fprintf(ser_fh, "%s", wifi_conn_mux_cmd.c_str());
    string wifi_udp_srvr_cmd("AT+CIPSTART=");
    wifi_udp_srvr_cmd.append(cfg.multicast_addr);
    wifi_udp_srvr_cmd.append(",");
    wifi_udp_srvr_cmd.append(cfg.remote_port);
    wifi_udp_srvr_cmd.append(",");
    wifi_udp_srvr_cmd.append(cfg.local_port);
    wifi_udp_srvr_cmd.append(",0\r\n");
    fprintf(ser_fh, "%s", wifi_udp_srvr_cmd.c_str());

    // Enter passthrough mode
    string wifi_pass_cmd("AT+CIPSEND\r\n");
    fprintf(ser_fh, "%s", wifi_pass_cmd.c_str());
    portability::sleep(QUARTER_SECOND);

    fclose(ser_fh);
    printf("Done with configuration\r\n");
}


void KISSSerialUART::sleep() {
//    delete ser;
}


void KISSSerialUART::wake() {
#if 0
    if(ser == nullptr) {
        ser = new UARTSerial(tx_port, rx_port, SER_BAUD_RATE);
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
        fprintf(ser_fh, "%s", bt_leave_passthrough_cmd.c_str());
        portability::sleep(ONE_SECOND);
    } 
    delete ser;
}


void KISSSerial::enqueue_msg(shared_ptr<SerMsg> ser_msg_sptr) { //NOLINT
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
        if(ser_msg_sptr->data_msg().voice && 
            (port_type == DEBUG_PORT || port_type == VOICE_PORT || port_type == APRS_PORT)) {
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
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = ps.getc();
        if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = ps.getc();
            if(++byte_read_count > SerMsg::maxSize()+sizeof(crc_t)) { return READ_MSG_OVERRUN_ERR; }
        }
        if(cur_byte != FEND) {
            if(cur_byte == SETHW) {
                kissExtended(true);
                break;
            } 
            if(cur_byte == DATAPKT) {
                kissExtended(false);
                break;
            } 
            if(cur_byte == EXITKISS) {
                ser_msg.clear();
                ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
                return READ_SUCCESS;
            }  
            return READ_INVALID_KISS_ID;
        }
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
        ser_msg.type(SerialMsg_Type_DATA);
        ser_msg.data_msg().type = DataMsg_Type_KISSTX;
        ser_msg.data_msg().payload.size = buf.size();
        memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


void KISSSerial::tx_serial_thread_fn() {
    for(;;) {
        auto ser_msg_sptr = tx_ser_queue.dequeue_mail();
        // In KISS mode, we only send out data packets in KISS format.
        if(!kiss_extended) {
            // Silently drop anything that isn't a KISSRX frame when we're in KISS mode.
            //  Also, we don't want to send out redundant packets that we use for testing/debugging.
            if(ser_msg_sptr->type() == SerialMsg_Type_DATA && ser_msg_sptr->has_data_msg() && 
                ser_msg_sptr->data_msg().type == DataMsg_Type_KISSRX && 
                !ser_msg_sptr->data_msg().redundant) {
                save_SerMsg(*ser_msg_sptr, *pser_wr, true);
            } else {
                continue;
            }           
        } else {
            save_SerMsg(*ser_msg_sptr, *pser_wr, false);  
        }
    }
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
    debug_printf(DBG_WARN, "%s", err_str.c_str());
    tx_ser_queue.enqueue_mail(ser_msg_sptr);   
}  


static auto check_upd_pkt_sha256(const UpdateMsg &update_msg) -> bool;
static auto check_upd_pkt_sha256(const UpdateMsg &update_msg) -> bool {
    mbedtls_sha256_context sha256_cxt;
    mbedtls_sha256_init(&sha256_cxt);
    mbedtls_sha256_starts(&sha256_cxt, 0);
    mbedtls_sha256_update(&sha256_cxt, update_msg.pld.bytes, update_msg.pld.size);
    vector<uint8_t> sha256_cksum(SHA256_SIZE);
    mbedtls_sha256_finish(&sha256_cxt, sha256_cksum.data());
    return memcmp(update_msg.sha256_pkt.bytes, sha256_cksum.data(), SHA256_SIZE) == 0;
}


static auto getFlashCompileString() -> string;
static auto getFlashCompileString() -> string {
    string str("COMPILE TIME: ");
    str.append(__DATE__);
    str.append("; ");
    str.append(__TIME__);
    str.append("\r\n");
    return str;
}


extern shared_ptr<Adafruit_SSD1306_I2c> oled;
void KISSSerial::rx_serial_thread_fn() {
    int upd_pkt_cnt = -1;
	FILE *f = nullptr;
    FILE *upd_file = nullptr;
    mbedtls_sha256_context sha256_cxt;
	int line_count = 0;
	bool reading_log = false;
	bool reading_bootlog = false;
    past_log_msg.clear();
    auto ser_msg = make_shared<SerMsg>();
    int err = 0;
    auto voice = make_shared<VoiceMsgProcessor>();
    for(;;) {
        ser_msg->clear();
        err = load_SerMsg(*ser_msg, *pser_rd);
        if(err != 0) {
            debug_printf(DBG_WARN, "Error in reading serial port entry. Error %d\r\n", err);
            if(err == CRC_ERR) {
                auto reply_msg = make_shared<SerMsg>();
                reply_msg->type(SerialMsg_Type_ERR);
                reply_msg->error_msg().type = ErrorMsg_Type_CRC_ERR;
                string err_reason("CRC Error");
                strncpy(reply_msg->error_msg().msg, err_reason.c_str(), sizeof(reply_msg->error_msg().msg));
                tx_ser_queue.enqueue_mail(reply_msg);
            }
            continue;
        }
        if(ser_msg->type() == SerialMsg_Type_TURN_OLED_OFF) {
            debug_printf(DBG_INFO, "Received a request to turn OFF the OLED display\r\n");
            auto *disp_file = fopen("/fs/display.off", "we");
            PORTABLE_ASSERT(disp_file != nullptr);
            fclose(disp_file);
            oled->displayOff();
        }
        if(ser_msg->type() == SerialMsg_Type_VOICE_MSG) {
            PORTABLE_ASSERT(ser_msg->has_voice_frame_msg());
            if(!ser_msg->voice_frame_msg().end_stream) {
                vector<uint8_t> voice_frame(ser_msg->voice_frame_msg().payload.size);
                memcpy(voice_frame.data(), ser_msg->voice_frame_msg().payload.bytes, 
                        ser_msg->voice_frame_msg().payload.size);
                if(voice->addFrame(voice_frame)) {
                    auto frame = make_shared<Frame>();
                    frame->createFromVoice(*voice);
                    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frame);
                    unified_radio_evt_mail->enqueue_mail(radio_evt); 
                    voice->clearFrames();
                }
            } else {
                auto frame = make_shared<Frame>();
                frame->createFromVoice(*voice);
                auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frame);
                unified_radio_evt_mail->enqueue_mail(radio_evt); 
                voice->clearFrames();
            }
            send_ack();
        }
        if(ser_msg->type() == SerialMsg_Type_TURN_OLED_ON) {
            debug_printf(DBG_INFO, "Received a request to turn ON the OLED display\r\n");
            fs->remove("display.off");
            auto *disp_file = fopen("/fs/display.off", "re");
            PORTABLE_ASSERT(disp_file == nullptr);
            oled->displayOn();
        }        
        if(ser_msg->type() == SerialMsg_Type_VERSION) {
            debug_printf(DBG_INFO, "Received a FW version request message\r\n");
            auto reply_msg = make_shared<SerMsg>();
            reply_msg->type(SerialMsg_Type_VERSION);
            string compile_str = getFlashCompileString();
            debug_printf(DBG_INFO, "Sending %s\r\n", compile_str.c_str());
            strncpy(reply_msg->ver_msg().msg, compile_str.c_str(), sizeof(reply_msg->ver_msg().msg));
            portability::sleep(HALF_SECOND);
            tx_ser_queue.enqueue_mail(reply_msg);
        }
        if(ser_msg->type() == SerialMsg_Type_UPDATE) {
            debug_printf(DBG_INFO, "Received an update message\r\n");
            auto reply_msg = make_shared<SerMsg>();
            reply_msg->type(SerialMsg_Type_UPDATE);
            reply_msg->update_msg().type = UpdateMsg_Type_ACK;
            if(ser_msg->update_msg().type == UpdateMsg_Type_ACK || 
                ser_msg->update_msg().type == UpdateMsg_Type_ACKERR) {
                reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
            } else if(ser_msg->update_msg().type == UpdateMsg_Type_FIRST) {
                upd_pkt_cnt = -1;
                if(upd_file != nullptr) {
                    fclose(upd_file);
                }
                string upd_fname(ser_msg->update_msg().path); 
                upd_fname.append(".tmp");
                upd_file = fopen(upd_fname.c_str(), "we");
                mbedtls_sha256_init(&sha256_cxt);
                mbedtls_sha256_starts(&sha256_cxt, 0);
                if(upd_file == nullptr) {
                    reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
                    string err_reason("No Update File");
                    strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                            sizeof(reply_msg->update_msg().err_reason)); 
                } else {
                    if(check_upd_pkt_sha256(ser_msg->update_msg())) {
                        fwrite(ser_msg->update_msg().pld.bytes, 1, ser_msg->update_msg().pld.size, upd_file); 
                        mbedtls_sha256_update(&sha256_cxt, ser_msg->update_msg().pld.bytes, 
                                            ser_msg->update_msg().pld.size);
                        upd_pkt_cnt += 1;
                    } else {
                        reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
                        string err_reason("SHA256 packet checksum failed");
                        strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                    sizeof(reply_msg->update_msg().err_reason)); 
                    }
                }
			} else if(ser_msg->update_msg().type == UpdateMsg_Type_MIDDLE) {
				if(upd_file == nullptr) {
					ser_msg->update_msg().type = UpdateMsg_Type_ACKERR;
					string err_reason("No Update File");
					strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                sizeof(reply_msg->update_msg().err_reason)); 
				} else {
					if(check_upd_pkt_sha256(ser_msg->update_msg())) {
						fwrite(ser_msg->update_msg().pld.bytes, 1, ser_msg->update_msg().pld.size, upd_file);
						mbedtls_sha256_update(&sha256_cxt, ser_msg->update_msg().pld.bytes,
												ser_msg->update_msg().pld.size);
						upd_pkt_cnt += 1;
					} else {
						reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
						string err_reason("SHA256 packet checksum failed");
						strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                    sizeof(reply_msg->update_msg().err_reason)); 
					}
				}
			} else if(ser_msg->update_msg().type == UpdateMsg_Type_LAST) {
                debug_printf(DBG_INFO, "Received the last message\r\n");
				if(upd_file == nullptr) {
					reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
					string err_reason("No Update File");
					strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                sizeof(reply_msg->update_msg().err_reason)); 
				} else {
					if(check_upd_pkt_sha256(ser_msg->update_msg())) {
                        debug_printf(DBG_INFO, "Update's SHA256 checkum passes!\r\n");
						fwrite(ser_msg->update_msg().pld.bytes, 1, ser_msg->update_msg().pld.size, upd_file); 
						fclose(upd_file);
						mbedtls_sha256_update(&sha256_cxt, ser_msg->update_msg().pld.bytes, 
												ser_msg->update_msg().pld.size);
						reply_msg->update_msg().sha256_pkt.size = SHA256_SIZE;
						mbedtls_sha256_finish(&sha256_cxt, reply_msg->update_msg().sha256_upd.bytes); 
						if(ser_msg->update_msg().sha256_upd.size == 0) {
							reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
							string err_reason("No SHA256 checksum");
							strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                        sizeof(reply_msg->update_msg().err_reason));
							string upd_fname(ser_msg->update_msg().path); 
							upd_fname.append(".tmp");
                            upd_fname.erase(0, 4);
							fs->remove(upd_fname.c_str());
						} else if(memcmp(ser_msg->update_msg().sha256_upd.bytes, 
                                        reply_msg->update_msg().sha256_upd.bytes, ERR_MSG_SIZE) != 0) { 
							reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
							string err_reason("SHA256 checksum failed");
							strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                        sizeof(reply_msg->update_msg().err_reason)); 
							string upd_fname(ser_msg->update_msg().path); 
							upd_fname.append(".tmp");
                            upd_fname.erase(0, 4);
							fs->remove(upd_fname.c_str());
						} else {
							string upd_fname_tmp(ser_msg->update_msg().path); 
							upd_fname_tmp.append(".tmp");
                            upd_fname_tmp.erase(0, 4);
                            string upd_fname(ser_msg->update_msg().path); 
                            upd_fname.erase(0, 4);
                            fs->remove(upd_fname.c_str());
							fs->rename(upd_fname_tmp.c_str(), upd_fname.c_str());
							string upd_fname_sha256(ser_msg->update_msg().path); 
							upd_fname_sha256.append(".sha256");
							FILE *upd_file_sha256 = fopen(upd_fname_sha256.c_str(), "we");
							if(upd_file_sha256 != nullptr) {
								fwrite(ser_msg->update_msg().sha256_upd.bytes, 1, SHA256_SIZE, upd_file_sha256); 
								fclose(upd_file_sha256);
								upd_pkt_cnt += 1;
                                debug_printf(DBG_INFO, "Successfully finished writing update.\r\n");
							} else {
								reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
								string err_reason("SHA256 open failed");
								strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                            sizeof(reply_msg->update_msg().err_reason));   
							}
						}
					} else {
						reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
						string err_reason("SHA256 packet checksum failed");
						strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                    sizeof(reply_msg->update_msg().err_reason)); 
					}
				}
			} else {
				PORTABLE_ASSERT(false);
			}
            reply_msg->update_msg().pkt_cnt = upd_pkt_cnt;
            tx_ser_queue.enqueue_mail(reply_msg);
        } else if(ser_msg->type() == SerialMsg_Type_EXIT_KISS_MODE) {
            shared_mtx->lock();
            kiss_extended = true;
            background_queue->call(oled_mon_fn);
            shared_mtx->unlock();
        } else if(ser_msg->type() == SerialMsg_Type_ENTER_KISS_MODE) {
            shared_mtx->lock();
            kiss_extended = false;
            background_queue->call(oled_mon_fn);
            shared_mtx->unlock();
        } else if(ser_msg->type() == SerialMsg_Type_GET_CONFIG) {
            auto out_msg_sptr = make_shared<SerMsg>();
            out_msg_sptr->type(SerialMsg_Type_CONFIG);
            shared_mtx->lock();
            PORTABLE_ASSERT(radio_cb.valid);
            out_msg_sptr->sys_cfg() = radio_cb;
            shared_mtx->unlock();
            tx_ser_queue.enqueue_mail(out_msg_sptr);
            send_ack();
        } else if(ser_msg->type() == SerialMsg_Type_SET_CONFIG) {
            debug_printf(DBG_INFO, "Serial config received\r\n");
            if(!ser_msg->has_sys_cfg()) {
                send_error(string("No Configuration Sent!\r\n"));
            } else {
                shared_mtx->lock();
                PORTABLE_ASSERT(radio_cb.valid);
                radio_cb = ser_msg->sys_cfg();
                save_settings_to_flash();
                shared_mtx->unlock();
			    send_ack();
            }
        }
        else if(ser_msg->type() == SerialMsg_Type_GET_STATUS) {
            send_status();      
        }
        else if(ser_msg->type() == SerialMsg_Type_CLOCK_SET) {
            PORTABLE_ASSERT(ser_msg->has_clock_set());
            shared_mtx->lock();
            set_time(ser_msg->clock_set().time);
            shared_mtx->unlock();
            send_ack();
        }
        else if(ser_msg->type() == SerialMsg_Type_STAY_IN_MGT) {
            stay_in_management = true;           
            send_ack();
        }
        else if(ser_msg->type() == SerialMsg_Type_DEBUG_MSG) {
            send_ack();
        }
        else if(ser_msg->type() == SerialMsg_Type_DATA) {
            if(ser_msg->data_msg().type == DataMsg_Type_TX) {
                auto frame = make_shared<Frame>();
                frame->loadFromPB(ser_msg->data_msg());
                PORTABLE_ASSERT(radio_cb.valid);
                frame->setSender(radio_cb.address);
                frame->setStreamID();
                auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frame);
                unified_radio_evt_mail->enqueue_mail(radio_evt); 
                send_ack();
            } else if(ser_msg->data_msg().type == DataMsg_Type_KISSTX) {
                debug_printf(DBG_INFO, "Received a KISS frame on port %s of size %d\r\n",
                            port_name.c_str(), ser_msg->data_msg().payload.size); 
                //size_t tot_bytes = ser_msg->data_msg.payload.size;
                size_t max_pld_size = Frame::getKISSMaxSize();
                //size_t cur_bytes = 0;
                size_t frame_num = 0;
                size_t tot_frames = ceilf(static_cast<float>(ser_msg->data_msg().payload.size)/
                                            static_cast<float>(max_pld_size));
                uint8_t stream_id = Frame::createStreamID();
                vector<uint8_t> frags(ser_msg->data_msg().payload.size);
                memcpy(frags.data(), ser_msg->data_msg().payload.bytes, frags.size());
                // KEEP CHECKING THIS
                while(!frags.empty()) {
                    auto frag_end_iter = frags.size() < max_pld_size ? frags.begin()+frags.size() : 
                                            frags.begin()+max_pld_size;
                    vector<uint8_t> frag_buf(frags.begin(), frag_end_iter);
                    frags = vector<uint8_t>(frag_end_iter, frags.end());
                    auto frag = make_shared<DataMsg>();
                    *frag = data_msg_zero;
                    frag->type = DataMsg_Type_KISSTX;
                    frag->kiss_tot_frames = tot_frames;
                    frag->kiss_cur_frame = frame_num++;
                    frag->kiss_stream_id = stream_id;
                    frag->payload.size = frag_buf.size();
                    copy(frag_buf.begin(), frag_buf.end(), frag->payload.bytes);
                    auto frag_frame = make_shared<Frame>();
                    frag_frame->createFromKISS(*frag);
                    PORTABLE_ASSERT(radio_cb.valid);
                    frag_frame->setSender(radio_cb.address);
                    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frag_frame);
                    unified_radio_evt_mail->enqueue_mail(radio_evt);
                    debug_printf(DBG_INFO, "Enqueued a KISS fragment of size %d\r\n",
                                    frag->payload.size); 
                } 
#if 0 
                while(cur_bytes < tot_bytes) {
                    auto frag = make_shared<DataMsg>();
                    *frag = data_msg_zero;
                    frag->type = DataMsg_Type_KISSTX;
                    frag->kiss_tot_frames = tot_frames;
                    frag->kiss_cur_frame = frame_num++;
                    frag->kiss_stream_id = stream_id;
                    if(tot_bytes-cur_bytes <= max_pld_size) {
                        frag->payload.size = tot_bytes-cur_bytes;
                        memcpy(frag->payload.bytes, ser_msg->data_msg.payload.bytes+cur_bytes, //NOLINT
                                tot_bytes-cur_bytes);
                        cur_bytes += tot_bytes-cur_bytes;
                    } else {
                        frag->payload.size = max_pld_size;
                        memcpy(frag->payload.bytes, ser_msg->data_msg.payload.bytes+cur_bytes, //NOLINT
                                max_pld_size);
                        cur_bytes += max_pld_size;
                    }
                    auto frag_frame = make_shared<Frame>();
                    frag_frame->createFromKISS(*frag);
                    PORTABLE_ASSERT(radio_cb.valid);
                    frag_frame->setSender(radio_cb.address);
                    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frag_frame);
                    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt);
                    debug_printf(DBG_INFO, "Enqueued a KISS fragment of size %d\r\n",
                                    frag->payload.size); 
                }
#endif
            } else {
                printf("Packet type is %d\r\n", ser_msg->data_msg().type);
                PORTABLE_ASSERT(false);
            }      
        }
        else if(ser_msg->type() == SerialMsg_Type_REBOOT) {
            debug_printf(DBG_INFO, "Received reboot command\r\n");
            send_ack();
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_LOGS) {
            shared_mtx->lock();
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                DIR *log_dir = opendir("/fs/log");
                PORTABLE_ASSERT(log_dir);
                for(;;) {
                    struct dirent *dir_entry = readdir(log_dir);
                    if(dir_entry == nullptr) { break; }
                    stringstream fname;
                    fname << "/log/" << dir_entry->d_name; 
                    if(string(dir_entry->d_name) != "." && string(dir_entry->d_name) != "..") { 
                        debug_printf(DBG_INFO, "Deleting %s\r\n", fname.str().c_str());
                        int rem_err = fs->remove(fname.str().c_str());
                        if(rem_err != 0) {
                            debug_printf(DBG_WARN, "File remove failed with code %d\r\n", rem_err);
                        }
                    }
                }
            }
            shared_mtx->unlock();
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_BOOT_LOGS) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx->lock();
                fs->remove("boot_log.bin");
                shared_mtx->unlock();
            }
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_CFG) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx->lock();
                fs->remove("settings.bin");
                shared_mtx->unlock();
            }
            send_ack();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_READ_LOG) {
            debug_printf(DBG_INFO, "Read log found\r\n");
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx->lock();
				if(!reading_log) {
                    DIR *log_dir = opendir("/fs/log");
                    PORTABLE_ASSERT(log_dir);
                    for(;;) {
                        struct dirent *my_dirent = readdir(log_dir);
                        if(my_dirent == nullptr) { break; }
                        if(string(my_dirent->d_name) == "." || 
                            string(my_dirent->d_name) == "..") { continue; } 
                        string file_path;
                        file_path.append("/fs/log/");
                        file_path.append(my_dirent->d_name); 
                        debug_printf(DBG_INFO, "STUFF: %s\r\n", my_dirent->d_name); 
                        logfile_names.push_back(file_path);
                    }								
                    reading_log = true;
                    f = fopen(logfile_names[0].c_str(), "re");
                    if(f == nullptr) {
                        string err_msg = "Unable to open logfile ";
                        err_msg.append(logfile_names[0]);
                        err_msg.append("\r\n");
                        send_error(err_msg);
                    } else {
                        logfile_names.erase(logfile_names.begin());
                    }
				}
				auto cur_log_msg = make_shared<SerMsg>();
                // Need to have an actually-open filehandle here
                int err_ser = load_SerMsg(*cur_log_msg, *pser_rd); 
                debug_printf(DBG_INFO, "Serial Error is %d\r\n", err_ser);
                if(err_ser != 0) { // Go to the next file if it exists
                    if(logfile_names.empty()) {
                        auto reply_msg_sptr = make_shared<SerMsg>(); 
                        reply_msg_sptr->type(SerialMsg_Type_REPLY_LOG);
                        reply_msg_sptr->log_msg().valid = false;
                        tx_ser_queue.enqueue_mail(reply_msg_sptr);   
                        debug_printf(DBG_WARN, "Finished reading logs. Now rebooting...\r\n");
					    reboot_system();	 	
                    } else {
                        f = fopen(logfile_names[0].c_str(), "re");
                        PORTABLE_ASSERT(f);
                        logfile_names.erase(logfile_names.begin());   
                        string cur_line;
                        if(load_SerMsg(*cur_log_msg, *pser_rd) == 0) {
                            auto reply_msg_sptr = make_shared<SerMsg>();
                            reply_msg_sptr->type(SerialMsg_Type_REPLY_LOG);
                            reply_msg_sptr->log_msg().valid = true;
                            reply_msg_sptr->log_msg().count = line_count++;	
                            if(!cur_log_msg->has_log_msg()) {
                                send_error("Logfile entry has no log message\r\n");
                            }
                            reply_msg_sptr->log_msg() = cur_log_msg->log_msg();
                            tx_ser_queue.enqueue_mail(reply_msg_sptr);   
                        } else {
                            send_error("Logfile read error\r\n");
                        }		
                    }
				} else {
                    auto reply_msg_sptr = make_shared<SerMsg>();
                    reply_msg_sptr->type(SerialMsg_Type_REPLY_LOG);
                    reply_msg_sptr->log_msg().valid = true;
                    reply_msg_sptr->log_msg().count = line_count++;	
                    if(!cur_log_msg->has_log_msg()) {
                        send_error("Logfile entry has no log message\r\n");
                    }
                    reply_msg_sptr->log_msg() = cur_log_msg->log_msg();
                    reply_msg_sptr->log_msg().valid = true;
                    tx_ser_queue.enqueue_mail(reply_msg_sptr); 
				}
                shared_mtx->unlock();
            }
        }
        else if(ser_msg->type() == SerialMsg_Type_READ_BOOT_LOG) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx->lock();
				if(!reading_bootlog) {
					reading_bootlog = true;
					f = fopen("/fs/boot_log.bin", "re");
					PORTABLE_ASSERT(f);
				}
                auto cur_log_msg = make_shared<SerMsg>();
                int err_ser = load_SerMsg(*cur_log_msg, *pser_rd);
                if(err_ser != 0) {
                    auto reply_msg = make_shared<SerMsg>();
                    reply_msg->type(SerialMsg_Type_REPLY_BOOT_LOG);
                    reply_msg->boot_log_msg().valid = false;
                    auto reply_msg_sptr = make_shared<SerMsg>(*reply_msg);
                    tx_ser_queue.enqueue_mail(reply_msg_sptr); 
					debug_printf(DBG_WARN, "Now rebooting...\r\n");
					reboot_system();	    
				} else {
                    cur_log_msg->type(SerialMsg_Type_REPLY_BOOT_LOG);
                    cur_log_msg->boot_log_msg().count = line_count++;
                    auto reply_msg_sptr = make_shared<SerMsg>(*cur_log_msg);
                    tx_ser_queue.enqueue_mail(reply_msg_sptr);                               						
				}
                shared_mtx->unlock();
            }
        }		
        else {
            continue;
        }
    }
}
