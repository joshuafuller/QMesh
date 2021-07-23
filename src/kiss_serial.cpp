/*
QMesh
Copyright (C) 2021 Daniel R. Fay

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
#include "mbed.h"
#include <random>
#include <string>
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

extern EventQueue background_queue;

void print_memory_info();

static constexpr int ERR_MSG_SIZE = 32;
static constexpr int SHA256_SIZE = 32;

// debug_printf() uses this vector to determine which serial ports to send out
vector<KISSSerial *> kiss_sers;
Mutex kiss_sers_mtx;

static constexpr uint8_t FEND = 0xC0;
static constexpr uint8_t FESC = 0xDB;
static constexpr uint8_t TFEND = 0xDC;
static constexpr uint8_t TFESC = 0xDD;
static constexpr uint8_t SETHW = 0x06;
static constexpr uint8_t DATAPKT = 0x00;
static constexpr uint8_t EXITKISS = 0xFF;
static constexpr size_t MAX_MSG_SIZE = (SerialMsg_size+sizeof(crc_t))*2;

static auto compare_frame_crc(const vector<uint8_t> &buf) -> bool;
static auto compute_frame_crc(const vector<uint8_t> &buf) -> crc_t; 

static Mutex shared_mtx;

static SerialMsg ser_msg_zero = SerialMsg_init_zero;
static DataMsg data_msg_zero = DataMsg_init_zero;

auto load_SerMsg(SerMsg &ser_msg, FILE *f) -> read_ser_msg_err_t {
    size_t byte_read_count = 0;
    bool kiss_extended = false;
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = fgetc(f);
            if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        }
        if(cur_byte != FEND) {
            printf("KISS Packet sent\r\n");
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
                ser_msg = ser_msg_zero;
                ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
                return READ_SUCCESS;
            }  
            return READ_INVALID_KISS_ID;  
        }
    }
    // Pull in the main frame
    vector<uint8_t> buf;
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } 
        if(cur_byte == FESC) {
            cur_byte = fgetc(f);
            if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
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
        ser_msg = ser_msg_zero;
        pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size()-sizeof(crc_t));
        if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
            return DECODE_SER_MSG_ERR;
        }
    } else {
        ser_msg = ser_msg_zero;
        ser_msg.type(SerialMsg_Type_DATA);
        ser_msg.data_msg().type = DataMsg_Type_KISSTX;
        ser_msg.data_msg().payload.size = buf.size();
        memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


auto save_SerMsg(SerMsg &ser_msg, FILE *f, const bool kiss_data_msg) -> write_ser_msg_err_t {
    vector<uint8_t> comb_buf; //NOTE: we can probably just have a buf, not a buf and comb_buf
    // If we're not doing KISS, we want to send the whole serialized protobuf message.
    // OTOH, if we're doing KISS, we just want to send back the payload.
    if(!kiss_data_msg) {
        vector<uint8_t> buf(SerialMsg_size);
        pb_ostream_t stream = pb_ostream_from_buffer(buf.data(), buf.size()); 
        if(!pb_encode(&stream, SerialMsg_fields, &ser_msg)) {
            return ENCODE_SER_MSG_ERR;
        }
        comb_buf.resize(stream.bytes_written+sizeof(crc_t));
        memcpy(comb_buf.data(), buf.data(), stream.bytes_written);
        crc_t crc = compute_frame_crc(vector<uint8_t>(comb_buf.begin(), comb_buf.begin()+stream.bytes_written));
        memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t));
    } else {
        comb_buf.resize(ser_msg.data_msg().payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg().payload.bytes, ser_msg.data_msg().payload.size);
    }

    // Make it into a KISS frame and write it out
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(fputc(SETHW, f) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(fputc(DATAPKT, f) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint8_t i : comb_buf) {
        if(i == FEND) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(i == FESC) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFESC, f) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(fputc(i, f) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; } 
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
    MBED_ASSERT(buf.size() >= 2);
    crc_t crc = compute_frame_crc(vector<uint8_t>(buf.begin(), buf.end()-2));
    crc_t crc_actual = 0;
    crc_actual |= *(buf.end()-1);
    crc_actual <<= EIGHT_BITS;
    crc_actual |= *(buf.end()-2);
    return crc == crc_actual;
}


void send_status() {
    kiss_sers_mtx.lock();
    for(auto & kiss_ser : kiss_sers) {
        kiss_ser->send_status();
    }
    kiss_sers_mtx.unlock();
}


auto KISSSerial::save_SerMsg(SerMsg &ser_msg, FILE *f, const bool kiss_data_msg) -> write_ser_msg_err_t {
    vector<uint8_t> comb_buf;
    // If we're not doing KISS, we want to send the whole serialized protobuf message.
    // OTOH, if we're doing KISS, we just want to send back the payload.
    if(!kiss_data_msg) {
        vector<uint8_t> buf(SerialMsg_size);
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
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(fputc(SETHW, f) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(fputc(DATAPKT, f) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint8_t i : comb_buf) {
        if(i == FEND) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(i == FESC) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFESC, f) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(fputc(i, f) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; } 
    return WRITE_SUCCESS;
}


static constexpr int SER_THREAD_STACK_SIZE = 8192;
KISSSerial::KISSSerial(const string &my_port_name, const ser_port_type_t ser_port_type) {
    past_log_msg = ser_msg_zero;
    using_stdio = true;
    kiss_extended = true;
    port_type = ser_port_type;
    hc05 = false;
    port_name = my_port_name;

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


static constexpr uint32_t SER_BAUD_RATE = 230400;
static constexpr uint32_t BT_BAUD_RATE = 38400;
KISSSerial::KISSSerial(PinName tx, PinName rx, const string &my_port_name, 
                        const ser_port_type_t ser_port_type) {
    en_pin = nullptr;
    state_pin = nullptr;
    past_log_msg = ser_msg_zero;
    tx_port = tx;
    rx_port = rx;
    kiss_extended = true;
    ser = new UARTSerial(tx_port, rx_port, SER_BAUD_RATE);
    MBED_ASSERT(ser);
    using_stdio = false;
    port_type = ser_port_type;
    port_name = my_port_name;
    hc05 = false;

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


static constexpr int QUARTER_SECOND = 250;
static constexpr int HALF_SECOND = 500;
static constexpr int REPLY_STR_SIZE = 64;
void KISSSerial::configure_hc05() {
    vector<char> reply_str(REPLY_STR_SIZE);
    *en_pin = 1;
    while(true) { };
    ThisThread::sleep_for(QUARTER_SECOND);
    ser->set_baud(BT_BAUD_RATE);
    ThisThread::sleep_for(QUARTER_SECOND);
    FILE *ser_fh = fdopen(ser, "rw");
    printf("testing\r\n");
    // Reset the module's configuration
    string reset_cmd("AT+ORGL\r\n");
    fprintf(ser_fh, "%s", reset_cmd.c_str());
    printf("%s", reset_cmd.c_str());
    fgets(reply_str.data(), REPLY_STR_SIZE, ser_fh);
    printf("%s", reply_str.data());
    ThisThread::sleep_for(HALF_SECOND);
    // Change the name
    string bt_name_cmd("AT+NAME=");
    bt_name_cmd.append(port_name);
    bt_name_cmd.append("\r\n");
    fprintf(ser_fh, "%s", bt_name_cmd.c_str());
    printf("%s", bt_name_cmd.c_str());
    ThisThread::sleep_for(QUARTER_SECOND);
#if 0
    string baud_cmd("AT+UART=38400,0,0,\r\n");
    fprintf(ser_fh, "%s", baud_cmd.c_str());    
    ThisThread::sleep_for(QUARTER_SECOND);
#endif
    //ser->set_baud(38400);
    string reboot_cmd("AT+RESET\r\n");
    fprintf(ser_fh, "%s", reboot_cmd.c_str());
    ThisThread::sleep_for(QUARTER_SECOND);
    string init_cmd("AT+INIT\r\n");
    fprintf(ser_fh, "%s", init_cmd.c_str());
    ThisThread::sleep_for(QUARTER_SECOND);
    *en_pin = 0;
    printf("Done with configuration\r\n");

}


KISSSerial::KISSSerial(PinName tx, PinName rx, PinName En, PinName State,
            const string &my_port_name, const ser_port_type_t ser_port_type) {
    past_log_msg = ser_msg_zero;
    kiss_extended = true;
    port_name = my_port_name;
    hc05 = true;
    en_pin = new DigitalOut(En);
    *en_pin = 1;
    state_pin = new DigitalIn(State);                
    tx_port = tx;
    rx_port = rx;
    ser = new UARTSerial(tx_port, rx_port, BT_BAUD_RATE);
    MBED_ASSERT(ser);
    using_stdio = false;
    port_type = ser_port_type;

    //configure_hc05();

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, SER_THREAD_STACK_SIZE, nullptr, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


void KISSSerial::sleep() {
    delete ser;
}


void KISSSerial::wake() {
    if(ser == nullptr) {
        ser = new UARTSerial(tx_port, rx_port, SER_BAUD_RATE);
        MBED_ASSERT(ser);
    }
    if(hc05) {
        configure_hc05();
    }
}


KISSSerial::~KISSSerial() {
    kiss_sers_mtx.lock();
    kiss_sers.erase(find(kiss_sers.begin(), kiss_sers.end(), this));
    kiss_sers_mtx.unlock();
    rx_ser_thread->join();
    delete rx_ser_thread;
    tx_ser_thread->join();
    delete tx_ser_thread;
    delete ser;
    delete en_pin;
    delete state_pin;
}


void KISSSerial::enqueue_msg(shared_ptr<SerMsg> ser_msg_sptr) { //NOLINT
    if(ser_msg_sptr->type() == SerialMsg_Type_DATA) {
        MBED_ASSERT(ser_msg_sptr->has_data_msg());
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
            (port_type == DEBUG_PORT || port_type == VOICE_PORT)) {
            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg_sptr);  
        } else if (!ser_msg_sptr->data_msg().voice && 
            (port_type == DEBUG_PORT || port_type == APRS_PORT)) {
            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg_sptr);  
        }
    } else if(kiss_extended) {
        enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg_sptr);
    }
}


auto KISSSerial::load_SerMsg(SerMsg &ser_msg, FILE *f) -> read_ser_msg_err_t {
    size_t byte_read_count = 0;
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = fgetc(f);
            if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        }
        if(cur_byte != FEND) {
            if(cur_byte == SETHW) {
                kiss_extended = true;
                break;
            } 
            if(cur_byte == DATAPKT) {
                kiss_extended = false;
                break;
            } 
            if(cur_byte == EXITKISS) {
                ser_msg = ser_msg_zero;
                ser_msg.type(SerialMsg_Type_EXIT_KISS_MODE);
                return READ_SUCCESS;
            }  
            return READ_INVALID_KISS_ID;
        }
    }
    // Pull in the main frame
    vector<uint8_t> buf;
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } 
        if(cur_byte == FESC) {
            cur_byte = fgetc(f);
            if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
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
        ser_msg = ser_msg_zero;
        pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size()-sizeof(crc_t));
        if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
            return DECODE_SER_MSG_ERR;
        }
    } else {
        ser_msg = ser_msg_zero;
        ser_msg.type(SerialMsg_Type_DATA);
        ser_msg.data_msg().type = DataMsg_Type_KISSTX;
        ser_msg.data_msg().payload.size = buf.size();
        memcpy(ser_msg.data_msg().payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


void KISSSerial::tx_serial_thread_fn() {
    FILE *kiss_ser = nullptr;
    if(using_stdio) {
        kiss_ser = stdout;
    } else {
        kiss_ser = fdopen(ser, "w");
    }
    for(;;) {
        auto ser_msg_sptr = dequeue_mail<shared_ptr<SerMsg>>(tx_ser_queue);
        // In KISS mode, we only send out data packets in KISS format.
        if(!kiss_extended) {
            // Silently drop anything that isn't a KISSRX frame when we're in KISS mode
            if(ser_msg_sptr->type() == SerialMsg_Type_DATA && ser_msg_sptr->has_data_msg() && 
                ser_msg_sptr->data_msg().type == DataMsg_Type_KISSRX) {
                save_SerMsg(*ser_msg_sptr, kiss_ser, true);
            } else {
                continue;
            }           
        } else {
            save_SerMsg(*ser_msg_sptr, kiss_ser, false);  
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
        MBED_ASSERT(false);
    }
    ser_msg->status().tx_full = tx_frame_mail.full();
    ser_msg->status().time = time(nullptr);
    auto *display_file = fopen("/fs/display.off", "r");
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
    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg);
}  


void KISSSerial::send_ack() {
    auto ser_msg_sptr = make_shared<SerMsg>();
    ser_msg_sptr->type(SerialMsg_Type_ACK);
    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg_sptr);    
}  


void KISSSerial::send_error(const string &err_str) {
    auto ser_msg_sptr = make_shared<SerMsg>();
    ser_msg_sptr->type(SerialMsg_Type_ERR);
    strncpy(ser_msg_sptr->error_msg().msg, err_str.c_str(), sizeof(ser_msg_sptr->error_msg().msg));
    debug_printf(DBG_WARN, "%s", err_str.c_str());
    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, ser_msg_sptr);   
}  


Mutex stream_id_lock;
mt19937 stream_id_rng; //NOLINT
static atomic<int> last_stream_id;
static constexpr int MAX_UINT8_VAL = 255;
static uniform_int_distribution<uint8_t> timing_off_dist(0, MAX_UINT8_VAL);  
static auto get_stream_id() -> uint8_t;
static auto get_stream_id() -> uint8_t {
    stream_id_lock.lock();
    uint8_t new_stream_id = 0;
    do {
        new_stream_id = timing_off_dist(stream_id_rng);
    } while(new_stream_id == last_stream_id);
    last_stream_id = new_stream_id;
    stream_id_lock.unlock();
    return new_stream_id;
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
    past_log_msg = ser_msg_zero;
    FILE *kiss_ser = nullptr;
    if(using_stdio) {
        kiss_ser = stdin;
    } else {
        kiss_ser = fdopen(ser, "r");
    }
    auto ser_msg = make_shared<SerMsg>();
    int err = 0;
    for(;;) {
        *ser_msg = ser_msg_zero;
        err = load_SerMsg(*ser_msg, kiss_ser);
        if(err != 0) {
            debug_printf(DBG_WARN, "Error in reading serial port entry. Error %d\r\n", err);
            if(err == CRC_ERR) {
                auto reply_msg = make_shared<SerMsg>();
                reply_msg->type(SerialMsg_Type_ERR);
                reply_msg->error_msg().type = ErrorMsg_Type_CRC_ERR;
                string err_reason("CRC Error");
                strncpy(reply_msg->error_msg().msg, err_reason.c_str(), sizeof(reply_msg->error_msg().msg));
                enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg);
            }
            continue;
        }
        if(ser_msg->type() == SerialMsg_Type_TURN_OLED_OFF) {
            debug_printf(DBG_INFO, "Received a request to turn OFF the OLED display\r\n");
            auto *disp_file = fopen("/fs/display.off", "w");
            MBED_ASSERT(disp_file != nullptr);
            fclose(disp_file);
            oled->displayOff();
        }
        if(ser_msg->type() == SerialMsg_Type_TURN_OLED_ON) {
            debug_printf(DBG_INFO, "Received a request to turn ON the OLED display\r\n");
            fs.remove("display.off");
            auto *disp_file = fopen("/fs/display.off", "r");
            MBED_ASSERT(disp_file == nullptr);
            oled->displayOn();
        }        
        if(ser_msg->type() == SerialMsg_Type_VERSION) {
            debug_printf(DBG_INFO, "Received a FW version request message\r\n");
            auto reply_msg = make_shared<SerMsg>();
            reply_msg->type(SerialMsg_Type_VERSION);
            string compile_str = getFlashCompileString();
            debug_printf(DBG_INFO, "Sending %s\r\n", compile_str.c_str());
            strncpy(reply_msg->ver_msg().msg, compile_str.c_str(), sizeof(reply_msg->ver_msg().msg));
            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg);
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
                upd_file = fopen(upd_fname.c_str(), "w");
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
							fs.remove(upd_fname.c_str());
						} else if(memcmp(ser_msg->update_msg().sha256_upd.bytes, 
                                        reply_msg->update_msg().sha256_upd.bytes, ERR_MSG_SIZE) != 0) { 
							reply_msg->update_msg().type = UpdateMsg_Type_ACKERR;
							string err_reason("SHA256 checksum failed");
							strncpy(reply_msg->update_msg().err_reason, err_reason.c_str(), 
                                        sizeof(reply_msg->update_msg().err_reason)); 
							string upd_fname(ser_msg->update_msg().path); 
							upd_fname.append(".tmp");
                            upd_fname.erase(0, 4);
							fs.remove(upd_fname.c_str());
						} else {
							string upd_fname_tmp(ser_msg->update_msg().path); 
							upd_fname_tmp.append(".tmp");
                            upd_fname_tmp.erase(0, 4);
                            string upd_fname(ser_msg->update_msg().path); 
                            upd_fname.erase(0, 4);
                            fs.remove(upd_fname.c_str());
							fs.rename(upd_fname_tmp.c_str(), upd_fname.c_str());
							string upd_fname_sha256(ser_msg->update_msg().path); 
							upd_fname_sha256.append(".sha256");
							FILE *upd_file_sha256 = fopen(upd_fname_sha256.c_str(), "w");
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
				MBED_ASSERT(false);
			}
            reply_msg->update_msg().pkt_cnt = upd_pkt_cnt;
            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg);
        } else if(ser_msg->type() == SerialMsg_Type_EXIT_KISS_MODE) {
            shared_mtx.lock();
            kiss_extended = true;
            background_queue.call(oled_mon_fn);
            shared_mtx.unlock();
        } else if(ser_msg->type() == SerialMsg_Type_ENTER_KISS_MODE) {
            shared_mtx.lock();
            kiss_extended = false;
            background_queue.call(oled_mon_fn);
            shared_mtx.unlock();
        } else if(ser_msg->type() == SerialMsg_Type_GET_CONFIG) {
            auto out_msg_sptr = make_shared<SerMsg>();
            out_msg_sptr->type(SerialMsg_Type_CONFIG);
            out_msg_sptr->sys_cfg() = radio_cb;
            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, out_msg_sptr);
            send_ack();
            reboot_system();
        } else if(ser_msg->type() == SerialMsg_Type_SET_CONFIG) {
            debug_printf(DBG_INFO, "Serial config received\r\n");
            if(!ser_msg->has_sys_cfg()) {
                send_error(string("No Configuration Sent!\r\n"));
            } else {
                shared_mtx.lock();
                radio_cb = ser_msg->sys_cfg();
                save_settings_to_flash();
                shared_mtx.unlock();
			    send_ack();
            }
        }
        else if(ser_msg->type() == SerialMsg_Type_GET_STATUS) {
            send_status();      
        }
        else if(ser_msg->type() == SerialMsg_Type_CLOCK_SET) {
            MBED_ASSERT(ser_msg->has_clock_set());
            shared_mtx.lock();
            set_time(ser_msg->clock_set().time);
            shared_mtx.unlock();
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
                frame->setSender(radio_cb.address);
                frame->setStreamID(get_stream_id());
                auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frame);
                enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt); 
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
                uint8_t stream_id = get_stream_id();
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
                    frag_frame->setSender(radio_cb.address);
                    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frag_frame);
                    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt);
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
                    frag_frame->setSender(radio_cb.address);
                    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frag_frame);
                    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt);
                    debug_printf(DBG_INFO, "Enqueued a KISS fragment of size %d\r\n",
                                    frag->payload.size); 
                }
#endif
            } else {
                printf("Packet type is %d\r\n", ser_msg->data_msg().type);
                MBED_ASSERT(false);
            }      
        }
        else if(ser_msg->type() == SerialMsg_Type_REBOOT) {
            debug_printf(DBG_INFO, "Received reboot command\r\n");
            send_ack();
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_LOGS) {
            shared_mtx.lock();
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                DIR *log_dir = opendir("/fs/log");
                MBED_ASSERT(log_dir);
                for(;;) {
                    struct dirent *dir_entry = readdir(log_dir);
                    if(dir_entry == nullptr) { break; }
                    stringstream fname;
                    fname << "/log/" << dir_entry->d_name; 
                    if(string(dir_entry->d_name) != "." && string(dir_entry->d_name) != "..") { 
                        debug_printf(DBG_INFO, "Deleting %s\r\n", fname.str().c_str());
                        int rem_err = fs.remove(fname.str().c_str());
                        if(rem_err != 0) {
                            debug_printf(DBG_WARN, "File remove failed with code %d\r\n", rem_err);
                        }
                    }
                }
            }
            shared_mtx.unlock();
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_BOOT_LOGS) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx.lock();
                fs.remove("boot_log.bin");
                shared_mtx.unlock();
            }
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type() == SerialMsg_Type_ERASE_CFG) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx.lock();
                fs.remove("settings.bin");
                shared_mtx.unlock();
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
                shared_mtx.lock();
				if(!reading_log) {
                    DIR *log_dir = opendir("/fs/log");
                    MBED_ASSERT(log_dir);
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
                    f = fopen(logfile_names[0].c_str(), "r");
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
                int err_ser = load_SerMsg(*cur_log_msg, f); 
                debug_printf(DBG_INFO, "Serial Error is %d\r\n", err_ser);
                if(err_ser != 0) { // Go to the next file if it exists
                    if(logfile_names.empty()) {
                        auto reply_msg_sptr = make_shared<SerMsg>(); 
                        reply_msg_sptr->type(SerialMsg_Type_REPLY_LOG);
                        reply_msg_sptr->log_msg().valid = false;
                        enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg_sptr);   
                        debug_printf(DBG_WARN, "Finished reading logs. Now rebooting...\r\n");
					    reboot_system();	 	
                    } else {
                        f = fopen(logfile_names[0].c_str(), "r");
                        MBED_ASSERT(f);
                        logfile_names.erase(logfile_names.begin());   
                        string cur_line;
                        if(load_SerMsg(*cur_log_msg, f) == 0) {
                            auto reply_msg_sptr = make_shared<SerMsg>();
                            reply_msg_sptr->type(SerialMsg_Type_REPLY_LOG);
                            reply_msg_sptr->log_msg().valid = true;
                            reply_msg_sptr->log_msg().count = line_count++;	
                            if(!cur_log_msg->has_log_msg()) {
                                send_error("Logfile entry has no log message\r\n");
                            }
                            reply_msg_sptr->log_msg() = cur_log_msg->log_msg();
                            enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg_sptr);   
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
                    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg_sptr); 
				}
                shared_mtx.unlock();
            }
        }
        else if(ser_msg->type() == SerialMsg_Type_READ_BOOT_LOG) {
            stay_in_management = true;
            while(current_mode == system_state_t::BOOTING) { };
            if(current_mode == system_state_t::MANAGEMENT) {
                shared_mtx.lock();
				if(!reading_bootlog) {
					reading_bootlog = true;
					f = fopen("/fs/boot_log.bin", "r");
					MBED_ASSERT(f);
				}
                auto cur_log_msg = make_shared<SerMsg>();
                int err_ser = load_SerMsg(*cur_log_msg, f);
                if(err_ser != 0) {
                    auto reply_msg = make_shared<SerMsg>();
                    reply_msg->type(SerialMsg_Type_REPLY_BOOT_LOG);
                    reply_msg->boot_log_msg().valid = false;
                    auto reply_msg_sptr = make_shared<SerMsg>(*reply_msg);
                    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg_sptr); 
					debug_printf(DBG_WARN, "Now rebooting...\r\n");
					reboot_system();	    
				} else {
                    cur_log_msg->type(SerialMsg_Type_REPLY_BOOT_LOG);
                    cur_log_msg->boot_log_msg().count = line_count++;
                    auto reply_msg_sptr = make_shared<SerMsg>(*cur_log_msg);
                    enqueue_mail<shared_ptr<SerMsg>>(tx_ser_queue, reply_msg_sptr);                               						
				}
                shared_mtx.unlock();
            }
        }		
        else {
            continue;
        }
    }
}
