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
#include <stdio.h>
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <string.h>
#include "USBSerial.h"

extern EventQueue background_queue;

void print_memory_info();

// debug_printf() uses this vector to determine which serial ports to send out
vector<KISSSerial *> kiss_sers;
Mutex kiss_sers_mtx;

static const uint8_t FEND = 0xC0;
static const uint8_t FESC = 0xDB;
static const uint8_t TFEND = 0xDC;
static const uint8_t TFESC = 0xDD;
static const uint8_t SETHW = 0x06;
static const uint8_t DATAPKT = 0x00;
static const uint8_t EXITKISS = 0xFF;
static const size_t MAX_MSG_SIZE = (SerialMsg_size+sizeof(crc_t))*2;

static crc_t compute_frame_crc(const uint8_t *buf, const size_t buf_size); 
static bool compare_frame_crc(const uint8_t *buf, const size_t buf_size);

static Mutex shared_mtx;

static SerialMsg ser_msg_zero = SerialMsg_init_zero;
static DataMsg data_msg_zero = DataMsg_init_zero;


read_ser_msg_err_t load_SerialMsg(SerialMsg &ser_msg, FILE *f) {
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
            cur_byte &= 0x0F;
            if(cur_byte == SETHW) {
                kiss_extended = true;
                break;
            } else if(cur_byte == DATAPKT) {
                kiss_extended = false;
                break;
            } else if(cur_byte == EXITKISS) {
                ser_msg = ser_msg_zero;
                ser_msg.type = SerialMsg_Type_EXIT_KISS_MODE;
                return READ_SUCCESS;
            } else {
                return READ_INVALID_KISS_ID;
            }
        }
    }
    // Pull in the main frame
    vector<uint8_t> buf(0);
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } else if(cur_byte == FESC) {
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
            buf.push_back((uint8_t) cur_byte);
        }
    }
    if(kiss_extended) {
        // Check the CRC
        if(!compare_frame_crc(buf.data(), buf.size())) {
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
        ser_msg.type = SerialMsg_Type_DATA;
        ser_msg.has_data_msg = true;
        ser_msg.data_msg.type = DataMsg_Type_KISSTX;
        ser_msg.data_msg.payload.size = buf.size();
        memcpy(ser_msg.data_msg.payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


write_ser_msg_err_t save_SerialMsg(const SerialMsg &ser_msg, FILE *f, const bool kiss_data_msg) {
    vector<uint8_t> comb_buf;
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
        crc_t crc = compute_frame_crc(comb_buf.data(), stream.bytes_written);
        memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t));
    } else {
        comb_buf.resize(ser_msg.data_msg.payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg.payload.bytes, ser_msg.data_msg.payload.size);
    }

    // Make it into a KISS frame and write it out
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(fputc(SETHW, f) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(fputc(DATAPKT, f) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint32_t i = 0; i < comb_buf.size(); i++) {
        if(comb_buf[i] == FEND) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(comb_buf[i] == FESC) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFESC, f) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(fputc(comb_buf[i], f) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; } 
    return WRITE_SUCCESS;
}


static crc_t compute_frame_crc(const uint8_t *buf, const size_t buf_size) {
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    union {
        uint32_t gen_crc;
        crc_t crc_chunk[2];
    } crc;
    crc.gen_crc = 0;
    ct.compute(buf, buf_size, &crc.gen_crc);
    return crc.crc_chunk[0];
}


static bool compare_frame_crc(const uint8_t *buf, const size_t buf_size) {
    crc_t crc = compute_frame_crc(buf, buf_size-2);
    crc_t crc_actual;
    memcpy(&crc_actual, buf+buf_size-sizeof(crc), sizeof(crc));
    return crc == crc_actual;
}


void send_status(void) {
    kiss_sers_mtx.lock();
    for(vector<KISSSerial *>::iterator iter = kiss_sers.begin(); iter != kiss_sers.end(); iter++) {
        (*iter)->send_status();
    }
    kiss_sers_mtx.unlock();
}


write_ser_msg_err_t KISSSerial::save_SerialMsg(const SerialMsg &ser_msg, FILE *f, const bool kiss_data_msg) {
    vector<uint8_t> comb_buf;
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
        crc_t crc = compute_frame_crc(comb_buf.data(), stream.bytes_written);
        memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t));
    } else {
        comb_buf.resize(ser_msg.data_msg.payload.size);
        memcpy(comb_buf.data(), ser_msg.data_msg.payload.bytes, ser_msg.data_msg.payload.size);
    }

    // Make it into a KISS frame and write it out
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(fputc(SETHW, f) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(fputc(DATAPKT, f) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint32_t i = 0; i < comb_buf.size(); i++) {
        if(comb_buf[i] == FEND) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
        } else if(comb_buf[i] == FESC) {
            if(fputc(FESC, f) == EOF) { return WRITE_SER_MSG_ERR; }
            if(fputc(TFESC, f) == EOF) { return WRITE_SER_MSG_ERR; }            
        } else {
            if(fputc(comb_buf[i], f) == EOF) { return WRITE_SER_MSG_ERR; } 
        }
    }
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; } 
    return WRITE_SUCCESS;
}


KISSSerial::KISSSerial(const string &my_port_name, const ser_port_type_t ser_port_type) {
    using_stdio = true;
    kiss_extended = true;
    port_type = ser_port_type;
    hc05 = false;
    port_name = my_port_name;

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


KISSSerial::KISSSerial(PinName tx, PinName rx, const string &my_port_name, 
                        const ser_port_type_t ser_port_type) {
    tx_port = tx;
    rx_port = rx;
    ser = new UARTSerial(tx_port, rx_port, 230400);
    MBED_ASSERT(ser);
    using_stdio = false;
    kiss_extended = true;
    port_type = ser_port_type;
    port_name = my_port_name;
    hc05 = false;

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


void KISSSerial::configure_hc05(void) {
    *en_pin = 1;
    ThisThread::sleep_for(250);
    ser->set_baud(9600);
    ThisThread::sleep_for(250);
    FILE *ser_fh = fdopen(ser, "w");
    // Reset the module's configuration
    string reset_cmd("AT+ORGL\r\n");
    fprintf(ser_fh, "%s", reset_cmd.c_str());
    printf("%s", reset_cmd.c_str());
    ThisThread::sleep_for(500);
    // Change the name
    string bt_name_cmd("AT+NAME=");
    bt_name_cmd.append(port_name);
    bt_name_cmd.append("\r\n");
    fprintf(ser_fh, "%s", bt_name_cmd.c_str());
    printf("%s", bt_name_cmd.c_str());
    // Change the baudrate
    //string baud_cmd("AT+UART=115200,0,0\r\n");
    ThisThread::sleep_for(250);
    //ser->set_baud(115200);
    fprintf(ser_fh, "%s", bt_name_cmd.c_str());
    ThisThread::sleep_for(250);
    *en_pin = 0;
}


KISSSerial::KISSSerial(PinName tx, PinName rx, PinName En, PinName State,
            const string &my_port_name, const ser_port_type_t ser_port_type) {
    port_name = my_port_name;
    hc05 = true;
    en_pin = new DigitalOut(En);
    *en_pin = 0;
    state_pin = new DigitalIn(State);                
    tx_port = tx;
    rx_port = rx;
    ser = new UARTSerial(tx_port, rx_port, 9600);
    MBED_ASSERT(ser);
    using_stdio = false;
    kiss_extended = true;
    port_type = ser_port_type;

    //configure_hc05();

    string rx_ser_name("RX-SERIAL-");
    rx_ser_name.append(port_name);
    rx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, rx_ser_name.c_str());
    string tx_ser_name("TX-SERIAL-");
    tx_ser_name.append(port_name);
    tx_ser_thread = new Thread(osPriorityNormal, 8192, NULL, tx_ser_name.c_str());

    kiss_sers_mtx.lock();
    kiss_sers.push_back(this);
    kiss_sers_mtx.unlock();

    tx_ser_thread->start(callback(this, &KISSSerial::tx_serial_thread_fn));
    rx_ser_thread->start(callback(this, &KISSSerial::rx_serial_thread_fn));
}


void KISSSerial::sleep(void) {
    if(ser) {
        delete ser;
    }
}


void KISSSerial::wake(void) {
    if(!ser) {
        ser = new UARTSerial(tx_port, rx_port, 230000);
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
    if(ser) {
        delete ser;
    }
    if(hc05) {
        delete en_pin;
        delete state_pin;
    }
}


void KISSSerial::enqueue_msg(shared_ptr<SerialMsg> ser_msg_sptr) {
    if(ser_msg_sptr->type == SerialMsg_Type_DATA) {
        MBED_ASSERT(ser_msg_sptr->has_data_msg);
        auto out_ser_msg = make_shared<SerialMsg>();
        *out_ser_msg = ser_msg_zero;
        *out_ser_msg = *ser_msg_sptr;
        if(!kiss_extended) {
            if(out_ser_msg->data_msg.type == DataMsg_Type_TX) {
                out_ser_msg->data_msg.type = DataMsg_Type_KISSTX;
            }
            if(out_ser_msg->data_msg.type == DataMsg_Type_RX) {
                out_ser_msg->data_msg.type = DataMsg_Type_KISSRX;
            }
        }
        if(ser_msg_sptr->data_msg.voice && 
            (port_type == DEBUG_PORT || port_type == VOICE_PORT)) {
            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);  
        } else if (!ser_msg_sptr->data_msg.voice && 
            (port_type == DEBUG_PORT || port_type == APRS_PORT)) {
            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);  
        }
    } else if(kiss_extended) {
        enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);
    }
}


read_ser_msg_err_t KISSSerial::load_SerialMsg(SerialMsg &ser_msg, FILE *f) {
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
            } else if(cur_byte == DATAPKT) {
                kiss_extended = false;
                break;
            } else if(cur_byte == EXITKISS) {
                ser_msg = ser_msg_zero;
                ser_msg.type = SerialMsg_Type_EXIT_KISS_MODE;
                return READ_SUCCESS;
            } else {
                return READ_INVALID_KISS_ID;
            }
        }
    }
    // Pull in the main frame
    vector<uint8_t> buf(0);
    for(;;) {
        int cur_byte = fgetc(f);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } else if(cur_byte == FESC) {
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
            buf.push_back((uint8_t) cur_byte);
        }
    }
    if(kiss_extended) {
        // Check the CRC
        if(!compare_frame_crc(buf.data(), buf.size())) {
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
        ser_msg.type = SerialMsg_Type_DATA;
        ser_msg.has_data_msg = true;
        ser_msg.data_msg.type = DataMsg_Type_KISSTX;
        ser_msg.data_msg.payload.size = buf.size();
        memcpy(ser_msg.data_msg.payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


void KISSSerial::tx_serial_thread_fn(void) {
    FILE *kiss_ser;
    if(using_stdio) {
        kiss_ser = stdout;
    } else {
        kiss_ser = fdopen(ser, "w");
    }
    for(;;) {
        auto ser_msg_sptr = dequeue_mail<shared_ptr<SerialMsg>>(tx_ser_queue);
        // In KISS mode, we only send out data packets in KISS format.
        if(!kiss_extended) {
            // Silently drop anything that isn't a KISSRX frame when we're in KISS mode
            if(ser_msg_sptr->type == SerialMsg_Type_DATA && ser_msg_sptr->has_data_msg && 
                ser_msg_sptr->data_msg.type == DataMsg_Type_KISSRX) {
                save_SerialMsg(*ser_msg_sptr, kiss_ser, true);
            } else {
                continue;
            }           
        } else {
            save_SerialMsg(*ser_msg_sptr, kiss_ser, false);  
        }
    }
}

/**
 * Sends the current status.
 */
void KISSSerial::send_status(void) {
    auto ser_msg = make_shared<SerialMsg>();
    *ser_msg = ser_msg_zero;
    ser_msg->type = SerialMsg_Type_STATUS;
    ser_msg->has_status = true;
    if(current_mode == BOOTING) {
        ser_msg->status.status = StatusMsg_Status_BOOTING;      
    } else if(current_mode == MANAGEMENT) {
        ser_msg->status.status = StatusMsg_Status_MANAGEMENT;     
    } else if(current_mode == RUNNING) {
        ser_msg->status.status = StatusMsg_Status_RUNNING;
    } else {
        MBED_ASSERT(false);
    }
    if(tx_frame_mail.full()) {
        ser_msg->status.tx_full = true;
    } else {
        ser_msg->status.tx_full = false;
    }
    ser_msg->status.time = time(NULL);
    //auto ser_msg_sptr = make_shared<SerialMsg>(*);
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg);
}  


void KISSSerial::send_ack(void) {
    auto ser_msg_sptr = make_shared<SerialMsg>();
    *ser_msg_sptr = ser_msg_zero;
    ser_msg_sptr->type = SerialMsg_Type_ACK;
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);    
}  


void KISSSerial::send_error(const string &err_str) {
    auto ser_msg_sptr = make_shared<SerialMsg>();
    *ser_msg_sptr = ser_msg_zero;
    ser_msg_sptr->type = SerialMsg_Type_ERR;
    ser_msg_sptr->has_error_msg = true;
    size_t str_len = err_str.size() < 256 ? err_str.size() : 256;
    memcpy((char *) ser_msg_sptr->error_msg.msg, err_str.c_str(), str_len);
    debug_printf(DBG_WARN, "%s", err_str.c_str());
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);   
}  


Mutex stream_id_lock;
mt19937 stream_id_rng;
static atomic<int> last_stream_id;
static uniform_int_distribution<uint8_t> timing_off_dist(0, 255);  
static uint8_t get_stream_id(void);
static uint8_t get_stream_id(void) {
    stream_id_lock.lock();
    uint8_t new_stream_id;
    do {
        new_stream_id = timing_off_dist(stream_id_rng);
    } while(new_stream_id == last_stream_id);
    last_stream_id = new_stream_id;
    stream_id_lock.unlock();
    return new_stream_id;
}


void KISSSerial::rx_serial_thread_fn(void) {
	FILE *f = NULL;
	int line_count = 0;
	bool reading_log = false;
	bool reading_bootlog = false;
    past_log_msg = ser_msg_zero;
    FILE *kiss_ser;
    if(using_stdio) {
        kiss_ser = stdin;
    } else {
        kiss_ser = fdopen(ser, "r");
    }
    auto ser_msg = make_shared<SerialMsg>();
    int err;
    for(;;) {
        *ser_msg = ser_msg_zero;
        err = load_SerialMsg(*ser_msg, kiss_ser);
        if(err != 0) {
            debug_printf(DBG_WARN, "Error in reading serial port entry. Error %d\r\n", err);
        }
        if(ser_msg->type == SerialMsg_Type_EXIT_KISS_MODE) {
            shared_mtx.lock();
            kiss_extended = true;
            background_queue.call(oled_mon_fn);
            shared_mtx.unlock();
        } else if(ser_msg->type == SerialMsg_Type_ENTER_KISS_MODE) {
            shared_mtx.lock();
            kiss_extended = false;
            background_queue.call(oled_mon_fn);
            shared_mtx.unlock();
        } else if(ser_msg->type == SerialMsg_Type_GET_CONFIG) {
            auto out_msg_sptr = make_shared<SerialMsg>();
            *out_msg_sptr = ser_msg_zero;
            out_msg_sptr->type = SerialMsg_Type_CONFIG;
            out_msg_sptr->has_sys_cfg = true;
            out_msg_sptr->sys_cfg = radio_cb;
            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, out_msg_sptr);
            send_ack();
            reboot_system();
        } else if(ser_msg->type == SerialMsg_Type_SET_CONFIG) {
            debug_printf(DBG_INFO, "Serial config received\r\n");
            if(!ser_msg->has_sys_cfg) {
                send_error(string("No Configuration Sent!\r\n"));
            } else {
                shared_mtx.lock();
                radio_cb = ser_msg->sys_cfg;
                save_settings_to_flash();
                shared_mtx.unlock();
			    send_ack();
            }
        }
        else if(ser_msg->type == SerialMsg_Type_GET_STATUS) {
            send_status();      
        }
        else if(ser_msg->type == SerialMsg_Type_CLOCK_SET) {
            MBED_ASSERT(ser_msg->has_clock_set);
            shared_mtx.lock();
            set_time(ser_msg->clock_set.time);
            shared_mtx.unlock();
            send_ack();
        }
        else if(ser_msg->type == SerialMsg_Type_STAY_IN_MGT) {
            stay_in_management = true;           
            send_ack();
        }
        else if(ser_msg->type == SerialMsg_Type_DEBUG_MSG) {
            send_ack();
        }
        else if(ser_msg->type == SerialMsg_Type_DATA) {
            if(ser_msg->data_msg.type == DataMsg_Type_TX) {
                auto frame = make_shared<Frame>();
                frame->loadFromPB(ser_msg->data_msg);
                frame->setSender(radio_cb.address);
                frame->setStreamID(get_stream_id());
                auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, frame);
                enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt); 
                send_ack();
            } else if(ser_msg->data_msg.type == DataMsg_Type_KISSTX) {
                debug_printf(DBG_INFO, "Received a KISS frame on port %s of size %d\r\n",
                            port_name.c_str(), ser_msg->data_msg.payload.size); 
                size_t tot_bytes = ser_msg->data_msg.payload.size;
                size_t max_pld_size = Frame::getKISSMaxSize();
                size_t cur_bytes = 0;
                size_t frame_num = 0;
                size_t tot_frames = ceilf((float) ser_msg->data_msg.payload.size/(float) max_pld_size);
                uint8_t stream_id = get_stream_id();
                while(cur_bytes < tot_bytes) {
                    auto frag = make_shared<DataMsg>();
                    *frag = data_msg_zero;
                    frag->kiss_tot_frames = tot_frames;
                    frag->kiss_cur_frame = frame_num++;
                    frag->kiss_stream_id = stream_id;
                    if(tot_bytes-cur_bytes <= max_pld_size) {
                        frag->payload.size = tot_bytes-cur_bytes;
                        memcpy(frag->payload.bytes, ser_msg->data_msg.payload.bytes+cur_bytes, 
                                tot_bytes-cur_bytes);
                        cur_bytes += tot_bytes-cur_bytes;
                    } else {
                        frag->payload.size = max_pld_size;
                        memcpy(frag->payload.bytes, ser_msg->data_msg.payload.bytes+cur_bytes, 
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
            } else {
                printf("Packet type is %d\r\n", ser_msg->data_msg.type);
                MBED_ASSERT(false);
            }      
        }
        else if(ser_msg->type == SerialMsg_Type_REBOOT) {
            debug_printf(DBG_INFO, "Received reboot command\r\n");
            send_ack();
            reboot_system();
        }
        else if(ser_msg->type == SerialMsg_Type_ERASE_LOGS) {
            shared_mtx.lock();
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                DIR *log_dir = opendir("/fs/log");
                MBED_ASSERT(log_dir);
                for(;;) {
                    struct dirent *dir_entry = readdir(log_dir);
                    if(!dir_entry) { break; }
                    stringstream fname;
                    fname << "/log/" << dir_entry->d_name; 
                    if(string(dir_entry->d_name) != "." && string(dir_entry->d_name) != "..") {
                        debug_printf(DBG_INFO, "Deleting %s\r\n", fname.str().c_str());
                        int rem_err = fs.remove(fname.str().c_str());
                        if(rem_err) {
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
        else if(ser_msg->type == SerialMsg_Type_ERASE_BOOT_LOGS) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                shared_mtx.lock();
                fs.remove("boot_log.bin");
                shared_mtx.unlock();
            }
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type == SerialMsg_Type_ERASE_CFG) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                shared_mtx.lock();
                fs.remove("settings.bin");
                shared_mtx.unlock();
            }
            send_ack();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg->type == SerialMsg_Type_READ_LOG) {
            debug_printf(DBG_INFO, "Read log found\r\n");
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                shared_mtx.lock();
				if(!reading_log) {
                    DIR *log_dir = opendir("/fs/log");
                    MBED_ASSERT(log_dir);
                    for(;;) {
                        struct dirent *my_dirent = readdir(log_dir);
                        if(!my_dirent) { break; }
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
                    if(!f) {
                        string err_msg = "Unable to open logfile ";
                        err_msg.append(logfile_names[0]);
                        err_msg.append("\r\n");
                        send_error(err_msg);
                    } else {
                        logfile_names.erase(logfile_names.begin());
                    }
				}
				auto cur_log_msg = make_shared<SerialMsg>();
                *cur_log_msg = ser_msg_zero;
                // Need to have an actually-open filehandle here
                int err_ser = load_SerialMsg(*cur_log_msg, f); 
                debug_printf(DBG_INFO, "Serial Error is %d\r\n", err_ser);
                if(err_ser) { // Go to the next file if it exists
                    if(logfile_names.size() == 0) {
                        auto reply_msg_sptr = make_shared<SerialMsg>(); 
                        *reply_msg_sptr = ser_msg_zero;
                        reply_msg_sptr->type = SerialMsg_Type_REPLY_LOG;
                        reply_msg_sptr->has_log_msg = true;
                        reply_msg_sptr->log_msg.valid = false;
                        enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);   
                        debug_printf(DBG_WARN, "Finished reading logs. Now rebooting...\r\n");
					    reboot_system();	 	
                    } else {
                        f = fopen(logfile_names[0].c_str(), "r");
                        MBED_ASSERT(f);
                        logfile_names.erase(logfile_names.begin());   
                        string cur_line;
                        if(!load_SerialMsg(*cur_log_msg, f)) {
                            auto reply_msg_sptr = make_shared<SerialMsg>();
                            *reply_msg_sptr = ser_msg_zero;
                            reply_msg_sptr->type = SerialMsg_Type_REPLY_LOG;
                            reply_msg_sptr->has_log_msg = true;
                            reply_msg_sptr->log_msg.valid = true;
                            reply_msg_sptr->log_msg.count = line_count++;	
                            if(!cur_log_msg->has_log_msg) {
                                send_error("Logfile entry has no log message\r\n");
                            }
                            reply_msg_sptr->log_msg = cur_log_msg->log_msg;
                            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);   
                        } else {
                            send_error("Logfile read error\r\n");
                        }		
                    }
				} else {
                    auto reply_msg_sptr = make_shared<SerialMsg>();
                    *reply_msg_sptr = ser_msg_zero;
                    reply_msg_sptr->type = SerialMsg_Type_REPLY_LOG;
                    reply_msg_sptr->has_log_msg = true;
                    reply_msg_sptr->log_msg.valid = true;
                    reply_msg_sptr->log_msg.count = line_count++;	
                    if(!cur_log_msg->has_log_msg) {
                        send_error("Logfile entry has no log message\r\n");
                    }
                    reply_msg_sptr->log_msg = cur_log_msg->log_msg;
                    reply_msg_sptr->has_log_msg = true;
                    reply_msg_sptr->log_msg.valid = true;
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr); 
				}
                shared_mtx.unlock();
            }
        }
        else if(ser_msg->type == SerialMsg_Type_READ_BOOT_LOG) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                shared_mtx.lock();
				if(!reading_bootlog) {
					reading_bootlog = true;
					f = fopen("/fs/boot_log.bin", "r");
					MBED_ASSERT(f);
				}
                auto cur_log_msg = make_shared<SerialMsg>();
                *cur_log_msg = ser_msg_zero;
                int err_ser = load_SerialMsg(*cur_log_msg, f);
                if(err_ser) {
                    auto reply_msg = make_shared<SerialMsg>();
                    *reply_msg = ser_msg_zero;
                    reply_msg->type = SerialMsg_Type_REPLY_BOOT_LOG;
                    reply_msg->has_boot_log_msg = true;
                    reply_msg->boot_log_msg.valid = false;
                    auto reply_msg_sptr = make_shared<SerialMsg>(*reply_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr); 
					debug_printf(DBG_WARN, "Now rebooting...\r\n");
					reboot_system();	    
				} else {
                    cur_log_msg->type = SerialMsg_Type_REPLY_BOOT_LOG;
                    cur_log_msg->boot_log_msg.count = line_count++;
                    auto reply_msg_sptr = make_shared<SerialMsg>(*cur_log_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);                               						
				}
                shared_mtx.unlock();
            }
        }		
        else {
            continue;
        }
    }
}
