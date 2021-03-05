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

#include "json_serial.hpp"
#include "mbed.h"
#include "MbedJSONValue.hpp"
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

UARTSerial kiss_ser_fh(MBED_CONF_APP_KISS_UART_TX, MBED_CONF_APP_KISS_UART_RX, 230400);
Mail<std::shared_ptr<SerialMsg>, QUEUE_DEPTH> tx_ser_queue;

void print_memory_info();

static const uint8_t FEND = 0xC0;
static const uint8_t FESC = 0xDB;
static const uint8_t TFEND = 0xDC;
static const uint8_t TFESC = 0xDD;
static const uint8_t SETHW = 0x06;
static const uint8_t DATAPKT = 0x00;
static const uint8_t EXITKISS = 0xFF;
static const size_t MAX_MSG_SIZE = (SerialMsg_size+sizeof(crc_t))*2;
static const SerialMsg ser_msg_zero = SerialMsg_init_zero;
atomic<bool> kiss_mode(false);

static crc_t compute_frame_crc(const uint8_t *buf, const size_t buf_size); 
static bool compare_frame_crc(const uint8_t *buf, const size_t buf_size);


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


write_ser_msg_err_t save_SerialMsg(const SerialMsg &ser_msg, FILE *f, const bool kiss_data_msg) {
    // Serialize the protobuf
    vector<uint8_t> buf(SerialMsg_size);
    pb_ostream_t stream = pb_ostream_from_buffer(buf.data(), buf.size()); 
    if(!pb_encode(&stream, SerialMsg_fields, &ser_msg)) {
        return ENCODE_SER_MSG_ERR;
    }
    vector<uint8_t> comb_buf(stream.bytes_written+sizeof(crc_t));
    memcpy(comb_buf.data(), buf.data(), stream.bytes_written);
    crc_t crc = compute_frame_crc(comb_buf.data(), stream.bytes_written);
    memcpy(comb_buf.data()+stream.bytes_written, &crc, sizeof(crc_t));

    // Make it into a KISS frame and write it out
    if(fputc(FEND, f) == EOF) { return WRITE_SER_MSG_ERR; }
    if(!kiss_data_msg) {
        if(fputc(SETHW, f) == EOF) { return WRITE_SER_MSG_ERR; }
    } else {
        if(fputc(DATAPKT, f) == EOF) { return WRITE_SER_MSG_ERR; }  
    }
    for(uint32_t i = 0; i < stream.bytes_written+sizeof(crc_t); i++) {
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


read_ser_msg_err_t load_SerialMsg(SerialMsg &ser_msg, FILE *f) {
    size_t byte_read_count = 0;
    bool kiss_extended = false;
    // Get past the first delimiter(s)
    for(;;) {
        int cur_byte = fgetc(f);
        printf("Read byte 0x%x\r\n", cur_byte);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        while(cur_byte == FEND) {
            cur_byte = fgetc(f);
            printf("Read byte 0x%x\r\n", cur_byte);
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
        printf("Read byte 0x%x\r\n", cur_byte);
        if(++byte_read_count > MAX_MSG_SIZE) { return READ_MSG_OVERRUN_ERR; }
        if(cur_byte == FEND) {
            break;
        } else if(cur_byte == FESC) {
            printf("Escaped Character\r\n");
            cur_byte = fgetc(f);
            printf("Read byte 0x%x\r\n", cur_byte);
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
            printf("CRC error\r\n");
            return CRC_ERR;
        }
        // Deserialize it
        ser_msg = ser_msg_zero;
        pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size()-sizeof(crc_t));
        if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
            printf("DECODE error\r\n");
            return DECODE_SER_MSG_ERR;
        }
    } else {
        ser_msg = ser_msg_zero;
        ser_msg.type = SerialMsg_Type_DATA;
        ser_msg.has_data_msg = true;
        ser_msg.data_msg.type = DataMsg_Type_KISSRX;
        ser_msg.data_msg.payload.size = buf.size();
        memcpy(ser_msg.data_msg.payload.bytes, buf.data(), buf.size());
    }
    return READ_SUCCESS;
}


void tx_serial_thread_fn(void) {
    FILE *kiss_ser = fdopen(&kiss_ser_fh, "w");
    printf("Entering the tx loop\r\n");
#if 0
    while(1) {
        string test_string("Hello world\r\n");
        //kiss_ser_fh->write(test_string.c_str(), test_string.size());
        fprintf(kiss_ser, "Testing out the KISS port \r\n");
        printf("Did another iteration\r\n");
    }
#endif
    for(;;) {
        auto ser_msg_sptr = dequeue_mail<shared_ptr<SerialMsg>>(tx_ser_queue);
        SerialMsg ser_msg = *ser_msg_sptr;
        // In KISS mode, we only send out data packets in KISS format.
        if(kiss_mode.load()) {
            printf("In KISS mode\r\n");
            // Silently drop anything that isn't a KISSRX frame when we're in KISS mode
            if(ser_msg.type == SerialMsg_Type_DATA && ser_msg.has_data_msg && 
                ser_msg.data_msg.type == DataMsg_Type_KISSRX) {
                save_SerialMsg(ser_msg, kiss_ser, true);
            } else {
                continue;
            }           
        } else {
            save_SerialMsg(ser_msg, kiss_ser, false);  
        }
    }
}

/**
 * Sends the current status.
 */
void send_status(void);
void send_status(void) {
    SerialMsg ser_msg = SerialMsg_init_zero;
    ser_msg.type = SerialMsg_Type_STATUS;
    ser_msg.has_status = true;
    if(current_mode == BOOTING) {
        ser_msg.status.status = StatusMsg_Status_BOOTING;      
    } else if(current_mode == MANAGEMENT) {
        ser_msg.status.status = StatusMsg_Status_MANAGEMENT;     
    } else if(current_mode == RUNNING) {
        ser_msg.status.status = StatusMsg_Status_RUNNING;
    } else {
        MBED_ASSERT(false);
    }
    if(tx_frame_mail.full()) {
        ser_msg.status.tx_full = true;
    } else {
        ser_msg.status.tx_full = false;
    }
    ser_msg.status.time = time(NULL);
    auto ser_msg_sptr = make_shared<SerialMsg>(ser_msg);
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);
}  


void send_ack(void);
void send_ack(void) {
    SerialMsg ser_msg = SerialMsg_init_zero;
    ser_msg.type = SerialMsg_Type_ACK;
    auto ser_msg_sptr = make_shared<SerialMsg>(ser_msg);
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);    
}  


void send_error(const string &err_str);
void send_error(const string &err_str) {
    SerialMsg ser_msg = SerialMsg_init_zero;
    ser_msg.type = SerialMsg_Type_ERR;
    ser_msg.has_error_msg = true;
    size_t str_len = err_str.size() < 256 ? err_str.size() : 256;
    memcpy((char *) ser_msg.error_msg.msg, err_str.c_str(), str_len);
    auto ser_msg_sptr = make_shared<SerialMsg>(ser_msg);
    debug_printf(DBG_WARN, "%s", err_str.c_str());
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);   
}  


void rx_serial_thread_fn(void) {
    printf("Entering the RX serial function\r\n");
	FILE *f = NULL;
	int line_count = 0;
	bool reading_log = false;
	bool reading_bootlog = false;
    static SerialMsg past_log_msg = SerialMsg_init_zero;
    for(;;) {
        SerialMsg ser_msg = SerialMsg_init_zero;
        FILE *kiss_ser = fdopen(&kiss_ser_fh, "r");
        printf("Entering the RX serial\r\n");
        int err = load_SerialMsg(ser_msg, kiss_ser);
        if(err != 0) {
            debug_printf(DBG_WARN, "Error in reading serial port entry. Error %d\r\n", err);
        }
        if(ser_msg.type == SerialMsg_Type_EXIT_KISS_MODE) {
            kiss_mode.store(false);
            background_queue.call(oled_mon_fn);
        } else if(ser_msg.type == SerialMsg_Type_ENTER_KISS_MODE) {
            kiss_mode.store(true);
            background_queue.call(oled_mon_fn);
        } else if(ser_msg.type == SerialMsg_Type_GET_CONFIG) {
            SerialMsg out_msg = SerialMsg_init_zero;
            out_msg.type = SerialMsg_Type_CONFIG;
            out_msg.has_sys_cfg = true;
            out_msg.sys_cfg = radio_cb;
            auto out_msg_sptr = make_shared<SerialMsg>(out_msg);
            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, out_msg_sptr);
            send_ack();
            reboot_system();
        } else if(ser_msg.type == SerialMsg_Type_SET_CONFIG) {
            debug_printf(DBG_INFO, "Serial config received\r\n");
            if(!ser_msg.has_sys_cfg) {
                send_error(string("No Configuration Sent!\r\n"));
            } else {
                radio_cb = ser_msg.sys_cfg;
                save_settings_to_flash();
			    send_ack();
            }
        }
        else if(ser_msg.type == SerialMsg_Type_GET_STATUS) {
            send_status();      
        }
        else if(ser_msg.type == SerialMsg_Type_CLOCK_SET) {
            MBED_ASSERT(ser_msg.has_clock_set);
            set_time(ser_msg.clock_set.time);
            send_ack();
        }
        else if(ser_msg.type == SerialMsg_Type_STAY_IN_MGT) {
            stay_in_management = true;           
            send_ack();
        }
        else if(ser_msg.type == SerialMsg_Type_DEBUG_MSG) {
            send_ack();
        }
        else if(ser_msg.type == SerialMsg_Type_DATA) {
            if(ser_msg.data_msg.type == DataMsg_Type_TX) {
                auto frame = make_shared<Frame>();
                frame->loadFromPB(ser_msg.data_msg);
                enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, frame);
                send_ack();
            } else if(ser_msg.data_msg.type == DataMsg_Type_KISSTX) {
                auto frame = make_shared<Frame>();
                // Right now, we're just doing a single frame for a KISS frame. 
                //  Given that AX.25/APRS frames can be considerably bigger than
                //  a single QMesh frame, we need to implement some sort of 
                //  fragmentation scheme. This scheme should be able to support 
                //  some sort of erasure support that uses e.g. Reed-Solomon to 
                //  recover from a missing fragment.
                frame->createFromKISS(ser_msg.data_msg);
                enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, frame);
            } else {
                MBED_ASSERT(false);
            }        
        }
        else if(ser_msg.type == SerialMsg_Type_REBOOT) {
            debug_printf(DBG_INFO, "Received reboot command\r\n");
            send_ack();
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_ERASE_LOGS) {
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
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_ERASE_BOOT_LOGS) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("boot_log.bin");
            }
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_ERASE_CFG) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("settings.bin");
            }
            send_ack();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_READ_LOG) {
            debug_printf(DBG_INFO, "Read log found\r\n");
            static vector<string> logfile_names;
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
				if(!reading_log) {
                    //logfile_names.clear();
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
                        //debug_printf(DBG_INFO, "STUFF: %s\r\n", file_path.str().c_str());
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
				SerialMsg cur_log_msg = SerialMsg_init_zero;
                // Need to have an actually-open filehandle here
                if(!load_SerialMsg(cur_log_msg, f)) { // Go to the next file if it exists
                    if(logfile_names.size() == 0) {
                        SerialMsg reply_msg = SerialMsg_init_zero;
                        reply_msg.type = SerialMsg_Type_REPLY_LOG;
                        reply_msg.has_log_msg = true;
                        reply_msg.log_msg.valid = false;	
                        auto reply_msg_sptr = make_shared<SerialMsg>(reply_msg); 
                        enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);   
                        debug_printf(DBG_WARN, "Finished reading logs. Now rebooting...\r\n");
					    reboot_system();	 	
                    } else {
                        f = fopen(logfile_names[0].c_str(), "r");
                        MBED_ASSERT(f);
                        logfile_names.erase(logfile_names.begin());   
                        string cur_line;
                        if(!load_SerialMsg(cur_log_msg, f)) {
                            SerialMsg reply_msg = SerialMsg_init_zero;
                            reply_msg.type = SerialMsg_Type_REPLY_LOG;
                            reply_msg.has_log_msg = true;
                            reply_msg.log_msg.valid = true;
                            reply_msg.log_msg.count = line_count++;	
                            if(!cur_log_msg.has_log_msg) {
                                send_error("Logfile entry has no log message\r\n");
                            }
                            reply_msg.log_msg = cur_log_msg.log_msg;
                            auto reply_msg_sptr = make_shared<SerialMsg>(reply_msg);	
                            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);   
                        } else {
                            send_error("Logfile read error\r\n");
                        }		
                    }
				} else {
                    SerialMsg reply_msg = SerialMsg_init_zero;
                    reply_msg.type = SerialMsg_Type_REPLY_LOG;
                    reply_msg.has_log_msg = true;
                    reply_msg.log_msg.valid = true;
                    reply_msg.log_msg.count = line_count++;	
                    if(!cur_log_msg.has_log_msg) {
                        send_error("Logfile entry has no log message\r\n");
                    }
                    reply_msg.log_msg = cur_log_msg.log_msg;
                    auto reply_msg_sptr = make_shared<SerialMsg>(reply_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr); 
				}
            }
        }
        else if(ser_msg.type == SerialMsg_Type_READ_BOOT_LOG) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
				if(!reading_bootlog) {
					reading_bootlog = true;
					f = fopen("/fs/boot_log.bin", "r");
					MBED_ASSERT(f);
				}
                SerialMsg cur_log_msg = SerialMsg_init_zero;
                if(!load_SerialMsg(cur_log_msg, f)) {
                    SerialMsg reply_msg = SerialMsg_init_zero;
                    reply_msg.type = SerialMsg_Type_REPLY_BOOT_LOG;
                    reply_msg.has_boot_log_msg = true;
                    reply_msg.boot_log_msg.valid = false;
                    auto reply_msg_sptr = make_shared<SerialMsg>(reply_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr); 
					debug_printf(DBG_WARN, "Now rebooting...\r\n");
					reboot_system();	    
				} else {
                    cur_log_msg.type = SerialMsg_Type_READ_BOOT_LOG;
                    cur_log_msg.boot_log_msg.count = line_count++;
                    auto reply_msg_sptr = make_shared<SerialMsg>(cur_log_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, reply_msg_sptr);                     
					debug_printf(DBG_WARN, "Now rebooting...\r\n");                    						
				}
            }
        }		
        else {
            continue;
        }
    }
}
