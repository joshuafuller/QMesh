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

Mail<std::shared_ptr<SerialMsg>, QUEUE_DEPTH> tx_ser_queue;

void print_memory_info();

static const uint8_t FEND = 0xC0;
static const uint8_t FESC = 0xDB;
static const uint8_t TFEND = 0xDC;
static const uint8_t TFESC = 0xDD;
static const uint8_t SETHW = 0x06;
static const uint8_t DATAPKT = 0x00;
atomic<bool> kiss_mode(true);
void tx_serial_thread_fn(void) {
    uint8_t line_buffer[SerialMsg_size+sizeof(uint16_t)];
    for(;;) {
        // In KISS mode, we only send out data packets in KISS format.
        if(kiss_mode.load()) {
            auto ser_msg_sptr = dequeue_mail<shared_ptr<SerialMsg>>(tx_ser_queue);
            SerialMsg ser_msg = *ser_msg_sptr;
            if(ser_msg.type == SerialMsg_Type_DATA && ser_msg.has_data_msg && 
                ser_msg.data_msg.type == DataMsg_Type_KISSRX) {
                memcpy((char *) line_buffer, (char *) ser_msg.data_msg.payload.bytes, 
                        ser_msg.data_msg.payload.size);
            }            
        } else {
            auto ser_msg_sptr = dequeue_mail<shared_ptr<SerialMsg>>(tx_ser_queue);
            SerialMsg ser_msg = *ser_msg_sptr;
            pb_byte_t buffer[SerialMsg_size];
            pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
            pb_encode(&stream, SerialMsg_fields, &ser_msg);
            MbedCRC<POLY_32BIT_ANSI, 32> ct;
            uint32_t crc = 0;
            memcpy((char *) line_buffer, buffer, SerialMsg_size);
            ct.compute(line_buffer, stream.bytes_written, &crc);
            memcpy((char *) line_buffer+stream.bytes_written, (char *) crc, sizeof(crc));  
        }
        // KISS-ify it
        // Put the first delimiter character in
        fputc(FESC, stdout);
        if(kiss_mode.load()) {
            fputc(SETHW, stdout);
        } else {
            fputc(DATAPKT, stdout);
        }
        for(unsigned int i = 0; i < sizeof(line_buffer); i++) {
            uint8_t cur_byte = line_buffer[i];
            if(cur_byte == FEND) {
                fputc(FESC, stdout);
                fputc(TFEND, stdout);
            } else if(cur_byte == FESC) {
                fputc(FESC, stdout);
                fputc(TFESC, stdout);
            } else {
                fputc(cur_byte, stdout);
            }
        }
        fputc(FESC, stdout);
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
    debug_printf(DBG_ERR, "%s", err_str.c_str());
    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, ser_msg_sptr);   
}  


void get_next_line(FILE *f, string &str);
void get_next_line(FILE *f, string &str) {
	for(;;) {
		int c = fgetc(f);
		if(c == EOF) {
			str.clear();
			return;
		}
		str.push_back(c);
		if(c == '\n') {
			return;
		}
	}
}

int get_next_kiss_entry(FILE *f, SerialMsg &ser_msg, bool retry = false);
int get_next_kiss_entry(FILE *f, SerialMsg &ser_msg, bool retry) {
    static SerialMsg last_ser_msg;
    static size_t last_entry_size = 0;
    vector<uint8_t> ser_data;
    if(retry) {
        ser_msg = last_ser_msg;
        return (int) last_entry_size;
    }
    static bool first_time = true;
    // Get the first frame
    uint8_t cur_byte = 0;
    if(first_time) {
        for(;;) {
            if(fread((char *) &cur_byte, 1, sizeof(cur_byte), f) != sizeof(cur_byte)) {
                return -1;
            }
            if(cur_byte == FEND) {
                break;
            }
        }
    }
    for(;;) {
        if(fread((char *) &cur_byte, 1, sizeof(cur_byte), f) != sizeof(cur_byte)) {
            return -1;
        }
        if(cur_byte == FESC) {
            if(fread((char *) &cur_byte, 1, sizeof(cur_byte), f) != sizeof(cur_byte)) {
                return -1;
            }
            if(cur_byte == TFEND) {
                ser_data.push_back(FEND);
            } else if(cur_byte == TFESC) {
                ser_data.push_back(FESC);
            } else {
                ser_data.push_back(cur_byte);
            }
            MBED_ASSERT(ser_data.size() <= 1024);
        } else if(cur_byte == FEND) {
            // In the KISS protocol, you can have unlimited adjacent FEND bytes.
            //  As a result, if we accumulate a zero-length packet, then we need 
            //  to just continue and try to receive the next frame.
            if(ser_data.size() == 0) {
                continue;
            }
            if(kiss_mode.load()) {
                if(ser_data.size() < 1) {
                    ser_data.clear();
                    continue;
                }
                // We've received a message to exit KISS mode
                if(ser_data.size() == 1 && ser_data[0] == 0xFF) {
                    ser_data.clear();
                    kiss_mode.store(false);
                    continue;
                }
                // We've received a packet on TNC port 0.
                if(ser_data.size() > 1 && ser_data[0] == 0x00) {
                    SerialMsg zero_ser_msg = SerialMsg_init_zero;
                    ser_msg = zero_ser_msg;
                    ser_msg.type = SerialMsg_Type_DATA;
                    ser_msg.has_data_msg = true;
                    ser_msg.data_msg.type = DataMsg_Type_KISSTX;
                    ser_msg.data_msg.payload.size = ser_data.size()-1;
                    memcpy(ser_msg.data_msg.payload.bytes, ser_data.data()+1, ser_data.size()-1);
                    return ser_msg.data_msg.payload.size;
                } else {
                    MBED_ASSERT(false);
                }
            } else {
                // Another issue is that we currently expect our packets to be at 
                //  least three bytes long. We need to handle the (hopefully very rare)
                //  case where we get fewer than three bytes.
                if(ser_data.size() < 3) {
                    ser_data.clear();
                    continue;
                }
                // Right now, we should have the payload as well as a two-byte CRC.
                //  Let's first compute the CRC.
                MbedCRC<POLY_16BIT_CCITT, 16> ct;
                uint32_t crc = 0;
                ct.compute(ser_data.data()+1, ser_data.size()-3, &crc);
                union { 
                    uint32_t u;
                    uint8_t b[4];
                } crc_orig;
                crc_orig.u = 0;
                crc_orig.b[0] = *(ser_data.end()-2);
                crc_orig.b[1] = *(ser_data.end()-1);
                if(crc != crc_orig.u) {
                    debug_printf(DBG_ERR, "Serial packet checksum error\r\n");
                    SerialMsg out_msg = SerialMsg_init_zero;
                    out_msg.type = SerialMsg_Type_CRC_ERR;
                    auto out_msg_sptr = make_shared<SerialMsg>(out_msg);
                    enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, out_msg_sptr);   
                    continue;
                }
                SerialMsg zero_ser_msg = SerialMsg_init_zero;
                ser_msg = zero_ser_msg;
                pb_istream_t stream = pb_istream_from_buffer(ser_data.data()+1, ser_data.size()-3);
                if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
                    return -3;
                }
                // Handle switching to KISS mode at this layer
                if(ser_msg.type == SerialMsg_Type_ENTER_KISS_MODE) {
                    kiss_mode.store(true);
                    ser_data.clear();
                    continue;
                }
                last_entry_size = ser_data.size();
                last_ser_msg = ser_msg;
                ser_data.clear();
                return last_entry_size;
            }
        } else {
            ser_data.push_back(cur_byte);
            MBED_ASSERT(ser_data.size() <= 1024);
        }
    }
}

int get_next_entry(FILE *f, SerialMsg &ser_msg, bool retry = false);
int get_next_entry(FILE *f, SerialMsg &ser_msg, bool retry) {
    size_t entry_size = 0;
    static SerialMsg last_ser_msg;
    static size_t last_entry_size = 0;
    if(retry) {
        ser_msg = last_ser_msg;
        return (int) last_entry_size;
    }
    if(fread((char *) &entry_size, 1, sizeof(entry_size), f) != sizeof(entry_size)) {
        return -1;
    }
    pb_byte_t entry_bytes[SerialMsg_size];
    if(fread((char *) &entry_bytes, 1, entry_size, f) != sizeof(entry_size)) {
        return -2;
    }
    SerialMsg zero_ser_msg = SerialMsg_init_zero;
    ser_msg = zero_ser_msg;
    pb_istream_t stream = pb_istream_from_buffer(entry_bytes, entry_size);
    if(!pb_decode(&stream, SerialMsg_fields, &ser_msg)) {
        return -3;
    }
    last_ser_msg = ser_msg;
    last_entry_size = entry_size;
    return (int) entry_size;
}

void rx_serial_thread_fn(void) {
	FILE *f = NULL;
	int line_count = 0;
	bool reading_log = false;
	bool reading_bootlog = false;
    for(;;) {
        SerialMsg ser_msg = SerialMsg_init_zero;
        if(get_next_kiss_entry(stdin, ser_msg)) {
            debug_printf(DBG_WARN, "Error in reading serial port entry\r\n");
        }
        if(ser_msg.type == SerialMsg_Type_GET_CONFIG) {
            SerialMsg out_msg = SerialMsg_init_zero;
            out_msg.type = SerialMsg_Type_CONFIG;
            out_msg.has_sys_cfg = true;
            out_msg.sys_cfg = radio_cb;
            auto out_msg_sptr = make_shared<SerialMsg>(out_msg);
            enqueue_mail<shared_ptr<SerialMsg>>(tx_ser_queue, out_msg_sptr);   
        } else if(ser_msg.type == SerialMsg_Type_SET_CONFIG) {
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
                    fname << "/fs/log/" << dir_entry->d_name; 
                    debug_printf(DBG_INFO, "Deleting %s\r\n", fname.str().c_str());
                    int rem_err = fs.remove(fname.str().c_str());
                    if(rem_err) {
                        debug_printf(DBG_ERR, "File remove failed with code %d\r\n", rem_err);
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
                fs.remove("boot_log.json");
            }
            send_status();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_ERASE_CFG) {
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
                fs.remove("settings.json");
            }
            send_ack();
            debug_printf(DBG_WARN, "Now rebooting...\r\n");
            reboot_system();
        }
        else if(ser_msg.type == SerialMsg_Type_READ_LOG) {
            static vector<string> logfile_names;
            stay_in_management = true;
            while(current_mode == BOOTING);
            if(current_mode == MANAGEMENT) {
				if(!reading_log) {
                    logfile_names.clear();
                    DIR *log_dir = opendir("/fs/log");
                    MBED_ASSERT(log_dir);
                    for(;;) {
                        struct dirent *my_dirent = readdir(log_dir);
                        if(!my_dirent) { break; }
                        stringstream file_path;
                        file_path << "/fs/log/" << my_dirent->d_name;
                        if(string(my_dirent->d_name) == "." || 
                            string(my_dirent->d_name) == "..") { continue; }
                        debug_printf(DBG_INFO, "STUFF: %s\r\n", file_path.str().c_str());
                        logfile_names.push_back(file_path.str());
                    }
                    reading_log = true;
                    if(!fopen(logfile_names[0].c_str(), "r")) {
                        string err_msg = "Unable to open logfile ";
                        err_msg.append(logfile_names[0]);
                        err_msg.append("\r\n");
                        send_error(err_msg);
                    } else {
                        logfile_names.erase(logfile_names.begin());
                    }
				}
				SerialMsg cur_log_msg = SerialMsg_init_zero;
                if(!get_next_entry(f, cur_log_msg, ser_msg.retry)) { // Go to the next file if it exists
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
                        if(!get_next_entry(f, cur_log_msg, ser_msg.retry)) {
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
					f = fopen("/fs/boot_log.json", "r");
					MBED_ASSERT(f);
				}
                SerialMsg cur_log_msg = SerialMsg_init_zero;
                if(!get_next_entry(f, cur_log_msg, ser_msg.retry)) {
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
