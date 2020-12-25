/*
QMesh
Copyright (C) 2019 Daniel R. Fay

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
#include <stdio.h>
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include "mesh_protocol.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <string.h>

Mutex tx_ser_queue_lock;
list<shared_ptr<SerialMsg>> tx_ser_queue;

void print_memory_info();
void tx_serial_thread_fn(void) {
    for(;;) {
        tx_ser_queue_lock.lock();
        auto ser_msg_sptr = *tx_ser_queue.begin();
        tx_ser_queue.pop_front();
        tx_ser_queue_lock.unlock();
        SerialMsg ser_msg = *ser_msg_sptr;
        pb_byte_t buffer[SerialMsg_size];
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        pb_encode(&stream, SerialMsg_fields, &ser_msg);

        uint16_t delim = 0xBEEF;
        size_t buf_size = stream.bytes_written;
        MbedCRC<POLY_32BIT_ANSI, 32> ct;
        uint32_t crc = 0;
        uint8_t line_buffer[SerialMsg_size+sizeof(delim)+sizeof(buf_size)+sizeof(crc)];

        memcpy((char *) line_buffer, (char *) &delim, sizeof(delim));
        memcpy((char *) line_buffer+sizeof(delim), (char *) &buf_size, sizeof(buf_size));
        memcpy((char *) line_buffer+sizeof(delim)+sizeof(buf_size), buffer, SerialMsg_size);
        ct.compute(line_buffer+sizeof(delim)+sizeof(buf_size), stream.bytes_written, &crc);
        memcpy((char *) line_buffer+sizeof(delim)+sizeof(buf_size)+stream.bytes_written, 
                (char *) crc, sizeof(crc));  

        for(unsigned int i = 0; i < sizeof(line_buffer); i++) {
            fputc(line_buffer[i], stdout);
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

    shared_ptr<SerialMsg> ser_msg_sptr;
    *ser_msg_sptr = ser_msg;
    tx_ser_queue_lock.lock();
    tx_ser_queue.push_back(ser_msg_sptr);
    tx_ser_queue_lock.unlock(); 
}  


void send_ack(void);
void send_ack(void) {
    SerialMsg ser_msg = SerialMsg_init_zero;
    ser_msg.type = SerialMsg_Type_ACK;
    shared_ptr<SerialMsg> ser_msg_sptr;
    *ser_msg_sptr = ser_msg;
    tx_ser_queue_lock.lock();
    tx_ser_queue.push_back(ser_msg_sptr);
    tx_ser_queue_lock.unlock();      
}  


void send_error(const string &err_str);
void send_error(const string &err_str) {
    SerialMsg ser_msg = SerialMsg_init_zero;
    ser_msg.type = SerialMsg_Type_ERR;
    ser_msg.has_error_msg = true;
    size_t str_len = err_str.size() < 256 ? err_str.size() : 256;
    memcpy((char *) ser_msg.error_msg.msg, err_str.c_str(), str_len);
    shared_ptr<SerialMsg> ser_msg_sptr;
    *ser_msg_sptr = ser_msg;
    debug_printf(DBG_ERR, "%s", err_str.c_str());
    tx_ser_queue_lock.lock();
    tx_ser_queue.push_back(ser_msg_sptr);
    tx_ser_queue_lock.unlock();     
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

int get_next_entry(FILE *f, SerialMsg &ser_msg, bool retry = false);
int get_next_entry(FILE *f, SerialMsg &ser_msg, bool retry) {
    size_t entry_size = 0;
    static SerialMsg last_ser_msg = SerialMsg_init_zero;
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
    vector<uint8_t> rx_buf(SerialMsg_size);
    for(;;) {
        // Receive the delimiter. This delimiter helps to ensure that we don't lose sync.
        uint16_t acc = 0x0000;
        for(;;) {
            acc <<= 8;
            acc |= fgetc(stdin);
            if(acc == 0xBEEF) {
                break;
            }
        }
        // Get the packet size
        size_t pkt_size = 0;
        fread((char *) &pkt_size, sizeof(pkt_size), 1, stdin);
        MBED_ASSERT(pkt_size <= SerialMsg_size);
        // Get the packet itself
        fread((char *) rx_buf.data(), 1, pkt_size, stdin);
        // Get the packet checksum
        MbedCRC<POLY_32BIT_ANSI, 32> ct;
        uint32_t crc = 0;
        ct.compute(rx_buf.data(), rx_buf.size(), &crc);
        uint32_t crc_orig = 0;
        fread((char *) &crc_orig, sizeof(crc_orig), 1, stdin); 
        if(crc != crc_orig) {
            debug_printf(DBG_ERR, "Serial packet checksum error\r\n");
            SerialMsg out_msg = SerialMsg_init_zero;
            out_msg.type = SerialMsg_Type_CRC_ERR;
            shared_ptr<SerialMsg> out_msg_sptr;
            *out_msg_sptr = out_msg;
            tx_ser_queue_lock.lock();
            tx_ser_queue.push_back(out_msg_sptr);
            tx_ser_queue_lock.unlock(); 
            continue;
        }
        SerialMsg ser_msg = SerialMsg_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(rx_buf.data(), rx_buf.size());
        pb_decode(&stream, SerialMsg_fields, &ser_msg);
        if(ser_msg.type == SerialMsg_Type_GET_CONFIG) {
            SerialMsg out_msg = SerialMsg_init_zero;
            out_msg.type = SerialMsg_Type_CONFIG;
            out_msg.has_sys_cfg = true;
            out_msg.sys_cfg = radio_cb;
            shared_ptr<SerialMsg> out_msg_sptr;
            *out_msg_sptr = out_msg;
            tx_ser_queue_lock.lock();
            tx_ser_queue.push_back(out_msg_sptr);
            tx_ser_queue_lock.unlock(); 
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
            auto frame = make_shared<Frame>();
            frame->loadFromPB(ser_msg.data_msg);
            enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, frame);
            send_ack();           
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
                        shared_ptr<SerialMsg> reply_msg_sptr;
                        *reply_msg_sptr = reply_msg;
                        tx_ser_queue_lock.lock();
                        tx_ser_queue.push_back(reply_msg_sptr);
                        tx_ser_queue_lock.unlock();  
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
                            shared_ptr<SerialMsg> reply_msg_sptr;
                            *reply_msg_sptr = reply_msg;
                            tx_ser_queue_lock.lock();
                            tx_ser_queue.push_back(reply_msg_sptr);
                            tx_ser_queue_lock.unlock(); 	
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
                    shared_ptr<SerialMsg> reply_msg_sptr;
                    *reply_msg_sptr = reply_msg;
                    tx_ser_queue_lock.lock();
                    tx_ser_queue.push_back(reply_msg_sptr);
                    tx_ser_queue_lock.unlock(); 					
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
                    shared_ptr<SerialMsg> reply_msg_sptr;
                    *reply_msg_sptr = reply_msg;
                    tx_ser_queue_lock.lock();
                    tx_ser_queue.push_back(reply_msg_sptr);
                    tx_ser_queue_lock.unlock();  
					debug_printf(DBG_WARN, "Now rebooting...\r\n");
					reboot_system();	    
				} else {
                    cur_log_msg.type = SerialMsg_Type_READ_BOOT_LOG;
                    cur_log_msg.boot_log_msg.count = line_count++;
                    shared_ptr<SerialMsg> reply_msg_sptr;
                    *reply_msg_sptr = cur_log_msg;
                    tx_ser_queue_lock.lock();
                    tx_ser_queue.push_back(reply_msg_sptr);
                    tx_ser_queue_lock.unlock(); 
					debug_printf(DBG_WARN, "Now rebooting...\r\n");                    						
				}
            }
        }		
        else {
            continue;
        }
    }
}
