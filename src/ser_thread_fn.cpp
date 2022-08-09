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
#include <string>
#include <vector>
#include <sstream>
#include <atomic>
#include <cstdio>
#include "mesh_protocol.hpp"
#include "serial_msg.hpp"
#include <cstring>
#include "sha256.h"
#include "pseudo_serial.hpp"
#include "serial_data.hpp"
#include "peripherals.hpp"
#include "Adafruit_SSD1306.h"


extern portability::EventQueue *background_queue;
static portability::mutex *shared_mtx;
static const DataMsg data_msg_zero = DataMsg_init_zero;

static constexpr int ERR_MSG_SIZE = 32;
static constexpr int SHA256_SIZE = 32;


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
    auto voice = make_shared<VoiceMsgProcessor>();
    for(;;) {
        ser_msg->clear();
        int err = load_SerMsg(*ser_msg, *pser_rd);
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
        if(ser_msg->type() == SerialMsg_Type_SETHW) {
            debug_printf(DBG_INFO, "Received a SETHW request\r\n");
        }
        if(ser_msg->type() == SerialMsg_Type_SIGRPT) {
            debug_printf(DBG_INFO, "Received a SIGRPT message\r\n");
        }
        if(ser_msg->type() == SerialMsg_Type_TURN_OLED_OFF) {
            debug_printf(DBG_INFO, "Received a request to turn OFF the OLED display\r\n");
            auto *disp_file = fopen("/fs/display.off", "we");
            if(disp_file == nullptr) {
                PORTABLE_ASSERT(false);
            } else {
                fclose(disp_file);
            }
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
