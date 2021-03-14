/*
QMesh
Copyright (C) 2020 Daniel R. Fay

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

#include "mbed.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <cmath>
#include <list>
#include <map>
#include <algorithm>
#include <atomic>
#include <sstream>
#include "Adafruit_SSD1306.h"
#include "LoRaRadio.h"
#include "mesh_protocol.hpp"
#include "radio_timing.hpp"
#include "radio.hpp"
#include "mem_trace.hpp"
#include "qmesh.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

extern EventQueue background_queue;

static list<uint32_t> past_crc;
static map<uint32_t, time_t> past_timestamp;

volatile bool retransmit_pending = false;
InterruptIn retransmit_disable_in_n(MBED_CONF_APP_RETRANSMIT_DIS_IN, PullUp);
DigitalOut retransmit_disable_out_n(MBED_CONF_APP_RETRANSMIT_DIS_OUT);


/**
 * Checks to see if this Frame has been seen before. Returns a bool
 * of whether it has. 
 * @param rx_frame The frame to check.
 */
static bool checkRedundantPkt(shared_ptr<Frame> rx_frame);
static bool checkRedundantPkt(shared_ptr<Frame> rx_frame) {
    uint32_t crc = rx_frame->calcUniqueCRC();
    bool ret_val = false;
    if(find(past_crc.begin(), past_crc.end(), crc) == past_crc.end()) {
        past_crc.push_back(crc);
        past_timestamp.insert(pair<uint32_t, time_t>(crc, time(NULL)));
        if(past_crc.size() > PKT_CHK_HISTORY) {
            debug_printf(DBG_INFO, "Exceeded history length\r\n");
            past_timestamp.erase(*(past_crc.begin()));
            past_crc.pop_front();
        }
    }
    else { // redundant packet was found. Check the age of the packet.
        map<uint32_t, time_t>::iterator it = past_timestamp.find(crc);
        ret_val = true;
        if(time(NULL) - it->second > 10) { // Ignore entries more than a minute old
            ret_val = false;
            past_crc.erase(find(past_crc.begin(), past_crc.end(), crc));
            past_timestamp.erase(crc);
            past_crc.push_back(crc);
            past_timestamp.insert(pair<uint32_t, time_t>(crc, time(NULL)));
            debug_printf(DBG_INFO, "Exceeded age\r\n");
        }
    }
    return ret_val;
}

/// States that the mesh FSM can be in.
static enum {
    WAIT_FOR_EVENT,
    TX_PACKET,
    RETRANSMIT_PACKET,
    TX_POCSAG
} state = WAIT_FOR_EVENT;

RadioTiming radio_timing;

static atomic<int> total_rx_pkt(0);
static atomic<int> total_rx_corr_pkt(0);
static atomic<int> total_tx_pkt(0);
static atomic<int> last_rx_rssi(0.0);
static atomic<int> last_rx_snr(0.0);

/**
 * Main function handling the mesh protocol.
 */
void mesh_protocol_fsm(void) {
    shared_ptr<FEC> fec;
    FECCfg_Type fec_type = radio_cb.fec_cfg.type;
    debug_printf(DBG_INFO, "%d FEC was chosen\r\n", fec_type);
    if(fec_type == FECCfg_Type_NONE) {
        fec = make_shared<FEC>(Frame::size());
    } else if(fec_type == FECCfg_Type_INTERLEAVE) {
        fec = make_shared<FECInterleave>(Frame::size());
    } else if(fec_type == FECCfg_Type_CONV) {
        fec = make_shared<FECConv>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
                                    radio_cb.fec_cfg.conv_order);
    } else if(fec_type == FECCfg_Type_RSV) {
        fec = make_shared<FECRSVGolay>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
                radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    } else if(fec_type == FECCfg_Type_RSVGOLAY) {
        fec = make_shared<FECRSVGolay>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
                radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    } else {
        MBED_ASSERT(false);
    }
    std::shared_ptr<RadioEvent> radio_event, tx_radio_event;
    std::shared_ptr<Frame> tx_frame_sptr;
    std::shared_ptr<Frame> rx_frame_sptr;
    static vector<uint8_t> tx_frame_buf(256), rx_frame_buf(256);
    static mt19937 rand_gen(radio_cb.address);
    int32_t freq_bound = (lora_bw[radio_cb.radio_cfg.lora_cfg.bw]*FREQ_WOBBLE_PROPORTION);
    static uniform_int_distribution<int32_t> freq_dist(-freq_bound, freq_bound);  
    static mt19937 timing_rand_gen(radio_cb.address);
    uniform_int_distribution<uint8_t> timing_off_dist(0, radio_cb.net_cfg.num_offsets-1);  
    uint8_t next_sym_off = timing_off_dist(timing_rand_gen);
    static mt19937 pwr_diff_rand_gen(radio_cb.address);
    static uniform_int_distribution<int8_t> pwr_diff_dist(0, 6);

    // Set up an initial timer
    auto initial_timer = make_shared<Timer>();
    initial_timer->reset();
    initial_timer->start();
    radio_timing.setTimer(initial_timer);
    radio_timing.waitFullSlots(1); // TODO: change this to be zero once we know zero works
    for(;;) {
        debug_printf(DBG_INFO, "--------------------\r\n");
        //print_memory_info();
        switch(state) {
            case WAIT_FOR_EVENT:
                retransmit_disable_out_n.write(1);
                led2.LEDOff();
                debug_printf(DBG_INFO, "Current state is WAIT_FOR_EVENT\r\n");
                radio.lock();
                radio.rx_hop_frequency(); 
                radio.receive_cad_rx();
                radio.unlock();
                //debug_printf(DBG_INFO, "Started CAD\r\n");
                radio_event = dequeue_mail<shared_ptr<RadioEvent>>(unified_radio_evt_mail);
                radio.stop_cad.store(true);
                while(radio.cad_pending.load() == true);
                radio.stop_cad.store(false);
                debug_printf(DBG_INFO, "Event received is %d\r\n", radio_event->evt_enum);
                if(radio_event->evt_enum == TX_POCSAG_EVT) {
                    debug_printf(DBG_INFO, "Now transmitting a POCSAG page\r\n");
                    radio.lock();
                    radio.standby();
                    //ThisThread::sleep_for(250);
                    //debug_printf(DBG_INFO, "Radio is in standby\r\n");
                    led2.LEDFastBlink();
                    radio.set_channel(radio_cb.pocsag_cfg.frequency);
                    radio.unlock();
                    //ThisThread::sleep_for(250);
                    //debug_printf(DBG_INFO, "Channel is set\r\n");
                    //ThisThread::sleep_for(250);
                    send_pocsag_msg(radio_event->pocsag_msg);
                    //hisThread::sleep_for(250);
                    tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                    //ThisThread::sleep_for(250);
                    radio.lock();
                    reinit_radio();
                    radio.unlock();
                    //ThisThread::sleep_for(250);
                    //ThisThread::sleep_for(250);
                    //debug_printf(DBG_INFO, "Re-Init of radio complete\r\n");
                    led2.LEDOff();
                    //ThisThread::sleep_for(3000);
                    state = WAIT_FOR_EVENT;
                }
                else if(radio_event->evt_enum == TX_FRAME_EVT) {
                    tx_frame_sptr = radio_event->frame;
                    tx_frame_sptr->fec = fec;
                    radio.lock();
                    radio.standby();
                    radio.tx_hop_frequency(freq_dist(rand_gen));
                    radio.unlock();
                    state = TX_PACKET;
                }
                else if(radio_event->evt_enum == RX_DONE_EVT) {
                    //debug_printf(DBG_INFO, "Received a packet\r\n");
                    total_rx_pkt.store(total_rx_pkt.load()+1);
                    last_rx_rssi.store(radio_event->rssi);
                    last_rx_snr.store(radio_event->snr);
                    background_queue.call(oled_mon_fn);
                    // Load up the frame
                    led2.LEDSolid();
                    rx_frame_sptr = make_shared<Frame>(fec);
                    PKT_STATUS_ENUM pkt_status = rx_frame_sptr->deserializeCoded(radio_event->buf);
                    if(pkt_status == PKT_OK) {
                        total_rx_corr_pkt.store(total_rx_corr_pkt.load()+1);
                        background_queue.call(oled_mon_fn);
                        auto rx_frame_orig_sptr = make_shared<Frame>(*rx_frame_sptr);
                        rx_frame_sptr->setSender(radio_cb.address);
                        rx_frame_sptr->incrementTTL();
						rx_frame_sptr->tx_frame = false;
                        if(checkRedundantPkt(rx_frame_sptr)) {
                            radio.standby();
                            debug_printf(DBG_WARN, "Seen packet before, dropping frame\r\n");
                            state = WAIT_FOR_EVENT;
                        }
                        else {
                            radio.standby();
                            radio_timing.setTimer(radio_event->tmr_sptr);
                            enqueue_mail_nonblocking<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_orig_sptr);
                            radio.tx_hop_frequency(freq_dist(rand_gen));
                            retransmit_disable_out_n.write(0);
                            ThisThread::sleep_for(radio_timing.getWaitNoWarn()/2000);
                            if(retransmit_disable_in_n.read()) {
                                state = RETRANSMIT_PACKET;
                            }
                            else {
                                debug_printf(DBG_WARN, "Retransmit blocked by diversity I/O signal\r\n");
                                state = WAIT_FOR_EVENT;
                            }
                        }
                    }
                    else {
                        debug_printf(DBG_INFO, "Rx packet not received correctly\r\n");
                        state = WAIT_FOR_EVENT;
                    }                        
                }
                else if(radio_event->evt_enum == RX_TIMEOUT_EVT) {
                    debug_printf(DBG_INFO, "Rx timed out\r\n");
                    state = WAIT_FOR_EVENT;
                }
                else {
                    state = WAIT_FOR_EVENT;
                }
            break;

            case TX_PACKET:
                total_tx_pkt.store(total_tx_pkt.load()+1);
                background_queue.call(oled_mon_fn);
                debug_printf(DBG_INFO, "Current state is TX_PACKET\r\n");
                radio.lock();
                radio.standby();
                radio.set_tx_power(radio_cb.radio_cfg.tx_power);
                { 
                led2.LEDOff();
                led3.LEDSolid();
                tx_frame_sptr->setOffsets(0, 0, next_sym_off);
                size_t tx_frame_size = tx_frame_sptr->serializeCoded(tx_frame_buf);
                MBED_ASSERT(tx_frame_size < 256);
                debug_printf(DBG_INFO, "Sending %d bytes\r\n", tx_frame_size);
                radio.send_with_delay(tx_frame_buf.data(), tx_frame_size, radio_timing, true);
                radio.unlock();
				tx_frame_sptr->tx_frame = true;
                checkRedundantPkt(tx_frame_sptr); // Don't want to repeat packets we've already sent
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                if(radio_cb.log_packets_en) {
                    enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, tx_frame_sptr);
                }
                radio_timing.setTimer(tx_radio_event->tmr_sptr);
                led3.LEDOff(); 
                // Set the amount of time to wait until the next transmit
                // Start with the two-slot baseline delay
                // Subtract the last frame's symbol delay
                // Add the next frame's symbol delay
                radio_timing.waitFullSlots(2);              
                uint8_t pre_off, nsym_off, sym_off;
                tx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                next_sym_off = timing_off_dist(timing_rand_gen);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", next_sym_off);
                radio_timing.waitSymOffset(next_sym_off, 1.0f, radio_cb.net_cfg.num_offsets);
                state = WAIT_FOR_EVENT;
                }
            break;

            case RETRANSMIT_PACKET:
                debug_printf(DBG_INFO, "Current state is RETRANSMIT_PACKET\r\n");
                {
                radio.lock();
                radio.set_tx_power(radio_cb.radio_cfg.tx_power-pwr_diff_dist(pwr_diff_rand_gen)); 
                led3.LEDSolid();
                size_t rx_frame_size = rx_frame_sptr->serializeCoded(rx_frame_buf);
                MBED_ASSERT(rx_frame_size < 256);
                // Perform the timing offset work:
                //  1. Start by waiting one full slot
                //  2. Subtract the symbol fraction delay from the received packet
                //  3. Set the new symbol fraction delay in the frame
                //  4. Add the symbol fraction delay for the retransmitted packet
				radio_timing.waitFullSlots(1);
                uint8_t pre_off, nsym_off, sym_off;
                rx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                radio_timing.waitSymOffset(sym_off, -1.0f, radio_cb.net_cfg.num_offsets);
                rx_frame_sptr->setOffsets(0, 0, timing_off_dist(timing_rand_gen));
                rx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", sym_off);
                radio_timing.waitSymOffset(sym_off, 1.0f, radio_cb.net_cfg.num_offsets);
                radio.send_with_delay(rx_frame_buf.data(), rx_frame_size, radio_timing);
                radio.unlock();

                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                if(radio_cb.log_packets_en) {
                    enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, rx_frame_sptr);
                }
                led2.LEDOff();
                led3.LEDOff();
                state = WAIT_FOR_EVENT;
                }
            break;

            default:
                MBED_ASSERT(false);
            break;
        }   
    }
}


string beacon_msg;
/**
 * Periodically queues up for transmission a beacon message.
 */
extern time_t boot_timestamp;
void beacon_fn(void) {
    auto beacon_frame_sptr = make_shared<Frame>();
    string beacon_msg(radio_cb.net_cfg.beacon_msg);
    beacon_frame_sptr->setBeaconPayload(beacon_msg);
    beacon_frame_sptr->setSender(radio_cb.address);
    static uint8_t stream_id = 0;
    beacon_frame_sptr->setStreamID(stream_id++);
    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, beacon_frame_sptr);
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt); 
}

/**
 * Periodically queues up for transmission a beacon message.
 */
void beacon_pocsag_fn(void) {
    static bool status = false;
    status = !status;
    if(status) {
        debug_printf(DBG_INFO, "POCSAG beacon set\r\n");
        char msg[128];
        sprintf(msg, "KG5VBY:SF%d,BW%d,F%d\r\n", radio_cb.radio_cfg.lora_cfg.bw, 
                    radio_cb.radio_cfg.lora_cfg.bw, radio_cb.radio_cfg.frequency);
        string pocsag_msg(msg);
        auto radio_evt = make_shared<RadioEvent>(TX_POCSAG_EVT, pocsag_msg);
        enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt); 
        int pocsag_beacon_interval = radio_cb.pocsag_cfg.beacon_interval;
        if(pocsag_beacon_interval == -1) {
            pocsag_beacon_interval = 60000;
        }
    } else { 
        time_t cur_time;
        time(&cur_time);
        time_t uptime = cur_time - boot_timestamp;
        int uptime_rem = uptime;
        int uptime_d = uptime_rem / 86400;
        uptime_rem %= 86400;
        int uptime_h = uptime_rem / 3600;
        uptime_rem %= 3600;
        int uptime_m = uptime_rem / 60;
        uptime_rem %= 60;
        int uptime_s = uptime_rem;
        char msg[128];
        sprintf(msg, "KG5VBY:Uptime:%dd:%dh:%dm:%ds\r\n", uptime_d, uptime_h, uptime_m, uptime_s);
        string pocsag_msg(msg);
        auto radio_evt = make_shared<RadioEvent>(TX_POCSAG_EVT, pocsag_msg);
        enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt);
    }
}


extern Adafruit_SSD1306_I2c *oled;
/**
 * Function called by the OLED display monitor thread. Every second, it 
 *  updates the OLED display with new packet status information.
 */
void oled_mon_fn(void) {
    oled->clearDisplay();
    string kiss_modes_str;
    for(vector<KISSSerial *>::iterator iter = kiss_sers.begin(); iter != kiss_sers.end(); iter++) {
        if((*iter)->kiss_mode) {
            kiss_modes_str.append("KS ");
        } else {
            kiss_modes_str.append("K+ ");
        }
    }
    oled->printf("PACKET STATS  %s\r\n", kiss_modes_str.c_str());      
    oled->printf("#T/R:%5d/%5d\r\n", total_tx_pkt.load(), total_rx_pkt.load());
    oled->printf("Pct Corr Rx: %3d\r\n", (int) (((float) total_rx_corr_pkt.load()/
                    (float) total_rx_pkt.load())*100));
    oled->printf("RSSI/SNR: %4d/%4d\r\n", last_rx_rssi.load(), last_rx_snr.load());
    oled->display();
    oled->display();
}