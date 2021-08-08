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
#include "anti_interference.hpp"
#include "peripherals.hpp"

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
static auto checkRedundantPkt(const shared_ptr<Frame> &rx_frame) -> bool;
static auto checkRedundantPkt(const shared_ptr<Frame> &rx_frame) -> bool {
    uint32_t crc = rx_frame->calcUniqueCRC();
    bool ret_val = false;
    if(find(past_crc.begin(), past_crc.end(), crc) == past_crc.end()) {
        past_crc.push_back(crc);
        past_timestamp.emplace(crc, time(nullptr));
        if(past_crc.size() > PKT_CHK_HISTORY) {
            debug_printf(DBG_INFO, "Exceeded history length\r\n");
            past_timestamp.erase(*(past_crc.begin()));
            past_crc.pop_front();
        }
    }
    else { // redundant packet was found. Check the age of the packet.
        auto it = past_timestamp.find(crc);
        ret_val = true;
        constexpr int IGNORE_THRESHOLD_SEC = 10;
        if(time(nullptr) - it->second > IGNORE_THRESHOLD_SEC) { 
            ret_val = false;
            past_crc.erase(find(past_crc.begin(), past_crc.end(), crc));
            past_timestamp.erase(crc);
            past_crc.push_back(crc);
            past_timestamp.emplace(crc, time(nullptr));
            debug_printf(DBG_INFO, "Exceeded age\r\n");
        }
    }
    return ret_val;
}

/// States that the mesh FSM can be in.
static enum {
    WAIT_FOR_EVENT,
    TX_PACKET,
    RETRANSMIT_PACKET
} state = WAIT_FOR_EVENT;

RadioTiming radio_timing;

atomic<int> total_rx_pkt(0);
atomic<int> total_rx_corr_pkt(0);
atomic<int> total_tx_pkt(0);
atomic<int> last_rx_rssi(0.0);
atomic<int> last_rx_snr(0.0);

/**
 * Main function handling the mesh protocol.
 */
static constexpr int FRAME_BUF_SIZE = 256;
void mesh_protocol_fsm() {
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
        fec = make_shared<FECRSV>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
                radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    } else if(fec_type == FECCfg_Type_RSVGOLAY) {
        fec = make_shared<FECRSVGolay>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
                radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    } else {
        MBED_ASSERT(false);
    }
    std::shared_ptr<RadioEvent> radio_event;
    std::shared_ptr<RadioEvent> tx_radio_event;
    std::shared_ptr<Frame> tx_frame_sptr;
    std::shared_ptr<Frame> rx_frame_sptr;
    static vector<uint8_t> tx_frame_buf(FRAME_BUF_SIZE);
    static vector<uint8_t> rx_frame_buf(FRAME_BUF_SIZE);
    static constexpr int MAX_PWR_DIFF = 6;
    int32_t freq_bound = (lora_bw.at(radio_cb.radio_cfg.lora_cfg.bw)*FREQ_WOBBLE_PROPORTION);
    AntiInterference *anti_inter = nullptr;
    if(radio_cb.net_cfg.walsh_codes) {
        debug_printf(DBG_INFO, "Anti-Interference Walsh codes chosen\r\n");
        anti_inter = new AntiInterferenceWalsh(pair<int32_t, int32_t>(-freq_bound, freq_bound),
                        radio_cb.net_cfg.num_offsets, radio_cb.address, 
                        MAX_PWR_DIFF, radio_cb.radio_cfg.frequencies_count);
    } else {
        debug_printf(DBG_INFO, "Anti-Interference random codes chosen\r\n");
        anti_inter = new AntiInterferenceRand(pair<int32_t, int32_t>(-freq_bound, freq_bound),
                        radio_cb.net_cfg.num_offsets, radio_cb.address, 
                        MAX_PWR_DIFF, radio_cb.radio_cfg.frequencies_count);        
    }
    radio.configure_freq_hop(anti_inter);
    auto next_sym_off = 0;

    // Set up an initial timer
    auto initial_timer = make_shared<Timer>();
    initial_timer->reset();
    initial_timer->start();
    radio_timing.setTimer(initial_timer);
    radio_timing.waitFullSlots(1); // Change this to be zero once we know zero works
    for(;;) {
        debug_printf(DBG_INFO, "--------------------\r\n");
        //print_memory_info();
        switch(state) {
            case WAIT_FOR_EVENT:
                retransmit_disable_out_n.write(1);
                led2.LEDOff();
                debug_printf(DBG_INFO, "Current state is WAIT_FOR_EVENT\r\n");
                radio.lock();
                radio.receive_sel();
                radio.unlock();
                radio_event = dequeue_mail<shared_ptr<RadioEvent>>(unified_radio_evt_mail);
                radio.stop_cad.store(true);
                while(radio.cad_pending.load() == true) { };
                radio.stop_cad.store(false);
                debug_printf(DBG_INFO, "Event received is %d\r\n", radio_event->evt_enum);
                if(radio_event->evt_enum == TX_FRAME_EVT) {
                    tx_frame_sptr = radio_event->frame;
                    tx_frame_sptr->fec = fec;
                    radio.lock();
                    radio.standby();
                    anti_inter->setTTL(0);
                    radio.tx_hop_frequency();
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
                    auto rx_frame_noninv_sptr = make_shared<Frame>(fec);
                    auto rx_frame_inv_sptr = make_shared<Frame>(fec);
                    auto pkt_status_noninv = rx_frame_noninv_sptr->deserializeCoded(radio_event->buf);
                    auto pkt_status_inv = rx_frame_inv_sptr->deserializeCodedInv(radio_event->buf);
                    auto pkt_status = pkt_status_noninv;
                    rx_frame_sptr = rx_frame_noninv_sptr;
                    if(pkt_status != PKT_OK) {
                        pkt_status = pkt_status_inv;
                        rx_frame_sptr = rx_frame_inv_sptr;
                    }
                    if(pkt_status == PKT_OK) {
                        total_rx_corr_pkt.store(total_rx_corr_pkt.load()+1);
                        background_queue.call(oled_mon_fn);
                        auto rx_frame_orig_sptr = make_shared<Frame>(*rx_frame_sptr);
                        rx_frame_sptr->setSender(radio_cb.address);
                        rx_frame_sptr->incrementTTL();
                        anti_inter->setTTL(rx_frame_sptr->getTTL());
                        radio.tx_hop_frequency();
						rx_frame_sptr->tx_frame = false;
                        if(checkRedundantPkt(rx_frame_sptr)) {
                            radio.standby();
                            debug_printf(DBG_WARN, "Seen packet before, dropping frame\r\n");
                            rx_frame_sptr->setRedundant();
                            enqueue_mail_nonblocking<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_orig_sptr);
                            state = WAIT_FOR_EVENT;
                        }
                        else {
                            radio.standby();
                            radio_timing.setTimer(radio_event->tmr_sptr);
                            enqueue_mail_nonblocking<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_orig_sptr);
                            retransmit_disable_out_n.write(0);
                            constexpr int TWO_SECONDS = 2000;
                            ThisThread::sleep_for(radio_timing.getWaitNoWarn()/TWO_SECONDS);
                            if(retransmit_disable_in_n.read() != 0) {
                                state = RETRANSMIT_PACKET;
                            }
                            else {
                                debug_printf(DBG_WARN, "Retransmit blocked by diversity I/O signal\r\n");
                                rx_frame_sptr->setRedundant();
                                enqueue_mail_nonblocking<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_orig_sptr);
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
                next_sym_off = anti_inter->timingOffset(); // Also need to "use" this value
                radio.set_tx_power(radio_cb.radio_cfg.tx_power);
                { 
                led2.LEDOff();
                //led3.LEDSolid();
                tx_frame_sptr->setOffsets(0, 0, next_sym_off);
                size_t tx_frame_size = 0;
                if(anti_inter->invertBits() && radio_cb.net_cfg.invert_bits) {
                    tx_frame_size = tx_frame_sptr->serializeCodedInv(tx_frame_buf);
                } else {
                    tx_frame_size = tx_frame_sptr->serializeCoded(tx_frame_buf);
                }
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
                uint8_t pre_off = 0;
                uint8_t nsym_off = 0;
                uint8_t sym_off = 0;
                tx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", next_sym_off);
                radio_timing.waitSymOffset(next_sym_off, 1.0F, radio_cb.net_cfg.num_offsets);
                state = WAIT_FOR_EVENT;
                }
            break;

            case RETRANSMIT_PACKET:
                debug_printf(DBG_INFO, "Current state is RETRANSMIT_PACKET\r\n");
                {
                radio.lock();
                radio.set_tx_power(radio_cb.radio_cfg.tx_power-anti_inter->pwrDiff()); 
                led3.LEDSolid();
                size_t rx_frame_size = 0;
                if(anti_inter->invertBits() && radio_cb.net_cfg.invert_bits) {
                    rx_frame_size = rx_frame_sptr->serializeCodedInv(rx_frame_buf);
                } else {
                    rx_frame_size = rx_frame_sptr->serializeCoded(rx_frame_buf);
                }
                MBED_ASSERT(rx_frame_size < 256);
                // Perform the timing offset work:
                //  1. Start by waiting one full slot
                //  2. Subtract the symbol fraction delay from the received packet
                //  3. Set the new symbol fraction delay in the frame
                //  4. Add the symbol fraction delay for the retransmitted packet
				radio_timing.waitFullSlots(1);
                uint8_t pre_off = 0;
                uint8_t nsym_off = 0;
                uint8_t sym_off = 0;
                rx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                radio_timing.waitSymOffset(sym_off, -1.0F, radio_cb.net_cfg.num_offsets);
                rx_frame_sptr->setOffsets(0, 0, anti_inter->timingOffset());
                rx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", sym_off);
                radio_timing.waitSymOffset(sym_off, 1.0F, radio_cb.net_cfg.num_offsets);
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
void beacon_fn() {
    auto beacon_frame_sptr = make_shared<Frame>();
    string beacon_msg(radio_cb.net_cfg.beacon_msg);
    beacon_frame_sptr->setBeaconPayload(beacon_msg);
    beacon_frame_sptr->setSender(radio_cb.address);
    static uint8_t stream_id = 0;
    beacon_frame_sptr->setStreamID(stream_id++);
    auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, beacon_frame_sptr);
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt); 
}


extern shared_ptr<Adafruit_SSD1306_I2c> oled;
/**
 * Function called by the OLED display monitor thread. Every second, it 
 *  updates the OLED display with new packet status information.
 */
void oled_mon_fn() {
    oled->clearDisplay();
    string kiss_modes_str;
    kiss_sers_mtx.lock();
    for(auto & kiss_ser : kiss_sers) {
        if(!kiss_ser->isKISSExtended()) {
            kiss_modes_str.append("KS ");
        } else {
            kiss_modes_str.append("K+ ");
        }
    }
    kiss_sers_mtx.unlock();
    oled->printf("PACKET STATS  %s\r\n", kiss_modes_str.c_str());      
    oled->printf("#T/R:%5d/%5d\r\n", total_tx_pkt.load(), total_rx_pkt.load());
    constexpr int HUNDRED_PCT = 100;
    oled->printf("Pct Corr Rx: %3d\r\n", static_cast<int>((static_cast<float>(total_rx_corr_pkt.load())/
                    static_cast<float>(total_rx_pkt.load()))*HUNDRED_PCT));
    oled->printf("RSSI/SNR: %4d/%4d\r\n", last_rx_rssi.load(), last_rx_snr.load());
    oled->display();
    oled->display();
}