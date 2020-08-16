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

#include "mbed.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <cmath>
#include <list>
#include <map>
#include <algorithm>
#include <atomic>
#include "Adafruit_SSD1306.h"
#include "LoRaRadio.h"
#include "mesh_protocol.hpp"
#include "radio_timing.hpp"
#include "radio.hpp"

static LowPowerTimeout rx_mesh_time;
typedef enum {
    TX,
    RX,
} mesh_state_t;
mesh_state_t mesh_state; 


static list<uint32_t> past_crc;
static map<uint32_t, time_t> past_timestamp;
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
        if(time(NULL) - it->second > 60) { // Ignore entries more than a minute old
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

static enum {
    WAIT_FOR_EVENT,
    TX_PACKET,
    RETRANSMIT_PACKET,
} state = WAIT_FOR_EVENT;

RadioTiming radio_timing;

static atomic<int> total_rx_pkt(0);
static atomic<int> total_rx_corr_pkt(0);
static atomic<int> total_tx_pkt(0);
static atomic<int> last_rx_rssi(0.0);
static atomic<int> last_rx_snr(0.0);
void mesh_protocol_fsm(void) {
    shared_ptr<FEC> fec;
    string fec_algo = radio_cb["FEC Algorithm"].get<string>();
    debug_printf(DBG_INFO, "%s FEC was chosen\r\n", fec_algo.c_str());
    if(fec_algo == "None") {
        fec = make_shared<FEC>(Frame::size());
    }
    else if(fec_algo == "Interleave") {
        fec = make_shared<FECInterleave>(Frame::size());
    }
    else if(fec_algo == "Convolutional") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        fec = make_shared<FECConv>(Frame::size(), conv_rate, conv_order);
    }
    else if(fec_algo == "RSV") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        int rs_roots = radio_cb["Reed-Solomon Number Roots"].get<int>();
        fec = make_shared<FECRSV>(Frame::size(), conv_rate, conv_order, rs_roots);
    }
    else {
        MBED_ASSERT(false);
    }
    std::shared_ptr<RadioEvent> radio_event, tx_radio_event;
    std::shared_ptr<Frame> tx_frame_sptr;
    std::shared_ptr<Frame> rx_frame_sptr;
    static vector<uint8_t> tx_frame_buf(256), rx_frame_buf(256);
    int radio_bw = radio_cb["BW"].get<int>();
    int radio_sf = radio_cb["SF"].get<int>();
    int radio_cr = radio_cb["CR"].get<int>();
    int radio_freq = radio_cb["Frequency"].get<int>();
    int radio_pwr = radio_cb["TX Power"].get<int>();  
    int radio_preamble_len = radio_cb["Preamble Length"].get<int>();
    int full_pkt_len = radio_cb["Full Packet Size"].get<int>();
    int my_addr = radio_cb["Address"].get<int>();
    int timing_off_inc = radio_cb["Number Offsets"].get<int>();
    static mt19937 rand_gen(my_addr);
    int32_t freq_bound = (lora_bw[radio_bw]*FREQ_WOBBLE_PROPORTION);
    static uniform_int_distribution<int32_t> freq_dist(radio_freq-freq_bound, radio_freq+freq_bound);  
    static mt19937 timing_rand_gen(my_addr);
    uniform_int_distribution<uint8_t> timing_off_dist(0, timing_off_inc-1);  
    uint8_t next_sym_off = timing_off_dist(timing_rand_gen);
    static mt19937 pwr_diff_rand_gen(my_addr);
    static uniform_int_distribution<int8_t> pwr_diff_dist(0, 6);

    // Set up an initial timer
    auto initial_timer = make_shared<Timer>();
    initial_timer->reset();
    initial_timer->start();
    radio_timing.setTimer(initial_timer);
    radio_timing.waitFullSlots(1); // TODO: change this to be zero once we know zero works
    for(;;) {
        switch(state) {
            case WAIT_FOR_EVENT:
                led2.LEDOff();
                debug_printf(DBG_INFO, "Current state is WAIT_FOR_EVENT\r\n");
                radio.set_channel(radio_freq);
                radio.receive();
                radio_event = dequeue_mail<shared_ptr<RadioEvent>>(unified_radio_evt_mail);
                if(radio_event->evt_enum == TX_FRAME_EVT) {
                    tx_frame_sptr = radio_event->frame;
                    tx_frame_sptr->fec = fec;
                    radio.set_channel(freq_dist(rand_gen));
                    state = TX_PACKET;
                }
                else if(radio_event->evt_enum == RX_DONE_EVT) {
                    debug_printf(DBG_INFO, "Received a packet\r\n");
                    total_rx_pkt.store(total_rx_pkt.load()+1);
                    last_rx_rssi.store(radio_event->rssi);
                    last_rx_snr.store(radio_event->snr);
                    // Load up the frame
                    led2.LEDSolid();
                    rx_frame_sptr = make_shared<Frame>(fec);
                    PKT_STATUS_ENUM pkt_status = rx_frame_sptr->deserializeCoded(radio_event->buf);
                    if(pkt_status == PKT_OK) {
                        total_rx_corr_pkt.store(total_rx_corr_pkt.load()+1);
                        auto rx_frame_orig_sptr = make_shared<Frame>(*rx_frame_sptr);
                        rx_frame_sptr->setSender(my_addr);
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
                            radio.set_channel(freq_dist(rand_gen));
                            state = RETRANSMIT_PACKET;
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
                debug_printf(DBG_INFO, "Current state is TX_PACKET\r\n");
                radio.set_tx_power(radio_pwr);
                { 
                led2.LEDOff();
                led3.LEDSolid();
                tx_frame_sptr->setOffsets(0, 0, next_sym_off);
                size_t tx_frame_size = tx_frame_sptr->serializeCoded(tx_frame_buf);
                MBED_ASSERT(tx_frame_size < 256);
                debug_printf(DBG_INFO, "Sending %d bytes\r\n", tx_frame_size);
                //radio.send(tx_frame_buf.data(), tx_frame_size);
                radio.send_with_delay(tx_frame_buf.data(), tx_frame_size, radio_timing);
				tx_frame_sptr->tx_frame = true;
                checkRedundantPkt(tx_frame_sptr); // Don't want to repeat packets we've already sent
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, tx_frame_sptr);
                radio_timing.setTimer(tx_radio_event->tmr_sptr);
                led3.LEDOff(); 
                // Set the amount of time to wait until the next transmit
                // Start with the two-slot baseline delay
                // Subtract the last frame's symbol delay
                // Add the next frame's symbol delay
                radio_timing.waitFullSlots(2);              
                uint8_t pre_off, nsym_off, sym_off;
                tx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                radio_timing.waitSymOffset(sym_off, -1.0f, timing_off_inc);
                next_sym_off = timing_off_dist(timing_rand_gen);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", next_sym_off);
                radio_timing.waitSymOffset(next_sym_off, 1.0f, timing_off_inc);
                //radio_timing.wait();
                state = WAIT_FOR_EVENT;
                }
            break;

            case RETRANSMIT_PACKET:
                debug_printf(DBG_INFO, "Current state is RETRANSMIT_PACKET\r\n");
                {
                //radio.set_tx_power(radio_pwr-pwr_diff_dist(pwr_diff_rand_gen)); 
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
                radio_timing.waitSymOffset(sym_off, -1.0f, timing_off_inc);
                rx_frame_sptr->setOffsets(0, 0, timing_off_dist(timing_rand_gen));
                rx_frame_sptr->getOffsets(pre_off, nsym_off, sym_off);
                debug_printf(DBG_INFO, "Current timing offset is %d\r\n", sym_off);
                radio_timing.waitSymOffset(sym_off, 1.0f, timing_off_inc);
                radio.send_with_delay(rx_frame_buf.data(), rx_frame_size, radio_timing);
                
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, rx_frame_sptr);
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
void beacon_fn(void) {
    auto beacon_frame_sptr = make_shared<Frame>();
    beacon_frame_sptr->setBeaconPayload(radio_cb["Beacon Message"].get<string>());
    beacon_frame_sptr->setSender(radio_cb["Address"].get<int>());
    int beacon_interval = radio_cb["Beacon Interval"].get<int>();
    uint8_t stream_id = 0;
    for(;;) {
		if(rebooting) {
			return;
		}
        beacon_frame_sptr->setStreamID(stream_id++);
        auto radio_evt = make_shared<RadioEvent>(TX_FRAME_EVT, beacon_frame_sptr);
        MBED_ASSERT(!unified_radio_evt_mail.full());
        if(beacon_interval == -1) {
            ThisThread::sleep_for(60000);
        }
        else {
            enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_evt);
            ThisThread::sleep_for(beacon_interval*1000);
        }
    }
}


extern Adafruit_SSD1306_I2c *oled;
void oled_mon_fn(void) {
    for(;;) {
        oled->clearDisplay();
        oled->printf("PACKET STATISTICS\r\n");
        oled->printf("Pkt Tx/Rx: %4d/%4d\r\n", total_tx_pkt.load(), total_rx_pkt.load());
        oled->printf("Pct Corr Rx: %3d/%3d\r\n", total_rx_corr_pkt.load(), total_rx_pkt.load());
        oled->printf("RSSI/SNR: %4d/%4d\r\n", last_rx_rssi.load(), last_rx_snr.load());
        oled->display();
        oled->display();
        ThisThread::sleep_for(1000);
    }
}