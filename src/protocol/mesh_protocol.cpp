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
#include <math.h>
#include <list>
#include <map>
#include <algorithm>
#include "LoRaRadio.h"
#include "mesh_protocol.hpp"
#include "radio.hpp"

const float lora_bw[] = {125e3f, 250e3f, 500e3f};
static Timeout rx_mesh_time;
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
        ret_val = true;
        past_crc.push_back(crc);
        past_timestamp.insert(pair<uint32_t, time_t>(crc, time(NULL)));
        if(past_crc.size() > PKT_CHK_HISTORY) {
            past_crc.pop_front();
            past_timestamp.erase(crc);
        }
    }
    else { // redundant packet was found. Check the age of the packet.
        map<uint32_t, time_t>::iterator it = past_timestamp.find(crc);
        if(time(NULL) - it->second > 60) { // Ignore entries more than a minute old
            ret_val = true;
            past_crc.erase(find(past_crc.begin(), past_crc.end(), crc));
            past_timestamp.erase(crc);
        }
    }
    return ret_val;
}

void RadioTiming::computeTimes(const uint32_t bw, const uint8_t sf, const uint8_t cr, 
        const uint32_t n_pre_sym, const uint8_t n_pld_bytes) {
    float bw_f = lora_bw[bw];
    float sf_f = sf;
    float cr_f = cr;
    float n_pre_sym_f = n_pre_sym;
    float n_pld_bytes_f = n_pld_bytes;
    n_sym_pre = n_pre_sym;

    // Compute duration of a symbol
    sym_time_s = powf(2, sf_f)/bw_f;
    sym_time_ms = sym_time_s*1e3f;
    sym_time_us = sym_time_s*1e6f;

    // Determine whether we need the low datarate optimize
    float low_dr_opt_f = sym_time_ms >= 16.f ? 1.f : 0.f;

    // Compute the duration of the preamble
    pre_time_s = sym_time_s*n_pre_sym_f;
    pre_time_ms = sym_time_s*n_pre_sym_f*1e3f;
    pre_time_us = sym_time_s*n_pre_sym_f*1e6f;

    // Compute number of payload symbols
    float val = ceilf((8.f*n_pld_bytes_f-4.f*sf_f+28.f-20.f)/
            (4.f*(sf_f-2.f*low_dr_opt_f)));
    float n_sym_pld_f = 8.f + fmaxf(val*(cr_f+4.f), 0.f);
    n_sym_pld = n_sym_pld_f;

    // Compute the duration of the payload
    pld_time_s = n_sym_pld_f*sym_time_s;
    pld_time_ms = n_sym_pld_f*sym_time_s*1e3f;
    pld_time_us = n_sym_pld_f*sym_time_s*1e6f;

    // Compute the duration of the entire LoRa packet
    float n_sym_pkt_f = n_pre_sym_f+n_sym_pld_f;
    n_sym_pkt = n_sym_pkt_f;
    pkt_time_s = n_sym_pkt_f*sym_time_s;
    pkt_time_ms = n_sym_pkt_f*sym_time_s*1e3f;
    pkt_time_us = n_sym_pkt_f*sym_time_s*1e6f;

    // Compute the symbol wait factors
    sym_frac_s = sym_time_s/SYM_FRAC_DELAY_SLOTS;
    sym_frac_ms = sym_frac_s*1e3f;
    sym_frac_us = sym_frac_s*1e6f;
}

void RadioTiming::waitFullSlots(const size_t num_slots) {
    //uint32_t wait_duration_us = pkt_time_us + (4-1)*pre_time_us + sym_time_us;
    uint32_t wait_duration_us = pkt_time_us;
    //debug_printf(DBG_INFO, "xWait duration is %d\r\n", wait_duration_us);
    //debug_printf(DBG_INFO, "pkt_time %d; pre_time %d; sym_time %d\r\n", pkt_time_us, pre_time_us, sym_time_us);
    wait_duration_us *= num_slots;
    int elapsed_us = tmr.read_us();
    wait_us(wait_duration_us-elapsed_us);
}

void RadioTiming::waitRxRemainder(const size_t sym_wait, const size_t pre_wait) {
    uint32_t wait_duration_us = (8-sym_wait)*sym_frac_us + (4-pre_wait)*pre_time_us;
    wait_duration_us += 16*sym_time_us; // 16 symbol padding for right now
    int elapsed_us = tmr.read_us();
    wait_us(wait_duration_us-elapsed_us);
}

void RadioTiming::calcWaitSymbol(void) {
    sym_wait_factor = rand() & 0x07; // 8 different symbol wait options
    sym_wait_us = sym_frac_us*sym_wait_factor;
}

void RadioTiming::calcWaitPreamble(void) {
    pre_wait_factor = rand() & 0x03; // 4 different preamble wait options
    pre_wait_us = pre_time_us*pre_wait_factor;
}

void RadioTiming::waitTx(void) {
    wait_us(sym_wait_us+pre_wait_us);
}

void RadioTiming::startTimer(void) {
    tmr.stop();
    tmr.reset();
    tmr.start();
}

uint32_t RadioFrequency::getWobbledFreq(void) {
    float wobble_factor = (float) rand() / (float) RAND_MAX;
    //debug_printf(DBG_INFO, "Wobble proportion is %f\r\n", wobble_factor);
    float wobble_amount = wobble_factor * lora_bw[radio_cb["BW"].get<int>()] * FREQ_WOBBLE_PROPORTION;
    int wobble_direction = rand() & 0x1;
    wobble_amount = wobble_direction ? -wobble_amount : wobble_amount;
    uint32_t wobbled_freq = (int32_t) radio_cb["Frequency"].get<int>() + (int32_t) wobble_amount;
    //debug_printf(DBG_INFO, "Wobbled freq is %d\r\n", wobbled_freq);
    return wobbled_freq;   
}

RadioFrequency radio_frequency;

static enum {
    WAIT_FOR_RX,
    CHECK_TX_QUEUE,
    TX_PACKET,
    RETRANSMIT_PACKET,
} state = WAIT_FOR_RX;

RadioTiming radio_timing;


volatile bool rx_active = false;
void mesh_protocol_fsm(void) {
    shared_ptr<FEC> fec;
    string fec_algo = radio_cb["FEC Algorithm"].get<string>();
    debug_printf(DBG_INFO, "%s FEC was chosen\r\n", fec_algo.c_str());
    if(fec_algo == "None") {
        fec = make_shared<FEC>();
    }
    else if(fec_algo == "Interleave") {
        fec = make_shared<FECInterleave>();
    }
    else if(fec_algo == "Convolutional") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        fec = make_shared<FECConv>(conv_rate, conv_order);
    }
    else if(fec_algo == "RSV") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        int rs_roots = radio_cb["RS Num Roots"].get<int>();
        fec = make_shared<FECRSV>(conv_rate, conv_order, rs_roots);
    }
    else {
        MBED_ASSERT(false);
    }
    std::shared_ptr<RadioEvent> rx_radio_event, tx_radio_event;
    std::shared_ptr<Frame> tx_frame_sptr;
    std::shared_ptr<Frame> rx_frame_sptr;
    static vector<uint8_t> tx_frame_buf(256), rx_frame_buf(256);
    for(;;) {
#if 0
        while(true) {
            ThisThread::sleep_for(1000);
        }
#endif
		if(rebooting) {
			return;
		}
        int radio_bw = radio_cb["BW"].get<int>();
        int radio_sf = radio_cb["SF"].get<int>();
        int radio_cr = radio_cb["CR"].get<int>();
        int radio_freq = radio_cb["Frequency"].get<int>();
        int radio_pwr = radio_cb["TX Power"].get<int>();  
        int radio_preamble_len = radio_cb["Preamble Length"].get<int>();
        int full_pkt_len = radio_cb["Full Packet Size"].get<int>();
        switch(state) {
            case WAIT_FOR_RX:
#if 0
                radio.set_rx_config(MODEM_LORA, radio_bw,
                            radio_sf, radio_cr,
                            0, radio_preamble_len,
                            radio_preamble_len, RADIO_FIXED_LEN,
                            full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, false);
#endif
				radio.set_channel((int32_t) radio_cb["Frequency"].get<int>());
                if(!rx_active) {
                    debug_printf(DBG_INFO, "Current state is WAIT_FOR_RX\r\n");
                    rx_active = true;
                    radio.receive();
                }
                //debug_printf(DBG_INFO, "Received\r\n");
                //ThisThread::sleep_for(250);
                if(!rx_radio_evt_mail.empty()) {
                    //debug_printf(DBG_INFO, "Dequeuing\r\n");
                    //ThisThread::sleep_for(250);
                    rx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(rx_radio_evt_mail);
                    //debug_printf(DBG_INFO, "Dequeued\r\n");
                    //ThisThread::sleep_for(250);
					radio_timing.startTimer();
                    //debug_printf(DBG_INFO, "Received2\r\n");
                    //ThisThread::sleep_for(250);
                    if(rx_radio_event->evt_enum == RX_DONE_EVT) {
                        debug_printf(DBG_INFO, "Received a packet\r\n");
                        // Load up the frame
                        led2.LEDSolid();
                        rx_frame_sptr = make_shared<Frame>(fec);
                        PKT_STATUS_ENUM pkt_status = rx_frame_sptr->deserializeCoded(rx_radio_event->buf);
                        rx_frame_sptr->incrementTTL();
						rx_frame_sptr->tx_frame = false;
                        if(pkt_status == PKT_OK) {
                            enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, rx_frame_sptr);
                            enqueue_mail_nonblocking<std::shared_ptr<Frame>>(rx_frame_mail, rx_frame_sptr);
                        }
#if 0                        
                        if(pkt_status == PKT_OK && checkRedundantPkt(rx_frame_sptr)) {
                            debug_printf(DBG_WARN, "Rx Queue full, dropping frame\r\n");
                            state = WAIT_FOR_RX;
                        }
#endif
                        if(pkt_status != PKT_OK) {
                            debug_printf(DBG_INFO, "Rx packet not received correctly\r\n");
                            state = CHECK_TX_QUEUE;
                        }
                        else {
                            radio.set_channel(radio_frequency.getWobbledFreq());
                            state = RETRANSMIT_PACKET;
                        }
                    }
                    else if(rx_radio_event->evt_enum == RX_TIMEOUT_EVT) {
                        debug_printf(DBG_INFO, "Rx timed out\r\n");
                        state = CHECK_TX_QUEUE;
                    }
                    else { MBED_ASSERT(false); }
                }
                else {
                    state = CHECK_TX_QUEUE;
                }
            break;

            case CHECK_TX_QUEUE:
                //debug_printf(DBG_INFO, "Current state is CHECK_TX_QUEUE\r\n");
                if(!tx_frame_mail.empty()) {
                    tx_frame_sptr = dequeue_mail<std::shared_ptr<Frame>>(tx_frame_mail);
                    tx_frame_sptr->fec = fec;
                    radio.set_channel(radio_frequency.getWobbledFreq());
                    state = TX_PACKET;
                }
                else {
                    state = WAIT_FOR_RX;
                }
            break;

            case TX_PACKET:
                rx_active = false;
#if 0
                radio.set_tx_config(MODEM_LORA, radio_pwr, 0,
                            radio_bw, radio_sf,
                            radio_cr, radio_preamble_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
#endif
                debug_printf(DBG_INFO, "Current state is TX_PACKET\r\n");
                { 
                led3.LEDSolid();
                size_t tx_frame_size = tx_frame_sptr->serializeCoded(tx_frame_buf);
                MBED_ASSERT(tx_frame_size < 256);
                debug_printf(DBG_INFO, "Sending %d bytes\r\n", tx_frame_size);
                radio.send(tx_frame_buf.data(), tx_frame_size);
				tx_frame_sptr->tx_frame = true;
                enqueue_mail<std::shared_ptr<Frame>>(nv_logger_mail, tx_frame_sptr);
                debug_printf(DBG_INFO, "Waiting on dequeue\r\n");
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
				radio_timing.startTimer();
                debug_printf(DBG_INFO, "dequeued\r\n");
                led3.LEDOff();
                radio_timing.waitFullSlots(2);
                debug_printf(DBG_INFO, "waiting for slots\r\n");
                state = CHECK_TX_QUEUE;
                }
            break;

            case RETRANSMIT_PACKET:
                rx_active = false;
#if 0
                radio.set_tx_config(MODEM_LORA, radio_pwr, 0,
                            radio_bw, radio_sf,
                            radio_cr, radio_preamble_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
#endif
                debug_printf(DBG_INFO, "Current state is RETRANSMIT_PACKET\r\n");
                { 
                led3.LEDSolid();
                size_t rx_frame_size = rx_frame_sptr->serializeCoded(tx_frame_buf);
                //ThisThread::sleep_for(250);
                MBED_ASSERT(rx_frame_size < 256);
				radio_timing.waitFullSlots(1);
                radio.send(rx_frame_buf.data(), rx_frame_size);
                tx_radio_event = dequeue_mail<std::shared_ptr<RadioEvent>>(tx_radio_evt_mail);
                led2.LEDOff();
                led3.LEDOff();
                state = WAIT_FOR_RX;
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
    for(;;) {
		if(rebooting) {
			return;
		}
        enqueue_mail<std::shared_ptr<Frame>>(tx_frame_mail, beacon_frame_sptr);
        ThisThread::sleep_for(radio_cb["Beacon Interval"].get<int>()*1000);
    }
}