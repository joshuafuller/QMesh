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
#include "rtos.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <string>
#include <utility>
#include "radio.hpp"
#include "radio_timing.hpp"
#include "correct.h"
#include "mem_trace.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"

#define SX126X   0xDD

#define CDEBYTES_E22

#include "SX126X_LoRaRadio.h"

SX126X_LoRaRadio radio(MBED_CONF_APP_LORA_SPI_MOSI, // PinName mosi
                       MBED_CONF_APP_LORA_SPI_MISO, // PinName miso
                       MBED_CONF_APP_LORA_SPI_SCLK, // PinName sclk
                       MBED_CONF_APP_LORA_CS,  // PinName nss
                       MBED_CONF_APP_LORA_RXCTL,  // PinName rxen
                       MBED_CONF_APP_LORA_TXCTL,  // PinName txen
                       MBED_CONF_APP_LORA_RESET,  // PinName reset
                       MBED_CONF_APP_LORA_DIO1,  // PinName dio1
                       MBED_CONF_APP_LORA_DIO2,  // PinName dio2
                       MBED_CONF_APP_LORA_RESET,  // PinName nrst
                       MBED_CONF_APP_LORA_BUSY,   // PinName busy,
                       NC);  
SysCfgMsg radio_cb;

// The callbacks used by the LoRa radio driver
static radio_events_t radio_events;

// Event queue for communicating events from the radio
Mail<shared_ptr<RadioEvent>, QUEUE_DEPTH> unified_radio_evt_mail, tx_radio_evt_mail;

// Prototypes for the callbacks
static void tx_done_cb(shared_ptr<CalTimer> tmr_sptr);
static void rx_done_cb(uint8_t const *payload, 
                shared_ptr<list<pair<uint32_t, uint8_t> > > my_rssi_list_sptr,
                shared_ptr<CalTimer> tmr_sptr, uint16_t size, int16_t rssi, int8_t snr);
static void tx_timeout_cb();
static void rx_timeout_cb();
static void rx_error_cb();
static void rx_preamble_det_cb();
static void fhss_change_channel_cb(uint8_t current_channel);

shared_ptr<FEC> frame_fec;  

// Included from lora_radio_helper.h is a radio object for our radio.
// Let's set it up.
static constexpr int QUARTER_SECOND = 250;
static constexpr int ONE_SECOND = 1000;
void init_radio() {
    // Initialize Radio driver
    debug_printf(DBG_INFO, "Now initializing the radio\r\n");
    radio_events.tx_done_tmr = tx_done_cb;
    radio_events.rx_done_tmr = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.rx_preamble_det = rx_preamble_det_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    ThisThread::sleep_for(QUARTER_SECOND);
    radio.init_radio(&radio_events);
    debug_printf(DBG_INFO, "Radio BW is %d\r\n", radio_cb.radio_cfg.lora_cfg.bw);
    debug_printf(DBG_INFO, "Radio SF is %d\r\n", radio_cb.radio_cfg.lora_cfg.sf);
    debug_printf(DBG_INFO, "Radio CR is %d\r\n", radio_cb.radio_cfg.lora_cfg.cr);
    debug_printf(DBG_INFO, "Radio Frequency is %d\r\n", radio_cb.radio_cfg.frequency);
    debug_printf(DBG_INFO, "Radio Power is %d\r\n", radio_cb.radio_cfg.tx_power);    
    debug_printf(DBG_INFO, "Initial Radio Preamble Length is %d\r\n", 
                radio_cb.radio_cfg.lora_cfg.preamble_length); 
    radio_cb.radio_cfg.lora_cfg.fhss_pre_len = radio_cb.radio_cfg.lora_cfg.preamble_length + 
            7*(radio_cb.radio_cfg.frequencies_count+2); 
    debug_printf(DBG_INFO, "FHSS-Adjusted Radio Preamble Length is %d\r\n", 
            radio_cb.radio_cfg.lora_cfg.fhss_pre_len); 
    vector<uint32_t> freqs;
    for(int i = 0; i < radio_cb.radio_cfg.frequencies_count; i++) {
        freqs.push_back(radio_cb.radio_cfg.frequencies[i]); 
        debug_printf(DBG_INFO, "Frequencies PULLED %d\r\n", radio_cb.radio_cfg.frequencies[i]);
    }
    radio.configure_freq_hop(radio_cb.address, freqs);
    switch(radio_cb.fec_cfg.type) {
        case FECCfg_Type_NONE:
            debug_printf(DBG_INFO, "FEC algorithm is NONE\r\n");
            break;
        case FECCfg_Type_INTERLEAVE:
            debug_printf(DBG_INFO, "FEC algorithm is INTERLEAVE\r\n");
            break;    
        case FECCfg_Type_CONV:
            debug_printf(DBG_INFO, "FEC algorithm is CONV\r\n");
            break;    
        case FECCfg_Type_RSV:
            debug_printf(DBG_INFO, "FEC algorithm is RSV\r\n");
            break;    
        case FECCfg_Type_RSVGOLAY:
            debug_printf(DBG_INFO, "FEC algorithm is RSVGOLAY\r\n");
            break; 
        default:
            MBED_ASSERT(false);   
    }
    if(radio_cb.fec_cfg.type == FECCfg_Type_NONE) {
        frame_fec = make_shared<FEC>(Frame::size());
    }
    else if(radio_cb.fec_cfg.type == FECCfg_Type_INTERLEAVE) {
        frame_fec = make_shared<FECInterleave>(Frame::size());
    }
    else if(radio_cb.fec_cfg.type == FECCfg_Type_CONV) {
        frame_fec = make_shared<FECConv>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
            radio_cb.fec_cfg.conv_order);
    }
    else if(radio_cb.fec_cfg.type == FECCfg_Type_RSV) {
        frame_fec = make_shared<FECRSV>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
            radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    }
    else if(radio_cb.fec_cfg.type == FECCfg_Type_RSVGOLAY) {
        frame_fec = make_shared<FECRSVGolay>(Frame::size(), radio_cb.fec_cfg.conv_rate, 
            radio_cb.fec_cfg.conv_order, radio_cb.fec_cfg.rs_num_roots);
    }
    else {
        MBED_ASSERT(false);
    }
    Frame tmp_frame(frame_fec);
    radio_cb.net_cfg.full_pkt_len = tmp_frame.codedSize();
    debug_printf(DBG_INFO, "Setting RX size to %d\r\n", radio_cb.net_cfg.full_pkt_len);
    uint32_t pre_len = 0;
    if(radio_cb.radio_cfg.frequencies_count > 1) {
        pre_len = radio_cb.radio_cfg.lora_cfg.fhss_pre_len;
    } else {
        pre_len = radio_cb.radio_cfg.lora_cfg.preamble_length;
    }
    radio.set_rx_config(MODEM_LORA, 
                            radio_cb.radio_cfg.lora_cfg.bw,
                            radio_cb.radio_cfg.lora_cfg.sf, 
                            radio_cb.radio_cfg.lora_cfg.cr,
                            0, 
                            pre_len,
                            pre_len, 
                            RADIO_FIXED_LEN,
                            radio_cb.net_cfg.full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(MODEM_LORA, radio_cb.radio_cfg.tx_power, 0,
                            radio_cb.radio_cfg.lora_cfg.bw, 
                            radio_cb.radio_cfg.lora_cfg.sf,
                            radio_cb.radio_cfg.lora_cfg.bw, 
                            pre_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
    radio.set_channel(*freqs.begin());
    radio_timing.computeTimes(radio_cb.radio_cfg.lora_cfg.bw, 
                        radio_cb.radio_cfg.lora_cfg.sf, 
                        radio_cb.radio_cfg.lora_cfg.cr, 
                        pre_len, 
                        radio_cb.net_cfg.full_pkt_len);
    constexpr float RX_TIMEOUT_INCREMENT = 15.625F;
    radio.cad_rx_timeout = radio_timing.get_pkt_time_us() / RX_TIMEOUT_INCREMENT;
    if(radio_cb.test_cfg.cw_test_mode) { 
        debug_printf(DBG_WARN, "Starting continuous wave output...\r\n");
        radio.set_tx_continuous_wave(0, 0, 0);
        while(true) {
            ThisThread::sleep_for(ONE_SECOND);
        }
    }
    else if(radio_cb.test_cfg.preamble_test_mode) {
        debug_printf(DBG_WARN, "Starting continuous preamble output...\r\n");
        radio.set_tx_continuous_preamble();
        while(true) {
            ThisThread::sleep_for(ONE_SECOND);
        }
    }
    else if(radio_cb.test_cfg.test_fec) {
        debug_printf(DBG_WARN, "Starting the FEC testing...\r\n");
        testFEC();
    }
}


void reinit_radio() {
    uint32_t pre_len = 0;
    if(radio_cb.radio_cfg.frequencies_count > 1) {
        pre_len = radio_cb.radio_cfg.lora_cfg.fhss_pre_len;
    } else {
        pre_len = radio_cb.radio_cfg.lora_cfg.preamble_length;
    }
    // Initialize Radio driver
    radio.set_rx_config(MODEM_LORA, 
                            radio_cb.radio_cfg.lora_cfg.bw,
                            radio_cb.radio_cfg.lora_cfg.sf, 
                            radio_cb.radio_cfg.lora_cfg.cr,
                            0, 
                            pre_len,
                            pre_len, 
                            RADIO_FIXED_LEN,
                            radio_cb.net_cfg.full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(MODEM_LORA, radio_cb.radio_cfg.tx_power, 0,
                            radio_cb.radio_cfg.lora_cfg.bw, 
                            radio_cb.radio_cfg.lora_cfg.sf,
                            radio_cb.radio_cfg.lora_cfg.bw, 
                            pre_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum) {
    evt_enum = my_evt_enum;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr_sptr) {
    tmr_sptr = std::move(my_tmr_sptr);
    evt_enum = my_evt_enum;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr_sptr, const uint8_t *my_buf,
        shared_ptr<list<pair<uint32_t, uint8_t> > > my_rssi_list_sptr,
        const size_t my_size, const int16_t my_rssi, const int8_t my_snr) {
    rssi_list_sptr = std::move(my_rssi_list_sptr);
    tmr_sptr = std::move(my_tmr_sptr);
    evt_enum = my_evt_enum;
    MBED_ASSERT(my_size <= 256);
    buf = std::make_shared<vector<uint8_t>>();
    buf->resize(my_size);
    memcpy(buf->data(), my_buf, my_size);
    rssi = my_rssi;
    snr = my_snr;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, const shared_ptr<Frame> &my_frame) {
    evt_enum = my_evt_enum;
    frame = my_frame;
}


static void tx_done_cb(shared_ptr<CalTimer> tmr_sptr)
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(TX_DONE_EVT, tmr_sptr);
    MBED_ASSERT(!tx_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> > (tx_radio_evt_mail, radio_event);
}


static void rx_done_cb(uint8_t const *payload, shared_ptr<list<pair<uint32_t, uint8_t> > > rssi_list_sptr, 
                    shared_ptr<CalTimer> tmr_sptr, uint16_t size, int16_t rssi, int8_t snr)
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(RX_DONE_EVT, tmr_sptr, payload, rssi_list_sptr, 
                                            static_cast<size_t>(size), rssi, snr);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event);
    //debug_printf(DBG_INFO, "RX Done interrupt generated %d rssi %d snr %d\r\n", size, rssi, snr); 
}


static void rx_preamble_det_cb() { }


static void tx_timeout_cb()
{
    radio.standby();
    init_radio();
    auto radio_event = make_shared<RadioEvent>(TX_TIMEOUT_EVT);
    MBED_ASSERT(!tx_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(tx_radio_evt_mail, radio_event);
}
 
static void rx_timeout_cb()
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(RX_TIMEOUT_EVT);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event);  
}
 
static void rx_error_cb()
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(RX_ERROR_EVT);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event);
}

static void fhss_change_channel_cb(uint8_t current_channel) { }
