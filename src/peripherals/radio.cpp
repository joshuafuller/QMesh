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
#include "rtos.h"
#include "params.hpp"
#include "serial_data.hpp"
#include <string>
#include "radio.hpp"
#include "radio_timing.hpp"
#include "correct.h"
#include "pocsag.h"

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
MbedJSONValue radio_cb;

// The callbacks used by the LoRa radio driver
static radio_events_t radio_events;

// Event queue for communicating events from the radio
Mail<shared_ptr<RadioEvent>, QUEUE_DEPTH> unified_radio_evt_mail, tx_radio_evt_mail;

// Prototypes for the callbacks
static void tx_done_cb(shared_ptr<CalTimer> tmr_sptr);
static void rx_done_cb(uint8_t const *payload, 
                shared_ptr<list<pair<uint32_t, uint8_t> > > my_rssi_list_sptr,
                shared_ptr<CalTimer> tmr_sptr, uint16_t size, int16_t rssi, int8_t snr);
static void tx_timeout_cb(void);
static void rx_timeout_cb(void);
static void rx_error_cb(void);
static void rx_preamble_det_cb(void);
static void fhss_change_channel_cb(uint8_t current_channel);

shared_ptr<FEC> frame_fec;  
static DeepSleepLock *deep_sleep_lock;

void send_pocsag_msg(string &msg) {
    char *data = new char(msg.length()+1);
    memcpy(data, msg.c_str(), msg.length());
    data[msg.length()] = '\0';
    debug_printf(DBG_INFO, "orig size is %d\r\n", msg.length());
    Pocsag my_pocsag;
    if(!my_pocsag.CreatePocsag(10, 1, data, 1, 1)) {
        debug_printf(DBG_INFO, "Error is %d\r\n", my_pocsag.GetError());
        while(1);
        MBED_ASSERT(false);
    }
#if 0
    radio.set_tx_config_pocsag(20);
#else
    reinit_radio_pocsag();
#endif
    debug_printf(DBG_INFO, "size is %d\r\n", my_pocsag.GetSize());
    radio.send((uint8_t *) my_pocsag.GetMsgPointer(), my_pocsag.GetSize());
    delete data;
}

// Included from lora_radio_helper.h is a radio object for our radio.
// Let's set it up.
void init_radio(void) {
    // Initialize Radio driver
    debug_printf(DBG_INFO, "Now initializing the radio\r\n");
    radio_events.tx_done_tmr = tx_done_cb;
    radio_events.rx_done_tmr = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.rx_preamble_det = rx_preamble_det_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    ThisThread::sleep_for(250);
    radio.init_radio(&radio_events);
    int radio_bw = radio_cb["BW"].get<int>();
    debug_printf(DBG_INFO, "Radio BW is %d\r\n", radio_bw);
    int radio_sf = radio_cb["SF"].get<int>();
    debug_printf(DBG_INFO, "Radio SF is %d\r\n", radio_sf);
    int radio_cr = radio_cb["CR"].get<int>();
    debug_printf(DBG_INFO, "Radio CR is %d\r\n", radio_cr);
    int radio_freq = radio_cb["Frequency"].get<int>();
    debug_printf(DBG_INFO, "Radio Frequency is %d\r\n", radio_freq);
    int radio_pwr = radio_cb["TX Power"].get<int>();
    debug_printf(DBG_INFO, "Radio Power is %d\r\n", radio_pwr);    
    int radio_preamble_len = radio_cb["Preamble Length"].get<int>();
    debug_printf(DBG_INFO, "Initial Radio Preamble Length is %d\r\n", radio_preamble_len); 
    radio_preamble_len = radio_cb["Preamble Length"].get<int>();
    radio_cb["Preamble Length"] = radio_preamble_len + 7*(radio_cb["Frequencies"].size()+2);
    radio_preamble_len = radio_cb["Preamble Length"].get<int>();   
    debug_printf(DBG_INFO, "FHSS-Adjusted Radio Preamble Length is %d\r\n", radio_preamble_len); 
    int addr = radio_cb["Address"].get<int>();
    vector<uint32_t> freqs;
    for(int i = 0; i < radio_cb["Frequencies"].size(); i++) {
        freqs.push_back(radio_cb["Frequencies"][i].get<int>());
        debug_printf(DBG_INFO, "Frequencies PULLED %d\r\n", radio_cb["Frequencies"][i].get<int>());
    }
    radio.configure_freq_hop(addr, freqs);
    int pocsag_tx_freq = radio_cb["POCSAG Frequency"].get<int>();
    debug_printf(DBG_INFO, "POCSAG Frequency is %d\r\n", pocsag_tx_freq); 
    int pocsag_beacon_interval = radio_cb["POCSAG Beacon Interval"].get<int>();
    debug_printf(DBG_INFO, "POCSAG Beacon Interval is %d\r\n", pocsag_beacon_interval);
    string fec_algo = radio_cb["FEC Algorithm"].get<string>();
    debug_printf(DBG_INFO, "FEC algorithm is %s\r\n", fec_algo.c_str()); 
    if(fec_algo == "None") {
        frame_fec = make_shared<FEC>(Frame::size());
    }
    else if(fec_algo == "Interleave") {
        frame_fec = make_shared<FECInterleave>(Frame::size());
    }
    else if(fec_algo == "Convolutional") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        frame_fec = make_shared<FECConv>(Frame::size(), conv_rate, conv_order);
    }
    else if(fec_algo == "RSV") {
        int conv_rate = radio_cb["Conv Rate"].get<int>();
        int conv_order = radio_cb["Conv Order"].get<int>();
        int rs_roots = radio_cb["Reed-Solomon Number Roots"].get<int>();
        frame_fec = make_shared<FECRSV>(Frame::size(), conv_rate, conv_order, rs_roots);
    }
    else {
        MBED_ASSERT(false);
    }
    Frame tmp_frame(frame_fec);
    uint8_t full_pkt_len = tmp_frame.codedSize();
    radio_cb["Full Packet Size"] = full_pkt_len;
    debug_printf(DBG_INFO, "Setting RX size to %d\r\n", full_pkt_len);
    radio.set_rx_config(MODEM_LORA, radio_bw,
                            radio_sf, radio_cr,
                            0, radio_preamble_len,
                            radio_preamble_len, RADIO_FIXED_LEN,
                            full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(MODEM_LORA, radio_pwr, 0,
                            radio_bw, radio_sf,
                            radio_cr, radio_preamble_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
    radio.set_channel(*freqs.begin());
    radio_timing.computeTimes(radio_bw, radio_sf, radio_cr, radio_preamble_len, full_pkt_len);
    radio.cad_rx_timeout = radio_timing.pkt_time_us / 15.625f;
    int cw_test_mode = radio_cb["CW Test Mode"].get<int>();
    int pre_test_mode = radio_cb["Preamble Test Mode"].get<int>();
    int fec_test_mode = radio_cb["Test FEC"].get<int>();
    if(cw_test_mode == 1) { 
        debug_printf(DBG_WARN, "Starting continuous wave output...\r\n");
        radio.set_tx_continuous_wave(0, 0, 0);
        while(true) {
            ThisThread::sleep_for(1000);
        }
    }
    else if(pre_test_mode == 1) {
        debug_printf(DBG_WARN, "Starting continuous preamble output...\r\n");
        radio.set_tx_continuous_preamble();
        while(true) {
            ThisThread::sleep_for(1000);
        }
    }
    else if(fec_test_mode == 1) {
        debug_printf(DBG_WARN, "Starting the FEC testing...\r\n");
        testFEC();
    }
}

void reinit_radio(void) {
    // Initialize Radio driver
    int radio_bw = radio_cb["BW"].get<int>();
    int radio_sf = radio_cb["SF"].get<int>();
    int radio_cr = radio_cb["CR"].get<int>();
    int radio_freq = radio_cb["Frequency"].get<int>();
    int radio_pwr = radio_cb["TX Power"].get<int>();  
    int radio_preamble_len = radio_cb["Preamble Length"].get<int>();
    int pocsag_tx_freq = radio_cb["POCSAG Frequency"].get<int>();
    int pocsag_beacon_interval = radio_cb["POCSAG Beacon Interval"].get<int>();
    string fec_algo = radio_cb["FEC Algorithm"].get<string>();
    Frame tmp_frame(frame_fec);
    uint8_t full_pkt_len = radio_cb["Full Packet Size"].get<int>();
    int addr = radio_cb["Address"].get<int>();
#if 0
    vector<uint32_t> freqs;
    for(int i = 0; i < radio_cb["Frequencies"].size(); i++) {
        freqs.push_back(radio_cb["Frequencies"][i].get<int>());
    }
    radio.configure_freq_hop(addr, freqs);
#endif

    radio_events.tx_done_tmr = tx_done_cb;
    radio_events.rx_done_tmr = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.rx_preamble_det = rx_preamble_det_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    ThisThread::sleep_for(250);
    radio.init_radio(&radio_events);
    radio.set_rx_config(MODEM_LORA, radio_bw,
                            radio_sf, radio_cr,
                            0, radio_preamble_len,
                            radio_preamble_len, RADIO_FIXED_LEN,
                            full_pkt_len,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(MODEM_LORA, radio_pwr, 0,
                            radio_bw, radio_sf,
                            radio_cr, radio_preamble_len,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
}

void reinit_radio_pocsag(void) {
    // Initialize Radio driver
    int radio_bw = radio_cb["BW"].get<int>();
    int radio_sf = radio_cb["SF"].get<int>();
    int radio_cr = radio_cb["CR"].get<int>();
    //int radio_freq = radio_cb["Frequency"].get<int>();
    int radio_pwr = radio_cb["TX Power"].get<int>();  
    int radio_preamble_len = radio_cb["Preamble Length"].get<int>();
    int pocsag_tx_freq = radio_cb["POCSAG Frequency"].get<int>();
    int pocsag_beacon_interval = radio_cb["POCSAG Beacon Interval"].get<int>();
    string fec_algo = radio_cb["FEC Algorithm"].get<string>();
    Frame tmp_frame(frame_fec);
    uint8_t full_pkt_len = radio_cb["Full Packet Size"].get<int>();

    radio_events.tx_done_tmr = tx_done_cb;
    radio_events.rx_done_tmr = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.rx_preamble_det = rx_preamble_det_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    radio.init_radio(&radio_events);
    radio.set_tx_config_pocsag(radio_pwr);
    //radio.set_channel(pocsag_tx_freq);
    radio.rx_hop_frequency();
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum) {
    evt_enum = my_evt_enum;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, string &my_pocsag_msg) {
    evt_enum = my_evt_enum;
    pocsag_msg = my_pocsag_msg;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr_sptr) {
    tmr_sptr = my_tmr_sptr;
    evt_enum = my_evt_enum;
}

RadioEvent::RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr_sptr, const uint8_t *my_buf,
        shared_ptr<list<pair<uint32_t, uint8_t> > > my_rssi_list_sptr,
        const size_t my_size, const int16_t my_rssi, const int8_t my_snr) {
    rssi_list_sptr = my_rssi_list_sptr;
    tmr_sptr = my_tmr_sptr;
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
    //debug_printf(DBG_INFO, "TX done\r\n");
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
                                            (size_t) size, rssi, snr);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event);
    //debug_printf(DBG_INFO, "RX Done interrupt generated %d rssi %d snr %d\r\n", size, rssi, snr); 
}


static void cad_done_cb(const bool cad_detected) {
    if(!cad_detected) {
        radio.start_cad();
    }
    else {

    }
}


static void rx_preamble_det_cb(void) {
#if 0
    if(!deep_sleep_lock) {
        deep_sleep_lock = new DeepSleepLock();
    }
#endif
}


static void tx_timeout_cb(void)
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(TX_TIMEOUT_EVT);
    MBED_ASSERT(!tx_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(tx_radio_evt_mail, radio_event);
}
 
static void rx_timeout_cb(void)
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(RX_TIMEOUT_EVT);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event); 
#if 0
    if(deep_sleep_lock) {
        delete deep_sleep_lock;
    }
#endif   
}
 
static void rx_error_cb(void)
{
    radio.standby();
    auto radio_event = make_shared<RadioEvent>(RX_ERROR_EVT);
    MBED_ASSERT(!unified_radio_evt_mail.full());
    enqueue_mail<std::shared_ptr<RadioEvent> >(unified_radio_evt_mail, radio_event);
#if 0
    if(deep_sleep_lock) {
        delete deep_sleep_lock;
    }
#endif   
}

static void fhss_change_channel_cb(uint8_t current_channel) { }
