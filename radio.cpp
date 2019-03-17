#include "lora_radio_helper.h"
#include "mbed.h"
#include "params.hpp"
#include <string>

// Time on air, in ms
uint32_t time_on_air_ms = 0;
float time_on_air_s = 0.0;

// The callbacks used by the LoRa radio driver
static radio_events_t radio_events;

// Prototypes for the callbacks
static void tx_done_cb(void);
static void rx_done_cb(const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void tx_timeout_cb(void);
static void rx_timeout_cb(void);
static void rx_error_cb(void);
static void fhss_change_channel_cb(uint8_t current_channel);

// Included from lora_radio_helper.h is a radio object for our radio.
// Let's set it up.
void init_radio(void) {
    // Initialize Radio driver
    radio_events.tx_done = tx_done_cb;
    radio_events.rx_done = rx_done_cb;
    radio_events.rx_error = rx_error_cb;
    radio_events.tx_timeout = tx_timeout_cb;
    radio_events.rx_timeout = rx_timeout_cb;
    radio_events.fhss_change_channel = fhss_change_channel_cb;
    radio.init_radio(&radio_events);
    radio.set_rx_config(1, RADIO_BANDWIDTH,
                            RADIO_SF, RADIO_CODERATE,
                            0, RADIO_PREAMBLE_LEN,
                            RADIO_SYM_TIMEOUT, RADIO_FIXED_LEN,
                            FRAME_PAYLOAD_LEN,
                            RADIO_CRC_ON, RADIO_FREQ_HOP, RADIO_HOP_PERIOD,
                            RADIO_INVERT_IQ, true);
    radio.set_tx_config(1, RADIO_POWER, 0,
                            RADIO_BANDWIDTH, RADIO_SF,
                            RADIO_CODERATE, RADIO_PREAMBLE_LEN,
                            RADIO_FIXED_LEN, RADIO_CRC_ON, RADIO_FREQ_HOP,
                            RADIO_HOP_PERIOD, RADIO_INVERT_IQ, RADIO_TX_TIMEOUT);
    radio.set_public_network(false);
    radio.set_channel(RADIO_FREQUENCY);
    time_on_air_ms = radio.time_on_air(1, FRAME_PAYLOAD_LEN);
    time_on_air_s = time_on_air_ms/1000.0;
}

// Simple function that just sends out a simple packet every 1s
static std::string send_str("KG5VBY: This is a test string\r\n");
static char send_string[256];
void test_radio(void) {
    while(true) {
        memcpy(send_string, send_str.c_str(), send_str.length());
        radio.send((uint8_t *) send_string, send_str.length());
        wait(1.0);
    }
}

static void tx_done_cb(void)
{
    radio.sleep();
}
 
static void rx_done_cb(const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    radio.sleep();
}
 
static void tx_timeout_cb(void)
{
    radio.sleep();
}
 
static void rx_timeout_cb(void)
{
    radio.sleep();
}
 
static void rx_error_cb(void)
{
    radio.sleep();
}

static void fhss_change_channel_cb(uint8_t current_channel) {

}