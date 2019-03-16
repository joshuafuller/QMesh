#include "lora_radio_helper.h"
#include "params.hpp"

// Included from lora_radio_helper.h is a radio object for our radio.
// Also, init 
void init_radio(void) {
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
    radio.LoRaRadio::set_public_network(false);
    radio.LoRaRadio::set_channel(RADIO_FREQUENCY);
}