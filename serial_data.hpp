 /*
 * Copyright (c) 2019, Daniel R. Fay.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SERIAL_DATA_HPP
#define SERIAL_DATA_HPP

#include "mbed.h"
#include "MbedJSONValue.h"
#include <string>
#include "params.hpp"
#include "nv_settings.hpp"
#include "json_serial.hpp"

#if (MBED_CONF_APP_LORA_RADIO == SX1272)
#include "SX1272_LoRaRadio.h"
extern SX1272_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX1276)
#include "SX1276_LoRaRadio.h"
extern SX1276_LoRaRadio radio;
#elif (MBED_CONF_APP_LORA_RADIO == SX126X)
#include "SX126X_LoRaRadio.h"
extern SX126X_LoRaRadio radio;
#endif

extern NVSettings *nv_settings;
static uint8_t enc_buf[512], dec_buf[256];


// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
enum DBG_TYPES {
    DBG_INFO,
    DBG_WARN,
    DBG_ERR,
};

int debug_printf(const enum DBG_TYPES, const char *fmt, ...);

int debug_printf_clean(const enum DBG_TYPES, const char *fmt, ...);

typedef enum {
    PKT_OK = 0,
    PKT_FEC_FAIL,
    PKT_BAD_HDR_CRC,
    PKT_BAD_PLD_CRC,
    PKT_BAD_SIZE,
    PKT_UNITIALIZED,
} PKT_STATUS_ENUM;

class Frame {
    typedef struct __attribute__((__packed__)) {
        uint32_t type : 2;
        uint32_t stream_id : 8;
        uint32_t ttl : 3;
        uint32_t sender : 4;
        uint32_t pre_offset : 3;
        uint32_t nsym_offset : 3;
        uint32_t sym_offset : 3;
    } frame_hdr;
    typedef struct __attribute__((__packed__)) {
        frame_hdr hdr;
        uint16_t hdr_crc;
        // payload
        uint8_t data[FRAME_PAYLOAD_LEN];
        uint16_t data_crc;
    } frame_pkt;
protected:
    frame_pkt pkt;
    // receive stats
    int16_t rssi;
    int8_t snr;
    uint16_t rx_size;
    PKT_STATUS_ENUM pkt_status;

public:
    // Call operator. Just loads the object with the contents
    // of the other object.
    void load(Frame &frame) {
        memcpy(&pkt, &frame.pkt, sizeof(pkt));
        rssi = frame.rssi;
        snr = frame.snr;
        rx_size = frame.rx_size;
    }

    // Load the frame with a payload and dummy values.
    void loadTestFrame(uint8_t *buf);

    // Load the payload into a buffer. Returns the number of bytes put into 
    //  the buffer that's supplied as an argument.
    size_t getPayload(uint8_t *buf) {
        memcpy(buf, pkt.data, FRAME_PAYLOAD_LEN);
        return FRAME_PAYLOAD_LEN;
    }

    // Get an array of bytes of the frame for e.g. transmitting over the air.
    size_t serialize(uint8_t *buf);

    // Compute the header CRC.
    uint16_t calculateHeaderCrc(void);

    // Compute the payload CRC.
    uint16_t calculatePayloadCrc(void);

    // Calculate the CRC for the "unique" information.
    // The unique information consists of the type, stream_id, and payload.
    uint32_t calculateUniqueCrc(void);

    // Check the header CRC. Returns true if a match, false if not.
    bool checkHeaderCrc(void) {
        return (pkt.hdr_crc == calculateHeaderCrc());
    }

    // Check the payload CRC. Returns true if a match, false if not.
    bool checkPayloadCrc(void) {
        return (pkt.data_crc == calculatePayloadCrc());
    }

    // Check the integrity of the packet by checking both the header and payload CRCs
    bool checkIntegrity(void) {
        return (checkHeaderCrc() & checkPayloadCrc());
    }

    // Set the header CRC, by computing it based on the current header data.
    // Returns the computed CRC.
    uint16_t setHeaderCrc(void) {
        uint16_t crc = calculateHeaderCrc();
        pkt.hdr_crc = crc;
        return crc;
    }

    // Set the payload CRC, by computing it based on the current payload data.
    // Returns the computed CRC.
    uint16_t setPayloadCrc(void) {
        uint16_t crc = calculatePayloadCrc();
        pkt.data_crc = crc;
        return crc;
    }

    // Take an array of bytes and unpack it into the object's internal data structures.
    // In the process of doing this, it checks the packet for various things, returning
    // the following:
    //  1. PKT_BAD_HDR_CRC -- the header CRC is bad.
    //  2. PKT_BAD_PLD_CRC -- the payload CRC is bad.
    //  3. PKT_BAD_SIZE -- the received bytes do not match the packet size.
    //  4. PKT_FEC_FAIL -- FEC decode failed.
    //  5. PKT_OK -- the received packet data is ok
    PKT_STATUS_ENUM deserialize(shared_ptr<vector<uint8_t>> buf); 
    PKT_STATUS_ENUM deserialize(const uint8_t *buf, const size_t bytes_rx);

    // Increment the TTL, updating the header CRC in the process.
    void incrementTTL(void) {
        pkt.hdr.ttl += 1;
        setHeaderCrc();
    }

    // Get the frame's current TTL value
    uint8_t getTTL(void) {
        return pkt.hdr.ttl;
    }

    // Get the size of a packet
    static size_t getPktSize(void) {
        return sizeof(frame_pkt);
    }

    // Get the size of a packet header
    static size_t getHdrSize(void) {
        return sizeof(frame_hdr);
    }

    // Get the size of a packet with fec
    size_t getFullPktSize(void);

    // Get the offsets from the packet header
    void getOffsets(uint8_t *pre_offset, uint8_t *nsym_offset, uint8_t *sym_offset) {
        *pre_offset = pkt.hdr.pre_offset;
        *nsym_offset = pkt.hdr.nsym_offset;
        *sym_offset = pkt.hdr.sym_offset;
    }

    // Set the offsets to the packet header
    void setOffsets(const uint8_t *pre_offset, const uint8_t *nsym_offset, const uint8_t *sym_offset) {
        pkt.hdr.pre_offset = *pre_offset;
        pkt.hdr.nsym_offset = *nsym_offset;
        pkt.hdr.sym_offset = *sym_offset;
    }

    // Get/set the receive stats
    void getRxStats(int16_t *rssi, int8_t *snr, uint16_t *rx_size) {
        *rssi = this->rssi;
        *snr = this->snr;
        *rx_size = this->rx_size;
    }

    void setRxStats(int16_t rssi, int8_t snr, uint16_t rx_size) {
        this->rssi = rssi;
        this->snr = snr;
        this->rx_size = rx_size;
    }

    // Pretty-print the Frame.
    void prettyPrint(const enum DBG_TYPES dbg_type);

    // Load the frame with a parsed JSON object
    void loadFromJSON(MbedJSONValue &json);

    // Save the frame's contents to a JSON object.
    void saveToJSON(MbedJSONValue &json);
};







#endif /* SERIAL_DATA_HPP */