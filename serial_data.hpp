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
#include "params.hpp"
#include "nv_settings.hpp"
#include "SX1272_LoRaRadio.h"

extern SX1272_LoRaRadio radio;
class Frame {
    typedef struct {
        uint8_t type;
        uint16_t stream_id;
        uint8_t ttl;        
    } frame_hdr;
    typedef struct {
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

    public:
    // Call operator. Just loads the object with the contents
    // of the other object.
    void load(Frame &frame) {
        memcpy(&pkt, &frame.pkt, sizeof(pkt));
        rssi = frame.rssi;
        snr = frame.snr;
        rx_size = frame.rx_size;
    }

    // Get an array of bytes of the frame for e.g. transmitting over the air.
    size_t serialize(uint8_t *buf) {
        memcpy(buf, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }

    // Take an array of bytes and unpack it into the object's internal data structures.
    // It also internally checks to see whether this is a properly_sized frame.
    void deserialize(const uint8_t *buf, const size_t bytes_rx) {
        MBED_ASSERT(bytes_rx == sizeof(pkt));
        memcpy(&pkt, buf, sizeof(pkt));
    }

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

};

class FrameQueue {
    protected:
        Mail<Frame, 32> queue;
    public:
        // Enqueue a frame. Returns true if enqueue was successful,
        //  false if unsuccessful due to overflow
        bool enqueue(Frame &enq_frame);
        bool enqueue(const uint8_t *buf, const size_t buf_size);

        // Dequeue a frame. Copies the dequeued data into the frame if successful,
        //  returning true. Returns false if unsuccessful (queue is empty).
        bool dequeue(Frame &frame);

        // Returns whether queue is empty
        bool getEmpty(void);

        // Returns whether queue is full
        bool getFull(void);
};


// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
enum DBG_TYPES {
    DBG_INFO,
    DBG_WARN,
    DBG_ERROR,
};
int debug_printf(const enum DBG_TYPES, const char *fmt, ...);


#endif /* SERIAL_DATA_HPP */