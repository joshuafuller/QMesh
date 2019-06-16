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

#include <mbed.h>
#include <ATCmdParser.h>
#include "serial_data.hpp"
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include <string>
#include "fec.hpp"

extern FEC *fec; // forward error correction block

// Load the frame with a payload and dummy values.
void Frame::loadTestFrame(uint8_t *buf) {
    pkt.hdr.type = 0;
    pkt.hdr.stream_id = 1;
    pkt.hdr.ttl = 8;
    pkt.hdr.sender = 0xAB;
    pkt.hdr.pre_offset = 0;
    pkt.hdr.nsym_offset = 0;
    pkt.hdr.sym_offset = 0;
    memcpy(pkt.data, buf, FRAME_PAYLOAD_LEN);
    setHeaderCrc();
    setPayloadCrc();
}

// Get the size of a packet with fec
size_t Frame::getFullPktSize(void) {
    return fec->getEncSize(getPktSize());
}

// Get an array of bytes of the frame for e.g. transmitting over the air.
size_t Frame::serialize(uint8_t *buf) {
    debug_printf(DBG_WARN, "Frame size is now %d\r\n", sizeof(pkt));
    return fec->encode((uint8_t *) &pkt, sizeof(pkt), buf)/8;
}

// Compute the header CRC.
uint16_t Frame::calculateHeaderCrc(void) {
    // Header
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    uint32_t crc = 0;
    ct.compute((void *) &pkt.hdr, sizeof(pkt.hdr), &crc);
    return crc & 0x0000FFFF;
}

// Compute the payload CRC.
uint16_t Frame::calculatePayloadCrc(void) {
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    uint32_t crc = 0;
    ct.compute((void *) &pkt.data, sizeof(pkt.data), &crc);
    return crc & 0x0000FFFF;
}

// Calculate the CRC for the "unique" information.
// The unique information consists of the type, stream_id, and payload.
uint32_t Frame::calculateUniqueCrc(void) {
    const size_t buf_size = sizeof(pkt.data)+sizeof(pkt.hdr);
    uint8_t buf[buf_size];
    size_t byte_cnt = 0;
    memcpy(buf, &pkt.hdr, sizeof(pkt.hdr));
    memcpy(buf+sizeof(pkt.hdr), &pkt.data, sizeof(pkt.data));
    MbedCRC<POLY_32BIT_ANSI, 32> ct;
    uint32_t crc = 0;
    ct.compute((void *) buf, buf_size, &crc);
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
PKT_STATUS_ENUM Frame::deserialize(const uint8_t *buf, const size_t bytes_rx) {
    // Step zero: remove the forward error correction
    static uint8_t dec_buf[512];
    ssize_t bytes_dec = fec->decode(buf, bytes_rx, dec_buf);
    if(bytes_dec == -1) {
        pkt_status = PKT_FEC_FAIL;
        return pkt_status;
    }
    // Step one: check the size of the packet
    if(sizeof(pkt) != bytes_dec) {
        pkt_status = PKT_BAD_SIZE;
        return pkt_status;
    }
    // Step two: load the packet data and check the header CRC
    memcpy(&pkt, dec_buf, sizeof(pkt));
    if(!checkHeaderCrc()) {
        pkt_status = PKT_BAD_HDR_CRC;
        return pkt_status;
    }
    // Step three: check the payload CRC
    if(!checkPayloadCrc()) {
        pkt_status = PKT_BAD_PLD_CRC;
        return pkt_status;
    }
    // Size checked out, CRCs checked out, so return OK
    pkt_status = PKT_OK;
    return pkt_status;
}

// Pretty-print the Frame.
void Frame::prettyPrint(const enum DBG_TYPES dbg_type) {
    debug_printf(dbg_type, "==========\r\n");
    debug_printf(dbg_type, "Frame Info\r\n");
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "HEADER\r\n");
    debug_printf(dbg_type, "Type: %2d | Stream ID: %2d\r\n", pkt.hdr.type, pkt.hdr.stream_id);
    debug_printf(dbg_type, "TTL:  %2d |    Sender: %2d\r\n", pkt.hdr.ttl, pkt.hdr.sender);
    debug_printf(dbg_type, "Offsets -- %2d (pre), %2d (nsym), %2d (sym)\r\n", 
        pkt.hdr.pre_offset, pkt.hdr.nsym_offset, pkt.hdr.sym_offset);
    debug_printf(dbg_type, "CRC: %4x\r\n", pkt.hdr_crc);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "PAYLOAD\r\n");
    char *pld_str = (char *) malloc(2048);
    MBED_ASSERT(pld_str != NULL);
    for(size_t i = 0; i < FRAME_PAYLOAD_LEN; i++) {
        if(i%16 == 0) {
            debug_printf(dbg_type, "");
        }
        debug_printf_clean(dbg_type, "%2x ", pkt.data[i]);
    }
    debug_printf_clean(dbg_type, "\r\n");
    debug_printf(dbg_type, "CRC: %4x\r\n", pkt.data_crc);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "Rx Stats: %d (RSSI), %d (SNR)\r\n", rssi, snr);

    free(pld_str);
}

bool FrameQueue::enqueue(Frame &enq_frame) {
    if(queue.full()) 
        return false;
    Frame *tmp_frame = queue.alloc();
    tmp_frame->load(enq_frame);
    queue.put(tmp_frame);
    return true;
}

bool FrameQueue::enqueue(uint8_t *buf, const size_t buf_size) {
    if(queue.full()) 
        return false;
    Frame *tmp_frame = queue.alloc();
    tmp_frame->deserialize(buf, buf_size);
    queue.put(tmp_frame);
    return true;
}

// Dequeue a frame. Copies the dequeued data into the frame if successful,
//  returning true. Returns false if unsuccessful (queue is empty).
bool FrameQueue::dequeue(Frame &deq_frame) {
    osEvent evt = queue.get();
    if(evt.status == osEventMail) {
        Frame *tmp_frame = (Frame *) evt.value.p;
        deq_frame.load(*tmp_frame);
        queue.free(tmp_frame);
        return true;
    }
    return false;
}

// Returns whether queue is empty
bool FrameQueue::getEmpty(void) {
    return queue.empty();
}

// Returns whether queue is full
bool FrameQueue::getFull(void) {
    return queue.full();
}

FrameQueue tx_queue, rx_queue;


// The AT command parser for the serial interface.
// AT commands we'd like to have:
//
// AT+Freq? -- get the frequency
// -- <value> -- current frequency
//
// AT+Freq -- set the frequency
// -- OK -- response
// <data> -- frequency value
// -- OK -- response
//
// AT+SF? -- get the spreading factor
// -- <value> -- current spreading factor
//
// AT+SF -- set the spreading factor
// -- OK -- response
// <value> -- spreading factor
// -- OK -- response
//
// AT+BW? -- get the bandwidth
// -- <value> -- current bandwidth
//
// AT+BW -- set the bandwidth
// -- OK -- response
// <value> -- bandwidth value
// -- OK -- response 
//
// AT+RXDEQ? -- Dequeue the RX queue
// -- <value> -- RX frame 
//
// AT+RXCNT? -- get the number of RX frames waiting in the queue
// -- <value> -- number of RX frames
//
// AT+RXCLR -- clear the RX queue
// -- OK -- response
//
// AT+TXENQ -- Load the TX queue with another frame
// -- OK -- response
// <data> -- TX frame data
// -- OK -- response
//
// AT+TXCNT? -- get the number of TX frames waiting in the queue
// -- <value> -- number of TX frames
//
// AT+TXCLR -- clear the TX queue
// -- OK -- response
//
// AT+NVS -- save settings to Non-Volatile Storage (NVS)
// -- OK -- response
//
//
#if 0 
char response[64];
char recv_data[256];
void processATCmds(ATCmdParser *at) {
    at->recv("AT+%s", response);
    if(!strcmp(response, "CHECK?")) {
        at->send("CHECK OK");
    }
    else if(!strcmp(response, "FREQ?")) {
        at->send(radio->freq);
    } 
    else if(!strcmp(response, "FREQ")) {
        at->send("OK");
        unt32_t new_freq;
        at->recv("%d", new_freq);
        if(new_freq > 900000000 || new_freq < 930000000) {
            at->send("ERR OUT_OF_RANGE");   
        }
        else {
            radio->freq = new_freq;
            at->send("OK");   
        }
    }
    else if(!strcmp(response, "SF?")) {
        at->send(radio->sf);
    }
    else if(!strcmp(response, "SF")) {
        at->send("OK");
        int32_t new_sf;
        at->recv("%d", new_sf);
        if(new_sf < 6 || new_sf > 12) {
            at->send("ERR INVALID");
        }
        else {
            radio->sf = new_sf;
            at->send("OK");   
        }
    }
    else if(!strcmp(response, "BW?")) {
        at->send(radio->bw);
    }
    else if(!strcmp(response, "BW")) {
        at->send("OK");
        int32_t new_bw;
        at->recv("%d", new_bw);
        if(new_bw == 125000 || new_bw == 250000 || new_bw == 500000) {
            radio->bw = new_bw;
            at->send("OK");
        }
        else {
            at->send("ERR INVALID");   
        }
    }
    else if(!strcmp(response, "RXDEQ?")) {
        int errcode = rx_queue->dequeue();
        if(!errcode) {
            at->send(rx_queue->dequeue());
        }
        else {
            at->send("ERR %d", errcode);
        }
    }
    else if(!strcmp(response, "RXCNT?")) {
        at->send(rx_queue->getCount());
    }
    else if(!strcmp(response, "RXCLR")) {
        tx_queue->clear();
        at->send("OK");
    }
    else if(!strcmp(response, "TXENQ")) {
        at->send("OK");
        at->recv("%s", recv_data);
        int errcode = tx_queue->enqueue(recv_data);
        if(!errcode) {
            at->send("OK");
        }
        else {
            at->send("ERR %d", errcode);
        }
    }
    else if(!strcmp(response, "TXCNT?")) {
        at->send(tx_queue->getSize());
    }
    else if(!strcmp(response, "TXCLR")) {
        int errcode = tx_queue->clear();
        if(!errcode) {
            at->send("OK");
        else {
            at->send("ERR %d", errcode);   
        }
    }
    else if(!strcmp(response, "NVS")) {
        int errcode = nv_settings->save();
        if(!errcode) {
            at->send("OK");
        }
        else {
            at->send("ERR %d", errcode);   
        }
    }
    else if(!strcmp(response, "NVCLR")) {  
        clearNVSettings();
        at->send("OK");
    }   
    else if(!strcmp(response, "ENABLE_MESH")) {
        mesh_enable = true;
        at->send("OK");
    }
    else if(!strcmp(response, "DISABLE_MESH")) {
        mesh_enable = false;
        at->send("OK");
    }    
}
#endif



// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
static char tmp_str[512];
int debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    vsprintf(tmp_str, fmt, args);
    string msg_type;
    if(dbg_type == DBG_INFO) {
        #ifndef DEBUG_INFO
        return 0;
        #endif
        msg_type = "INFO";
    }
    else if(dbg_type == DBG_WARN) {
        #ifndef DEBUG_WARN
        return 0;
        #endif
        msg_type = "WARN";
    }
    else if(dbg_type == DBG_ERR) {
        #ifndef DEBUG_ERR
        return 0;
        #endif
        msg_type = "ERR ";
    }
    else {
        MBED_ASSERT(false);
    }
    int ret_val = printf("[+] %s -- %s", msg_type.c_str(), tmp_str);

    va_end(args);
    return ret_val;
}

int debug_printf_clean(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    vsprintf(tmp_str, fmt, args);
    string msg_type;
    if(dbg_type == DBG_INFO) {
        #ifndef DEBUG_INFO
        return 0;
        #endif
        msg_type = "INFO";
    }
    else if(dbg_type == DBG_WARN) {
        #ifndef DEBUG_WARN
        return 0;
        #endif
        msg_type = "WARN";
    }
    else if(dbg_type == DBG_ERR) {
        #ifndef DEBUG_ERR
        return 0;
        #endif
        msg_type = "ERR ";
    }
    else {
        MBED_ASSERT(false);
    }
    int ret_val = printf("%s", tmp_str);

    va_end(args);
    return ret_val;
}


// Takes an incoming Base64-encoded frame, decodes it, and stuffs it into the tx queue.
static uint8_t base64_buf[512];
void enqueue_tx_frames(const uint8_t *b64_tx_frame, const size_t b64_tx_frame_len) {
    size_t frame_size;
    int ret = mbedtls_base64_decode(base64_buf, sizeof(base64_buf), &frame_size, b64_tx_frame, b64_tx_frame_len);
    MBED_ASSERT(ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL);
    MBED_ASSERT(ret != MBEDTLS_ERR_BASE64_INVALID_CHARACTER);
    if(frame_size == Frame::getPktSize()) {
        tx_queue.enqueue(base64_buf, frame_size);
        debug_printf(DBG_INFO, "Enqueued Tx frame\r\n");
    }
    else {
        debug_printf(DBG_ERR, "Decoded bytes not equal to a frame. %d decoded, frame size is %d\r\n",
            frame_size, Frame::getPktSize());
    }
}