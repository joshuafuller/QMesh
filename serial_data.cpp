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
#include <vector>
#include "fec.hpp"
#include "json_serial.hpp"

extern FEC *fec; // forward error correction block

// Load the frame with a payload and dummy values.
void Frame::loadTestFrame(uint8_t *buf) {
    pkt.hdr.type = 0;
    pkt.hdr.stream_id = 1;
    pkt.hdr.ttl = 7;
    pkt.hdr.sender = 0xB;
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
PKT_STATUS_ENUM Frame::deserialize(std::shared_ptr<vector<uint8_t>> buf) {
    uint8_t tmp_buf[256];
    MBED_ASSERT(buf->size() > 256);
    memcpy(tmp_buf, buf->data(), buf->size());
    return this->deserialize(tmp_buf, buf->size());
}

PKT_STATUS_ENUM Frame::deserialize(const uint8_t *buf, const size_t bytes_rx) {
    // Step zero: remove the forward error correction
    static uint8_t dec_buf[512];
    debug_printf(DBG_WARN, "Received %d bytes\r\n", bytes_rx);
    ssize_t bytes_dec = fec->decode(buf, bytes_rx, dec_buf);
    debug_printf(DBG_WARN, "Decoded into %d bytes\r\n", bytes_dec);
    if(bytes_dec == -1) {
        debug_printf(DBG_INFO, "FEC Failed\r\n");
        pkt_status = PKT_FEC_FAIL;
        return pkt_status;
    }
    // Step one: check the size of the packet
    if(sizeof(pkt) != bytes_dec) {
        debug_printf(DBG_INFO, "Bad packet size\r\n");
        pkt_status = PKT_BAD_SIZE;
        return pkt_status;
    }
    // Step two: load the packet data and check the header CRC
    memcpy(&pkt, dec_buf, sizeof(pkt));
    if(!checkHeaderCrc()) {
        debug_printf(DBG_INFO, "Bad header CRC\r\n");
        pkt_status = PKT_BAD_HDR_CRC;
        return pkt_status;
    }
    // Step three: check the payload CRC
    if(!checkPayloadCrc()) {
        debug_printf(DBG_INFO, "Bad payload CRC\r\n");
        pkt_status = PKT_BAD_PLD_CRC;
        return pkt_status;
    }
    // Size checked out, CRCs checked out, so return OK
    debug_printf(DBG_INFO, "Packet OK\r\n");
    pkt_status = PKT_OK;
    return pkt_status;
}

// Load the frame with a parsed JSON object
void Frame::loadFromJSON(MbedJSONValue &json) {
    pkt.hdr.type = json["HDR Pkt Type"].get<int>();
    pkt.hdr.stream_id = json["HDR Stream ID"].get<int>();
    pkt.hdr.ttl = json["HDR TTL"].get<int>();
    pkt.hdr.sender = json["HDR Sender"].get<int>();
    pkt.hdr.pre_offset = json["HDR Pre Offset"].get<int>();
    pkt.hdr.nsym_offset = json["HDR Num Sym Offset"].get<int>();
    pkt.hdr.sym_offset = json["HDR Sym Offset"].get<int>();
    pkt.hdr_crc = json["Header CRC"].get<int>();
    pkt.data_crc = json["Data CRC"].get<int>();
    string b64_str = json["Data Payload"].get<string>();
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_decode(NULL, 0, &b64_len, 
        (uint8_t *) b64_str.c_str(), b64_str.size()) == 0);
    MBED_ASSERT(b64_len == FRAME_PAYLOAD_LEN);
    uint8_t *frame_data = new uint8_t[b64_len];
    MBED_ASSERT(mbedtls_base64_decode(frame_data, b64_len, &b64_len, 
        (uint8_t *) b64_str.c_str(), b64_str.size()) == 0);
    memcpy(pkt.data, frame_data, FRAME_PAYLOAD_LEN);

    delete [] frame_data;
}

// Save the frame's contents to a JSON object.
void Frame::saveToJSON(MbedJSONValue &json) {
    json["HDR Pkt Type"] = pkt.hdr.type;
    json["HDR Stream ID"] = pkt.hdr.stream_id;
    json["HDR TTL"] = pkt.hdr.ttl;
    json["HDR Sender"] = pkt.hdr.sender;
    json["HDR Pre Offset"] = pkt.hdr.pre_offset;
    json["HDR Num Sym Offset"] = pkt.hdr.nsym_offset;
    json["HDR Sym Offset"] = pkt.hdr.sym_offset;
    json["Header CRC"] = pkt.hdr_crc;
    json["Data CRC"] = pkt.data_crc;
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, pkt.data, FRAME_PAYLOAD_LEN) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, pkt.data, FRAME_PAYLOAD_LEN) == 0);
    json["Data Payload"] = b64_buf;

    delete [] b64_buf;
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


ATSettings::ATSettings(Serial *ser_port,  NVSettings *settings) {
    nv_settings = settings;
    at = new ATCmdParser(ser_port);
    parser_thread.start(callback(this, &ATSettings::threadFn));
}

void ATSettings::threadFn(void) {
    for(;;) {
        processATCmds();
    }
}

ATSettings::~ATSettings(void) {
    parser_thread.terminate();
    delete at;
}

void ATSettings::processATCmds(void) {
    int32_t val;
    at->recv("AT+%s=%d", cmd, &val);
	if(!strcmp(cmd, "FREQ?")) {
		at->send("%d", nv_settings->nv_settings.freq);
	}
    else if(!strcmp(cmd, "FREQ")) {
		nv_settings->nv_settings.freq = val;
		nv_settings->saveEEPROM();
		at->send("OK");
    }
	else if(!strcmp(cmd, "SF?")) {
		at->send("%d", nv_settings->nv_settings.sf);
	}
	else if(!strcmp(cmd, "SF")) {
		nv_settings->nv_settings.sf = val;
		nv_settings->saveEEPROM();
		at->send("OK");
	}    
	else if(!strcmp(cmd, "BW?")) {
		at->send("%d", nv_settings->nv_settings.bw);
	}
    else if(!strcmp(cmd, "BW")) {
        nv_settings->nv_settings.bw = val;
		nv_settings->saveEEPROM();
		at->send("OK");
    }
	else {
		at->send("ERR UNKNOWN CMD");
	}
}


// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
int debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char tmp_str[512];
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
    //int ret_val = printf("[+] %s -- %s", msg_type.c_str(), tmp_str);
    string dbg_str;
    char dbg_str_data[sizeof(tmp_str)+32];
    sprintf(dbg_str_data, "[+] %s -- %s", msg_type.c_str(), tmp_str);
    dbg_str = dbg_str_data;
    std::shared_ptr<string> tx_str(new string());
    JSONSerial json_ser;
    json_ser.dbgPrintfToJSON(dbg_str, *tx_str);
    auto tx_str_sptr = tx_ser_queue.alloc();
    *tx_str_sptr = tx_str;
    MBED_ASSERT(tx_ser_queue.full() == false);
    tx_ser_queue.put(tx_str_sptr);

    va_end(args);
    return 0;
}

int debug_printf_clean(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char tmp_str[512];
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
    string dbg_str;
    std::shared_ptr<string> tx_str(new string());;
    JSONSerial json_ser;
    json_ser.dbgPrintfToJSON(dbg_str, *tx_str);
    auto tx_str_sptr = tx_ser_queue.alloc();
    *tx_str_sptr = tx_str;
    MBED_ASSERT(tx_ser_queue.full() == false);
    tx_ser_queue.put(tx_str_sptr);

    va_end(args);
    return 0;
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