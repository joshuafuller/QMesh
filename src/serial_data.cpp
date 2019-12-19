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

#include <mbed.h>
#include <ATCmdParser.h>
#include "serial_data.hpp"
#include "mbedtls/platform.h"
#include "mbedtls/base64.h"
#include <string>
#include <vector>
#include "fec.hpp"
#include "json_serial.hpp"

Mail<shared_ptr<Frame>, 16> tx_frame_mail, rx_frame_mail, nv_logger_mail;

void Frame::loadTestFrame(vector<uint8_t> &buf) {
    pkt.hdr.type = 0;
    pkt.hdr.stream_id = 1;
    pkt.hdr.ttl = 7;
    pkt.hdr.sender = 0xB;
    pkt.hdr.pre_offset = 0;
    pkt.hdr.nsym_offset = 0;
    pkt.hdr.sym_offset = 0;
    data.assign(buf.begin(), buf.end());
    setHeaderCrc();
    setPayloadCrc();
}

size_t Frame::getFullPktSize(void) {
    return fec->getEncSize(getPktSize());
}

size_t Frame::serialize(vector<uint8_t> &buf) {
    //debug_printf(DBG_WARN, "Frame size is now %d\r\n", Frame::size());
    vector<uint8_t> ser_frame;
    for(int i = 0; i < sizeof(pkt); i++) {
        ser_frame.push_back(((uint8_t *) &pkt)[i]);
    }
    copy(data.begin(), data.end(), back_inserter(ser_frame));
    return fec->encode(ser_frame, buf);
}

uint16_t Frame::calculateHeaderCrc(void) {
    // Header
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    uint32_t crc = 0;
    ct.compute((void *) &pkt.hdr, sizeof(pkt.hdr), &crc);
    return crc & 0x0000FFFF;
}

uint16_t Frame::calculatePayloadCrc(void) {
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    uint32_t crc = 0;
    ct.compute((void *) data.data(), data.size(), &crc);
    return crc & 0x0000FFFF;
}

uint32_t Frame::calculateUniqueCrc(void) {
    vector<uint8_t> buf;
    buf.push_back((uint8_t) pkt.hdr.type);
    buf.push_back((uint8_t) pkt.hdr.stream_id);
    copy(data.begin(), data.end(), back_inserter(buf));
    MbedCRC<POLY_32BIT_ANSI, 32> ct;
    uint32_t crc = 0;
    ct.compute((void *) buf.data(), buf.size(), &crc);
    return crc;        
}

PKT_STATUS_ENUM Frame::deserialize(const std::shared_ptr<vector<uint8_t>> buf) {
    // Step zero: remove the forward error correction
    static vector<uint8_t> dec_buf;
    //debug_printf(DBG_WARN, "Received %d bytes\r\n", buf->size());
    ssize_t bytes_dec = fec->decode(*buf, dec_buf);
    //debug_printf(DBG_WARN, "Decoded into %d bytes\r\n", bytes_dec);
    if(bytes_dec == -1) {
        //debug_printf(DBG_INFO, "FEC Failed\r\n");
        pkt_status = PKT_FEC_FAIL;
        return pkt_status;
    }
    // Step one: check the size of the packet
    if(Frame::size() != bytes_dec) {
        //debug_printf(DBG_INFO, "Bad packet size\r\n");
        pkt_status = PKT_BAD_SIZE;
        return pkt_status;
    }
    // Step two: load the packet data and check the header CRC
    memcpy(&pkt, dec_buf.data(), sizeof(pkt));
    data.clear();
    copy(dec_buf.begin()+sizeof(pkt), dec_buf.end(), back_inserter(data));
    if(!checkHeaderCrc()) {
        //debug_printf(DBG_INFO, "Bad header CRC\r\n");
        pkt_status = PKT_BAD_HDR_CRC;
        return pkt_status;
    }
    // Step three: check the payload CRC
    if(!checkPayloadCrc()) {
        //debug_printf(DBG_INFO, "Bad payload CRC\r\n");
        pkt_status = PKT_BAD_PLD_CRC;
        return pkt_status;
    }
    // Size checked out, CRCs checked out, so return OK
    //debug_printf(DBG_INFO, "Packet OK\r\n");
    pkt_status = PKT_OK;
    return pkt_status;
}

void Frame::loadFromJSON(MbedJSONValue &json) {
    pkt.hdr.type = json["HDR Pkt Type"].get<int>();
    pkt.hdr.stream_id = json["HDR Stream ID"].get<int>();
    pkt.hdr.ttl = json["HDR TTL"].get<int>();
    pkt.hdr.sender = json["HDR Sender"].get<int>();
    pkt.hdr.pre_offset = json["HDR Pre Offset"].get<int>();
    pkt.hdr.nsym_offset = json["HDR Num Sym Offset"].get<int>();
    pkt.hdr.sym_offset = json["HDR Sym Offset"].get<int>();
    pkt.hdr_crc.crc = json["Header CRC"].get<int>();
    pkt.data_crc.crc = json["Data CRC"].get<int>();
    string b64_str = json["Data Payload"].get<string>();
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_decode(NULL, 0, &b64_len, 
        (uint8_t *) b64_str.c_str(), b64_str.size()) == 0);
    MBED_ASSERT(b64_len == radio_cb["Payload Length"].get<int>());
    data.resize(b64_len);
    MBED_ASSERT(mbedtls_base64_decode(data.data(), b64_len, &b64_len, 
        (uint8_t *) b64_str.c_str(), b64_str.size()) == 0);
}

void Frame::saveToJSON(MbedJSONValue &json) {
    json["Type"] = "Frame";
    json["HDR Pkt Type"] = pkt.hdr.type;
    json["HDR Stream ID"] = pkt.hdr.stream_id;
    json["HDR TTL"] = pkt.hdr.ttl;
    json["HDR Sender"] = pkt.hdr.sender;
    json["HDR Pre Offset"] = pkt.hdr.pre_offset;
    json["HDR Num Sym Offset"] = pkt.hdr.nsym_offset;
    json["HDR Sym Offset"] = pkt.hdr.sym_offset;
    json["Header CRC"] = pkt.hdr_crc.crc;
    json["Data CRC"] = pkt.data_crc.crc;
    size_t b64_len;
    MBED_ASSERT(mbedtls_base64_encode(NULL, 0, &b64_len, data.data(), data.size()) == 0);
    unsigned char *b64_buf = new unsigned char[b64_len];
    MBED_ASSERT(mbedtls_base64_encode(b64_buf, b64_len, &b64_len, data.data(), data.size()) == 0);
    json["Data Payload"] = b64_buf;

    delete [] b64_buf;
}

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
    for(size_t i = 0; i < data.size(); i++) {
        if(i%16 == 0) {
            debug_printf(dbg_type, "");
        }
        debug_printf_clean(dbg_type, "%2x ", data[i]);
    }
    debug_printf_clean(dbg_type, "\r\n");
    debug_printf(dbg_type, "CRC: %4x\r\n", pkt.data_crc);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "Rx Stats: %d (RSSI), %d (SNR)\r\n", rssi, snr);
}


static Mutex dbg_printf_mutex;
int debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    dbg_printf_mutex.lock();
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
    char dbg_str_data[sizeof(tmp_str)+32];
    sprintf(dbg_str_data, "[+] %s -- %s", msg_type.c_str(), tmp_str);
    dbg_str = dbg_str_data;
    auto tx_str = make_shared<string>();
    JSONSerial json_ser;
    json_ser.dbgPrintfToJSON(dbg_str, *tx_str);
    auto tx_str_sptr = tx_ser_queue.alloc();
    *tx_str_sptr = tx_str;
    MBED_ASSERT(tx_ser_queue.full() == false);
    tx_ser_queue.put(tx_str_sptr);

    va_end(args);
    dbg_printf_mutex.unlock();
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
    auto tx_str = make_shared<string>();
    JSONSerial json_ser;
    json_ser.dbgPrintfToJSON(dbg_str, *tx_str);
    auto tx_str_sptr = tx_ser_queue.alloc();
    *tx_str_sptr = tx_str;
    while(tx_ser_queue.full() == true);
    tx_ser_queue.put(tx_str_sptr);

    va_end(args);
    return 0;
}