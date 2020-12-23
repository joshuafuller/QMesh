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
#include <random>
#include "qmesh.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

Mail<shared_ptr<Frame>, QUEUE_DEPTH> tx_frame_mail, rx_frame_mail, nv_logger_mail;

size_t Frame::size(void) {
    return radio_cb.net_cfg.pld_len + sizeof(hdr) + sizeof(crc);
}

void Frame::loadTestFrame(vector<uint8_t> &buf) {
    hdr.cons_subhdr.fields.type = 0;
    hdr.cons_subhdr.fields.stream_id = 1;
    hdr.var_subhdr.fields.ttl = 7;
    hdr.var_subhdr.fields.sender = 0xB;
    hdr.var_subhdr.fields.sym_offset = 0;
    data.assign(buf.begin(), buf.end());
    setCRC();
}

size_t Frame::codedSize(void) {
    return fec->encSize();
}

void Frame::serialize(vector<uint8_t> &ser_frame) {
    for(int i = 0; i < sizeof(hdr); i++) {
        ser_frame.push_back(((uint8_t *) &hdr)[i]);
    }
    copy(data.begin(), data.end(), back_inserter(ser_frame));
    for(int i = 0; i < sizeof(crc); i++) {
        ser_frame.push_back(crc.b[i]);
    }
}

void Frame::serialize_pb(vector<uint8_t> &buf) {
    DataMsg data_msg = DataMsg_init_zero;
    data_msg.type = getDataMsgType();
    data_msg.stream_id = hdr.cons_subhdr.fields.stream_id;
    data_msg.sender = hdr.var_subhdr.fields.sender;
    data_msg.ttl = hdr.var_subhdr.fields.ttl;
    data_msg.sym_offset = hdr.var_subhdr.fields.sym_offset;
    data_msg.payload.size = data.size();
    memcpy(data_msg.payload.bytes, data.data(), data.size());
    pb_byte_t buffer[DataMsg_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, DataMsg_fields, &data_msg);
    MBED_ASSERT(status);
    buf.resize(stream.bytes_written);
    memcpy(buf.data(), stream.state, stream.bytes_written);
}

void Frame::deserialize_pb(const vector<uint8_t> &buf) {
    DataMsg data_msg = DataMsg_init_default;
    pb_istream_t stream = pb_istream_from_buffer(buf.data(), buf.size());
    bool status = pb_decode(&stream, DataMsg_fields, &data_msg);
    MBED_ASSERT(status);
    hdr.cons_subhdr.fields.type = data_msg.type;
    hdr.cons_subhdr.fields.stream_id = data_msg.stream_id;
    hdr.var_subhdr.fields.sender = data_msg.sender;
    hdr.var_subhdr.fields.ttl = data_msg.ttl;
    hdr.var_subhdr.fields.sym_offset = data_msg.sym_offset;
    data.resize(data_msg.payload.size);
    memcpy(data.data(), data_msg.payload.bytes, data_msg.payload.size);
}

void Frame::whiten(const vector<uint8_t> &buf, vector<uint8_t> &wht_buf, const uint16_t seed) {
    mt19937 rand_gen(seed);
    for(vector<uint8_t>::const_iterator iter = buf.begin(); iter != buf.end(); iter++) {
        uint8_t rand_byte = rand_gen();
        wht_buf.push_back(*iter ^ rand_byte);
    }
}

size_t Frame::serializeCoded(vector<uint8_t> &buf) {
    //debug_printf(DBG_WARN, "Frame size is now %d\r\n", Frame::size());
    vector<uint8_t> ser_buf;
    serialize(ser_buf);
	//debug_printf(DBG_WARN, "Serialized frame size is now %d\r\n", ser_buf.size());
    return fec->encode(ser_buf, buf);
}

uint16_t Frame::calcCRC(void) {
    MbedCRC<POLY_16BIT_CCITT, 16> ct;
    uint32_t crc = 0;
    vector<uint8_t> crc_buf;
    for(int i = 0; i < sizeof(hdr.cons_subhdr); i++) {
        crc_buf.push_back(((uint8_t *) &hdr.cons_subhdr)[i]);
    }
    copy(data.begin(), data.end(), back_inserter(crc_buf));
    ct.compute((void *) crc_buf.data(), crc_buf.size(), &crc);
    return crc & 0x0000FFFF;
}

uint32_t Frame::calcUniqueCRC(void) {
    vector<uint8_t> buf;
    //buf.push_back((uint8_t) hdr.type);
    buf.push_back((uint8_t) hdr.cons_subhdr.fields.stream_id);
    copy(data.begin(), data.end(), back_inserter(buf));
    MbedCRC<POLY_32BIT_ANSI, 32> ct;
    uint32_t crc = 0;
    ct.compute((void *) buf.data(), buf.size(), &crc);
    return crc;        
}

PKT_STATUS_ENUM Frame::deserializeCoded(const shared_ptr<vector<uint8_t>> buf) {
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
    memcpy(&hdr, dec_buf.data(), sizeof(hdr));
    data.clear();
    copy(dec_buf.begin()+sizeof(hdr), dec_buf.end()-sizeof(crc), back_inserter(data));
    for(int i = 0; i < sizeof(crc); i++) {
        crc.b[i] = *(dec_buf.begin()+sizeof(hdr)+data.size()+i);
    }
    // Step three: check the payload CRC
    if(!checkCRC()) {
        pkt_status = PKT_BAD_CRC;
        return pkt_status;
    }
    // Size checked out, CRCs checked out, so return OK
    pkt_status = PKT_OK;
    return pkt_status;
}


void Frame::loadFromPB(const DataMsg &data_msg) {
    hdr.cons_subhdr.fields.type = data_msg.type;
    hdr.cons_subhdr.fields.stream_id = data_msg.stream_id;
    hdr.var_subhdr.fields.ttl = data_msg.ttl;
    hdr.var_subhdr.fields.sender = data_msg.sender;
    hdr.var_subhdr.fields.sym_offset = data_msg.sym_offset;
    crc.s = data_msg.crc;
    data.resize(data_msg.payload.size);
    memcpy((char *) data.data(), data_msg.payload.bytes, data_msg.payload.size); 
}


void Frame::saveToPB(DataMsg &data_msg) {
    data_msg.type = getDataMsgType();
    data_msg.stream_id = hdr.cons_subhdr.fields.stream_id;
    data_msg.ttl = hdr.var_subhdr.fields.ttl;
    data_msg.sender = hdr.var_subhdr.fields.sender;
    data_msg.sym_offset = hdr.var_subhdr.fields.sym_offset;
    data_msg.crc = crc.s;
    data_msg.payload.size = data.size();
    memcpy(data_msg.payload.bytes, data.data(), data.size());
}


void Frame::prettyPrint(const enum DBG_TYPES dbg_type) {
    debug_printf(dbg_type, "==========\r\n");
    debug_printf(dbg_type, "Frame Info\r\n");
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "HEADER\r\n");
    debug_printf(dbg_type, "Type: %2d | Stream ID: %2d\r\n", hdr.cons_subhdr.fields.type, 
                    hdr.cons_subhdr.fields.stream_id);
    debug_printf(dbg_type, "TTL:  %2d |    Sender: %2d\r\n", hdr.var_subhdr.fields.ttl, 
                    hdr.var_subhdr.fields.sender);
    debug_printf(dbg_type, "Offsets -- %2d (pre), %2d (nsym), %2d (sym)\r\n", 
                    hdr.var_subhdr.fields.sym_offset);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "PAYLOAD\r\n");
    for(size_t i = 0; i < data.size(); i++) {
        if(i%16 == 0) {
            debug_printf(dbg_type, "");
        }
        debug_printf_clean(dbg_type, "%2x ", data[i]);
    }
    debug_printf_clean(dbg_type, "\r\n");
    debug_printf(dbg_type, "CRC: %4x\r\n", crc.s);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "Rx Stats: %d (RSSI), %d (SNR)\r\n", rssi, snr);
}


static Mutex dbg_printf_mutex;
int debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    dbg_printf_mutex.lock();
    va_list args;
    va_start(args, fmt);
    static char tmp_str[512];
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
    auto ser_msg_sptr = shared_ptr<SerialMsg>();
    *ser_msg_sptr = SerialMsg_init_zero;
    ser_msg_sptr->has_dbg_msg = true;
    sprintf(ser_msg_sptr->dbg_msg.msg, "[+] %s -- %s", msg_type.c_str(), tmp_str);   
    while(tx_ser_queue.size() > 128) { ThisThread::sleep_for(100); }
    tx_ser_queue_lock.lock();
    tx_ser_queue.push_back(ser_msg_sptr);
    tx_ser_queue_lock.unlock();
    if(dbg_type == DBG_ERR) { // Make DEBUG_ERR events throw an asssert
        ThisThread::sleep_for(2000);
        MBED_ASSERT(false);
    }
    va_end(args);
    dbg_printf_mutex.unlock();
    return 0;
}

int debug_printf_clean(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    static char tmp_str[512];
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
    auto ser_msg_sptr = shared_ptr<SerialMsg>();
    *ser_msg_sptr = SerialMsg_init_zero;
    ser_msg_sptr->has_dbg_msg = true;
    strncpy(ser_msg_sptr->dbg_msg.msg, tmp_str, 256);      
    while(tx_ser_queue.size() > 128) { ThisThread::sleep_for(100); }
    tx_ser_queue_lock.lock();
    tx_ser_queue.push_back(ser_msg_sptr);
    tx_ser_queue_lock.unlock();

    va_end(args);
    return 0;
}


void rx_frame_ser_thread_fn(void) {
    for(;;) {
        auto rx_frame_sptr = dequeue_mail<std::shared_ptr<Frame>>(rx_frame_mail);
        auto ser_msg_sptr = shared_ptr<SerialMsg>();
        *ser_msg_sptr = SerialMsg_init_zero;
        ser_msg_sptr->has_data_msg = true;
        rx_frame_sptr->saveToPB(ser_msg_sptr->data_msg);
        ser_msg_sptr->data_msg.type = DataMsg_Type_RX;
        while(tx_ser_queue.size() > 128) { ThisThread::sleep_for(100); }
        tx_ser_queue_lock.lock();
        tx_ser_queue.push_back(ser_msg_sptr);
        tx_ser_queue_lock.unlock();
    }
}
