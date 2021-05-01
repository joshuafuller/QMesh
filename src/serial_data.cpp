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

#include <mbed.h>
#include <ATCmdParser.h>
#include "serial_data.hpp"
#include <string>
#include <vector>
#include "fec.hpp"
#include "kiss_serial.hpp"
#include <random>
#include "qmesh.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"


Mail<shared_ptr<Frame>, QUEUE_DEPTH> tx_frame_mail, rx_frame_mail, nv_logger_mail;

static const SerialMsg ser_msg_zero = SerialMsg_init_zero;
static const DataMsg data_msg_zero = DataMsg_init_zero;

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
    for(size_t i = 0; i < sizeof(hdr); i++) {
        ser_frame.push_back(((uint8_t *) &hdr)[i]);
    }
    copy(data.begin(), data.end(), back_inserter(ser_frame));
    for(size_t i = 0; i < sizeof(crc); i++) {
        ser_frame.push_back(crc);
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

uint8_t Frame::calcCRC(void) {
    MbedCRC<POLY_8BIT_CCITT, 8> ct;
    uint32_t crc = 0;
    vector<uint8_t> crc_buf;
    for(size_t i = 0; i < sizeof(hdr.cons_subhdr); i++) {
        crc_buf.push_back(((uint8_t *) &hdr.cons_subhdr)[i]);
    }
    copy(data.begin(), data.end(), back_inserter(crc_buf));
    ct.compute((void *) crc_buf.data(), crc_buf.size(), &crc);
    return crc & 0x000000FF;
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
    if((ssize_t) Frame::size() != bytes_dec) {
        //debug_printf(DBG_INFO, "Bad packet size\r\n");
        pkt_status = PKT_BAD_SIZE;
        return pkt_status;
    }
    // Step two: load the packet data and check the header CRC
    memcpy(&hdr, dec_buf.data(), sizeof(hdr));
    data.clear();
    copy(dec_buf.begin()+sizeof(hdr), dec_buf.end()-sizeof(crc), back_inserter(data));
    for(size_t i = 0; i < sizeof(crc); i++) {
        crc = *(dec_buf.begin()+sizeof(hdr)+data.size()+i);
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
    crc = data_msg.crc;
    data.resize(data_msg.payload.size);
    memcpy((char *) data.data(), data_msg.payload.bytes, data_msg.payload.size); 
}


void Frame::saveToPB(DataMsg &data_msg) {
    data_msg.type = getDataMsgType();
    data_msg.stream_id = hdr.cons_subhdr.fields.stream_id;
    data_msg.ttl = hdr.var_subhdr.fields.ttl;
    data_msg.sender = hdr.var_subhdr.fields.sender;
    data_msg.sym_offset = hdr.var_subhdr.fields.sym_offset;
    data_msg.crc = crc;
    if(data_msg.type == DataMsg_Type_KISSRX || data_msg.type == DataMsg_Type_KISSTX) {
        kiss_subhdr k_sub;
        memcpy(&k_sub, data.data(), sizeof(k_sub));
        data_msg.payload.size = k_sub.fields.size;
        memcpy(data_msg.payload.bytes, data.data()+sizeof(k_sub), k_sub.fields.size);
        data_msg.kiss_cur_frame = k_sub.fields.cur_frame;
        data_msg.kiss_tot_frames = k_sub.fields.tot_frames;
    } else {
        data_msg.payload.size = data.size();
        memcpy(data_msg.payload.bytes, data.data(), data.size());
    }
}


void Frame::createFromKISS(DataMsg &data_msg) {
    hdr.var_subhdr.fields.ttl = 0;
    hdr.var_subhdr.fields.sender = radio_cb.address;
    hdr.var_subhdr.fields.sym_offset = 0;
    hdr.cons_subhdr.fields.stream_id = data_msg.kiss_stream_id;
    hdr.cons_subhdr.fields.type = DataMsg_Type_KISSTX;

    kiss_subhdr kshdr;
    kshdr.fields.size = data_msg.payload.size;
    kshdr.fields.cur_frame = data_msg.kiss_cur_frame;
    kshdr.fields.tot_frames = data_msg.kiss_tot_frames;

    data.resize(radio_cb.net_cfg.pld_len);
    memcpy((char *) data.data(), (char *) &kshdr, sizeof(kshdr));
    memcpy((char *) data.data()+sizeof(kshdr), (char *) data_msg.payload.bytes, data_msg.payload.size);
    setCRC();
}


size_t Frame::getKISSMaxSize(void) {
    return radio_cb.net_cfg.pld_len-sizeof(kiss_subhdr);
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
    debug_printf(dbg_type, "CRC: %4x\r\n", crc);
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "Rx Stats: %d (RSSI), %d (SNR)\r\n", rssi, snr);
}


static Mutex dbg_printf_mutex;
static char tmp_str[512];
int debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    dbg_printf_mutex.lock();
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
    auto ser_msg_sptr = make_shared<SerialMsg>();
    *ser_msg_sptr = ser_msg_zero;
    ser_msg_sptr->type = SerialMsg_Type_DEBUG_MSG;
    ser_msg_sptr->has_dbg_msg = true;
    sprintf(ser_msg_sptr->dbg_msg.msg, "[+] %s -- %s", msg_type.c_str(), tmp_str);   
    kiss_sers_mtx.lock();
    for(vector<KISSSerial *>::iterator iter = kiss_sers.begin(); iter != kiss_sers.end(); iter++) {
        auto ser_msg_sptr_en = make_shared<SerialMsg>(*ser_msg_sptr);
        (*iter)->enqueue_msg(ser_msg_sptr_en);
    }
    kiss_sers_mtx.unlock();
    if(dbg_type == DBG_ERR) { // Make DEBUG_ERR events throw an asssert
        ThisThread::sleep_for(2000);
        MBED_ASSERT(false);
    }
    va_end(args);
    dbg_printf_mutex.unlock();
    return 0;
}

static char tmp_str_clean[512];
int debug_printf_clean(const enum DBG_TYPES dbg_type, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsprintf(tmp_str_clean, fmt, args);
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
    *ser_msg_sptr = ser_msg_zero;
    ser_msg_sptr->has_dbg_msg = true;
    strncpy(ser_msg_sptr->dbg_msg.msg, tmp_str_clean, 256);
    kiss_sers_mtx.lock();
    for(vector<KISSSerial *>::iterator iter = kiss_sers.begin(); iter != kiss_sers.end(); iter++) {
        auto ser_msg_sptr_en = make_shared<SerialMsg>(*ser_msg_sptr);
        (*iter)->enqueue_msg(ser_msg_sptr_en);
    }
    kiss_sers_mtx.unlock();
    va_end(args);
    return 0;
}



static Mutex frame_map_mtx;
extern EventQueue background_queue;
typedef struct {
    uint8_t tot_frames;
    uint8_t stream_id;
    int tag;
    vector<shared_ptr<DataMsg>> frags;
} frag_info_t;
typedef map<uint8_t, frag_info_t> frag_map_t; 
static frag_map_t frag_map;
atomic<int> cur_tag(0);


static void purge_frag_map_entry(uint8_t stream_id, int tag);
static void purge_frag_map_entry(uint8_t stream_id, int tag) {
    frame_map_mtx.lock();
    if(frag_map.find(stream_id) != frag_map.end()) { // Make sure it's not already erased
        // Tag tracks whether this entry has already been erased and a new stream is being tracked
        if(tag == frag_map.find(stream_id)->second.tag) {
            frag_map.erase(stream_id);
        }
        debug_printf(DBG_INFO, "Purged stream id %d from frag map\r\n", stream_id);
    } else {
        debug_printf(DBG_INFO, "Tried to purge stream id %d from frag map, but frag already reassembled\r\n", 
                        stream_id);        
    }
    frame_map_mtx.unlock();
}


static shared_ptr<DataMsg> handle_incoming_frag(shared_ptr<DataMsg> frag);
static shared_ptr<DataMsg> handle_incoming_frag(shared_ptr<DataMsg> frag) {
    // If the KISS frame is a single fragment, just return
    shared_ptr<DataMsg> ret_val = NULL;
    if(frag->kiss_tot_frames == 1) {
        // Queue up assembled frame
        ret_val = frag;
    } else { // Otherwise, handle reassembly
        frame_map_mtx.lock();
        frag_map_t::iterator iter = frag_map.find(frag->stream_id);
        if(iter == frag_map.end()) { // first fragment of this KISS frame being tracked
            pair<uint8_t, frag_info_t> elem;
            elem.first = frag->stream_id;
            elem.second.stream_id = frag->stream_id;
            elem.second.tot_frames = frag->kiss_tot_frames;
            elem.second.tag = cur_tag++;
            elem.second.frags.push_back(frag);
            frag_map.insert(elem);
            background_queue.call_in(30000, &purge_frag_map_entry, frag->stream_id, elem.second.tag);
        } else {
            iter->second.frags.push_back(frag);
        }
        iter = frag_map.find(frag->stream_id);
        if(iter->second.tot_frames == iter->second.frags.size()) { // We got all the fragments
            vector<pair<uint8_t, shared_ptr<DataMsg>>> sort_frags;
            for(vector<shared_ptr<DataMsg>>::iterator it = iter->second.frags.begin(); // Sort the frames
                it != iter->second.frags.end(); it++) {
                sort_frags.push_back(pair<uint8_t, shared_ptr<DataMsg>>((**it).kiss_cur_frame, *it));
            }
            sort(sort_frags.begin(), sort_frags.end());
            vector<uint8_t> frag_comb;
            for(vector<pair<uint8_t, shared_ptr<DataMsg>>>::iterator it = sort_frags.begin();
                it != sort_frags.end(); it++) {
                for(int i = 0; i < it->second->payload.size; i++) {
                    frag_comb.push_back(it->second->payload.bytes[i]);
                }
            }
            auto frag_comb_datamsg = make_shared<DataMsg>();
            *frag_comb_datamsg = *(sort_frags.begin()->second);
            frag_comb_datamsg->type = DataMsg_Type_KISSRX;
            frag_comb_datamsg->payload.size = frag_comb.size();
            memcpy(frag_comb_datamsg->payload.bytes, frag_comb.data(), frag_comb.size());
            frag_map.erase(frag->stream_id);
            ret_val = frag_comb_datamsg;
        }
        frame_map_mtx.unlock();
    }
    return ret_val;
}


void rx_frame_ser_thread_fn(void) {
    for(;;) {
        auto rx_frame_sptr = dequeue_mail<std::shared_ptr<Frame>>(rx_frame_mail);
        auto ser_msg_sptr = make_shared<SerialMsg>();
        *ser_msg_sptr = ser_msg_zero;
        ser_msg_sptr->type = SerialMsg_Type_DATA;
        ser_msg_sptr->has_data_msg = false;
        rx_frame_sptr->saveToPB(ser_msg_sptr->data_msg);
        // Handle KISS frames vs. "regular" QMesh frames
        if(ser_msg_sptr->data_msg.type == DataMsg_Type_KISSRX || 
            ser_msg_sptr->data_msg.type == DataMsg_Type_KISSTX) {
            ser_msg_sptr->data_msg.type = DataMsg_Type_KISSRX;
            auto frag = make_shared<DataMsg>(ser_msg_sptr->data_msg);
            auto pkt = handle_incoming_frag(frag);
            if(pkt != NULL) {
                ser_msg_sptr->has_data_msg = true;
                ser_msg_sptr->data_msg = *pkt;
            }
        } else {
            ser_msg_sptr->data_msg.type = DataMsg_Type_RX;
            ser_msg_sptr->has_data_msg = true;
        }
        if(ser_msg_sptr->has_data_msg) { // Send it out over the serial ports
            kiss_sers_mtx.lock();
            for(vector<KISSSerial *>::iterator iter = kiss_sers.begin(); iter != kiss_sers.end(); iter++) {
                auto ser_msg_sptr_en = make_shared<SerialMsg>(*ser_msg_sptr);
                (*iter)->enqueue_msg(ser_msg_sptr_en);
            }
            kiss_sers_mtx.unlock();
        }
    }
}
