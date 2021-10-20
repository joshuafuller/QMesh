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
#include "serial_data.hpp"
#include "voice_msg.hpp"


Mail<shared_ptr<Frame>, QUEUE_DEPTH> tx_frame_mail, rx_frame_mail, nv_logger_mail;
static Mutex Frame_mutex;
static mt19937 Frame_stream_id_rng; //NOLINT
static atomic<int> Frame_last_stream_id;

auto Frame::size() -> size_t {
    MBED_ASSERT(radio_cb.valid);
    size_t my_size = radio_cb.net_cfg.pld_len + sizeof(hdr) + sizeof(crc);
    MBED_ASSERT(my_size <= 256);
    return my_size;
}

void Frame::loadTestFrame(vector<uint8_t> &buf) {
    MBED_ASSERT(buf.size() <= 256);
    MBED_ASSERT(!buf.empty());
    constexpr int NUM_HOPS = 7;
    constexpr int SENDER_ADDR = 11;
    hdr.cons_subhdr.fields.type = 0;
    hdr.cons_subhdr.fields.stream_id = 1;
    hdr.var_subhdr.fields.ttl = NUM_HOPS;
    hdr.var_subhdr.fields.sender = SENDER_ADDR;
    hdr.var_subhdr.fields.sym_offset = 0;
    data.assign(buf.begin(), buf.end());
    setCRC();
}

auto Frame::codedSize() -> size_t {
    return fec->encSize();
}

void Frame::serialize(vector<uint8_t> &ser_frame) {
    MBED_ASSERT(!ser_frame.empty());
    for(size_t i = 0; i < sizeof(hdr); i++) {
        ser_frame.push_back(((uint8_t *) &hdr)[i]); //NOLINT
    }
    copy(data.begin(), data.end(), back_inserter(ser_frame));
    for(size_t i = 0; i < sizeof(crc); i++) {
        ser_frame.push_back(crc);
    }
}

void Frame::serialize_pb(vector<uint8_t> &buf) {
    MBED_ASSERT(!buf.empty());
    DataMsg data_msg = DataMsg_init_zero;
    data_msg.type = getDataMsgType();
    data_msg.stream_id = hdr.cons_subhdr.fields.stream_id;
    data_msg.sender = hdr.var_subhdr.fields.sender;
    data_msg.ttl = hdr.var_subhdr.fields.ttl;
    data_msg.sym_offset = hdr.var_subhdr.fields.sym_offset;
    data_msg.payload.size = data.size();
    memcpy(data_msg.payload.bytes, data.data(), data.size());
    pb_byte_t buffer[DataMsg_size]; //NOLINT
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, DataMsg_fields, &data_msg);
    MBED_ASSERT(status);
    buf.resize(stream.bytes_written);
    memcpy(buf.data(), stream.state, stream.bytes_written);
}

void Frame::deserialize_pb(const vector<uint8_t> &buf) {
    MBED_ASSERT(!buf.empty());
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
    MBED_ASSERT(!buf.empty());
    wht_buf.clear();
    mt19937 rand_gen(seed);
    for(uint8_t iter : buf) {
        uint8_t rand_byte = rand_gen();
        wht_buf.push_back(iter ^ rand_byte);
    }
}

auto Frame::serializeCoded(vector<uint8_t> &buf) -> size_t {
    //debug_printf(DBG_WARN, "Frame size is now %d\r\n", Frame::size());
    buf.clear();
    vector<uint8_t> ser_buf;
    serialize(ser_buf);
    MBED_ASSERT(!ser_buf.empty());
	//debug_printf(DBG_WARN, "Serialized frame size is now %d\r\n", ser_buf.size());
    return fec->encode(ser_buf, buf);
}

auto Frame::serializeCodedInv(vector<uint8_t> &buf) -> size_t {
    buf.clear();
    size_t ret_val = 0;
    ret_val = serializeCoded(buf);
    MBED_ASSERT(!buf.empty());
    constexpr uint8_t ALL_ONES = 0xFF;
    for(uint8_t & it : buf) {
        it = it ^ ALL_ONES;
    }
    return ret_val;
}

auto Frame::calcCRC() -> uint8_t {
    constexpr int CRC_SIZE = 8;
    constexpr uint8_t POLY_8BIT_CDMA2000 = 0x9B;
    constexpr uint8_t INIT_8BIT_CDMA2000 = 0xFF;
    MbedCRC<POLY_8BIT_CDMA2000, CRC_SIZE> ct(INIT_8BIT_CDMA2000, 0x00, false, false);
    uint32_t crc = 0;
    vector<uint8_t> crc_buf;
    for(size_t i = 0; i < sizeof(hdr.cons_subhdr); i++) {
        crc_buf.push_back(((uint8_t *) &hdr.cons_subhdr)[i]); //NOLINT
    }
    copy(data.begin(), data.end(), back_inserter(crc_buf));
    ct.compute(static_cast<void *>(crc_buf.data()), crc_buf.size(), &crc);
    constexpr uint32_t LOWEST_BYTE_MASK = 0x000000FFU;
    return crc & LOWEST_BYTE_MASK;
}

auto Frame::calcUniqueCRC() -> uint32_t {
    vector<uint8_t> buf;
    //buf.push_back((uint8_t) hdr.type);
    buf.push_back(static_cast<uint8_t>(hdr.cons_subhdr.fields.stream_id)); //NOLINT
    copy(data.begin(), data.end(), back_inserter(buf));
    static constexpr int BITS_32 = 32;
    MbedCRC<POLY_32BIT_ANSI, BITS_32> ct;
    uint32_t crc = 0;
    ct.compute(buf.data(), buf.size(), &crc);
    return crc;        
}

auto Frame::deserializeCoded(const shared_ptr<vector<uint8_t>> &buf) -> PKT_STATUS_ENUM {
    MBED_ASSERT(!buf->empty());
    MBED_ASSERT(buf->size() <= 256);
    // Step zero: remove the forward error correction
    static vector<uint8_t> dec_buf;
    //debug_printf(DBG_WARN, "Received %d bytes\r\n", buf->size());
    ssize_t bytes_dec = 0;
    bytes_dec = fec->decode(*buf, dec_buf);
    //debug_printf(DBG_WARN, "Decoded into %d bytes\r\n", bytes_dec);
    if(bytes_dec == -1) {
        //debug_printf(DBG_INFO, "FEC Failed\r\n");
        pkt_status = PKT_FEC_FAIL;
        return pkt_status;
    }
    // Step one: check the size of the packet
    if(static_cast<ssize_t>(Frame::size()) != bytes_dec) {
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


auto Frame::deserializeCodedInv(const shared_ptr<vector<uint8_t>> &buf) -> PKT_STATUS_ENUM {
    MBED_ASSERT(!buf->empty());
    MBED_ASSERT(buf->size() <= 256);    
    auto buf_inv = make_shared<vector<uint8_t>>(*buf);
    // invert the encoded bits
    constexpr uint8_t ALL_ONES = 0xFF;
    for(uint8_t & it : *buf_inv) {
        it = it ^ ALL_ONES;
    }
    return deserializeCoded(buf_inv);
}


void Frame::loadFromPB(const DataMsg &data_msg) {
    hdr.cons_subhdr.fields.type = data_msg.type; //NOLINT
    hdr.cons_subhdr.fields.stream_id = data_msg.stream_id; //NOLINT
    hdr.var_subhdr.fields.ttl = data_msg.ttl; //NOLINT
    hdr.var_subhdr.fields.sender = data_msg.sender; //NOLINT
    hdr.var_subhdr.fields.sym_offset = data_msg.sym_offset; //NOLINT
    crc = data_msg.crc;
    data.resize(data_msg.payload.size);
    memcpy(data.data(), data_msg.payload.bytes, data_msg.payload.size); 
}


void Frame::saveToPB(DataMsg &data_msg) {
    data_msg.type = getDataMsgType();
    data_msg.stream_id = hdr.cons_subhdr.fields.stream_id; //NOLINT
    data_msg.ttl = hdr.var_subhdr.fields.ttl; //NOLINT
    data_msg.sender = hdr.var_subhdr.fields.sender; //NOLINT
    data_msg.sym_offset = hdr.var_subhdr.fields.sym_offset; //NOLINT
    data_msg.crc = crc;
    data_msg.redundant = redundant;
    if(data_msg.type == DataMsg_Type_KISSRX || data_msg.type == DataMsg_Type_KISSTX) {
        kiss_subhdr k_sub;
        memcpy(&k_sub, data.data(), sizeof(k_sub));
        data_msg.payload.size = k_sub.fields.size;
        memcpy(data_msg.payload.bytes, data.data()+sizeof(k_sub), k_sub.fields.size); //NOLINT
        data_msg.kiss_cur_frame = k_sub.fields.cur_frame;
        data_msg.kiss_tot_frames = k_sub.fields.tot_frames;
    } else {
        data_msg.payload.size = data.size();
        memcpy(data_msg.payload.bytes, data.data(), data.size());
    }
}


void Frame::createFromKISS(DataMsg &data_msg) {
    MBED_ASSERT(radio_cb.valid);
    hdr.var_subhdr.fields.ttl = 0; //NOLINT
    hdr.var_subhdr.fields.sender = radio_cb.address; //NOLINT
    hdr.var_subhdr.fields.sym_offset = 0; //NOLINT
    hdr.cons_subhdr.fields.stream_id = data_msg.kiss_stream_id; //NOLINT
    hdr.cons_subhdr.fields.type = DataMsg_Type_KISSTX; //NOLINT

    kiss_subhdr kshdr;
    kshdr.fields.size = data_msg.payload.size;
    kshdr.fields.cur_frame = data_msg.kiss_cur_frame;
    kshdr.fields.tot_frames = data_msg.kiss_tot_frames;

    data.resize(radio_cb.net_cfg.pld_len);
    memcpy(data.data(), &kshdr, sizeof(kshdr));
    memcpy(data.data()+sizeof(kshdr), data_msg.payload.bytes, data_msg.payload.size); //NOLINT
    setCRC();
}


void Frame::createFromVoice(VoiceMsgProcessor &vmp) {
    MBED_ASSERT(radio_cb.valid);
    hdr.var_subhdr.fields.ttl = 0; //NOLINT
    hdr.var_subhdr.fields.sender = radio_cb.address; //NOLINT
    hdr.var_subhdr.fields.sym_offset = 0; //NOLINT
    hdr.cons_subhdr.fields.stream_id = Frame::createStreamID(); //NOLINT
    hdr.cons_subhdr.fields.type = DataMsg_Type_VOICETX; //NOLINT
    vector<uint8_t> pld = vmp.getDataPayload();
    MBED_ASSERT(pld.size() == radio_cb.net_cfg.pld_len);
    data.resize(pld.size());
    copy(pld.begin(), pld.end(), data.begin());
    this->setCRC();
}


auto Frame::getKISSMaxSize() -> size_t {
    MBED_ASSERT(radio_cb.valid);
    return radio_cb.net_cfg.pld_len-sizeof(kiss_subhdr);
}


auto Frame::createStreamID() -> uint8_t {
    uniform_int_distribution<uint8_t> timing_off_dist(0, MAX_UINT8_VAL); 
    Frame_mutex.lock();
    uint8_t new_stream_id = 0;
    do {
        new_stream_id = timing_off_dist(Frame_stream_id_rng);
    } while(new_stream_id == Frame_last_stream_id);
    Frame_last_stream_id = new_stream_id;
    Frame_mutex.unlock();
    return new_stream_id;
}


void Frame::seed_stream_id(const int seed) {
    Frame_stream_id_rng.seed(seed);
}


void Frame::prettyPrint(const enum DBG_TYPES dbg_type) {
    debug_printf(dbg_type, "==========\r\n");
    debug_printf(dbg_type, "Frame Info\r\n");
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "HEADER\r\n");
    debug_printf(dbg_type, "Type: %2d | Stream ID: %2d\r\n", hdr.cons_subhdr.fields.type, //NOLINT
                    hdr.cons_subhdr.fields.stream_id); //NOLINT
    debug_printf(dbg_type, "TTL:  %2d |    Sender: %2d\r\n", hdr.var_subhdr.fields.ttl, //NOLINT
                    hdr.var_subhdr.fields.sender); //NOLINT
    debug_printf(dbg_type, "Offsets -- %2d (pre), %2d (nsym), %2d (sym)\r\n", 
                    hdr.var_subhdr.fields.sym_offset); //NOLINT
    debug_printf(dbg_type, "----------\r\n");
    debug_printf(dbg_type, "PAYLOAD\r\n");
    for(size_t i = 0; i < data.size(); i++) {
        constexpr int BYTES_PER_ROW = 16;
        if(i % BYTES_PER_ROW == 0) {
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
auto debug_printf(const enum DBG_TYPES dbg_type, const char *fmt, ...) -> int {
    dbg_printf_mutex.lock();
    vector<char> tmp_str(DbgMsg_size);
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp_str.data(), tmp_str.size(), fmt, args);
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
    auto ser_msg_sptr = make_shared<SerMsg>();
    ser_msg_sptr->type(SerialMsg_Type_DEBUG_MSG);
    snprintf(ser_msg_sptr->dbg_msg().msg, sizeof(ser_msg_sptr->dbg_msg().msg), 
                "[+] %s -- %s", msg_type.c_str(), tmp_str.data());   
    kiss_sers_mtx.lock();
    for(auto & kiss_ser : kiss_sers) {
        auto ser_msg_sptr_en = make_shared<SerMsg>(*ser_msg_sptr);
        kiss_ser->enqueue_msg(ser_msg_sptr_en);
    }
    kiss_sers_mtx.unlock();
    if(dbg_type == DBG_ERR) { // Make DEBUG_ERR events throw an asssert
        constexpr int TWO_SEC = 2000;
        ThisThread::sleep_for(TWO_SEC);
        MBED_ASSERT(false);
    }
    va_end(args);
    dbg_printf_mutex.unlock();
    return 0;
}


auto debug_printf_clean(const enum DBG_TYPES dbg_type, const char *fmt, ...) -> int {
    va_list args;
    vector<char> tmp_str(DbgMsg_size);
    va_start(args, fmt);
    vsnprintf(tmp_str.data(), tmp_str.size(), fmt, args);
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
    auto ser_msg_sptr = shared_ptr<SerMsg>();
    strncpy(ser_msg_sptr->dbg_msg().msg, tmp_str.data(), sizeof(ser_msg_sptr->dbg_msg().msg));
    kiss_sers_mtx.lock();
    for(auto & kiss_ser : kiss_sers) {
        auto ser_msg_sptr_en = make_shared<SerMsg>(*ser_msg_sptr);
        kiss_ser->enqueue_msg(ser_msg_sptr_en);
    }
    kiss_sers_mtx.unlock();
    va_end(args);
    return 0;
}



static Mutex frame_map_mtx;
extern EventQueue background_queue;
using frag_info_t = struct {
    uint8_t tot_frames{};
    uint8_t stream_id{};
    int tag{};
    vector<shared_ptr<DataMsg>> frags;
};
using frag_map_t = map<uint8_t, frag_info_t>; 
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


static auto handle_incoming_frag(const shared_ptr<DataMsg> &frag) -> shared_ptr<DataMsg>;
static auto handle_incoming_frag(const shared_ptr<DataMsg> &frag) -> shared_ptr<DataMsg> {
    // If the KISS frame is a single fragment, just return
    shared_ptr<DataMsg> ret_val = nullptr;
    if(frag->kiss_tot_frames == 1) {
        // Queue up assembled frame
        ret_val = frag;
    } else { // Otherwise, handle reassembly
        frame_map_mtx.lock();
        auto iter = frag_map.find(frag->stream_id);
        if(iter == frag_map.end()) { // first fragment of this KISS frame being tracked
            pair<uint8_t, frag_info_t> elem;
            elem.first = frag->stream_id;
            elem.second.stream_id = frag->stream_id;
            elem.second.tot_frames = frag->kiss_tot_frames;
            elem.second.tag = cur_tag++;
            elem.second.frags.push_back(frag);
            frag_map.insert(elem);
            constexpr int THIRTY_SEC = 30000;
            background_queue.call_in(THIRTY_SEC, &purge_frag_map_entry, frag->stream_id, elem.second.tag);
        } else {
            iter->second.frags.push_back(frag);
        }
        iter = frag_map.find(frag->stream_id);
        if(iter->second.tot_frames == iter->second.frags.size()) { // We got all the fragments
            vector<pair<uint8_t, shared_ptr<DataMsg>>> sort_frags;
            for(auto & frag : iter->second.frags) {
                sort_frags.emplace_back((*frag).kiss_cur_frame, frag); // DOES THIS EVEN WORK???
            }
            sort(sort_frags.begin(), sort_frags.end());
            vector<uint8_t> frag_comb;
            for(auto & sort_frag : sort_frags) {
                for(int i = 0; i < sort_frag.second->payload.size; i++) {
                    frag_comb.push_back(sort_frag.second->payload.bytes[i]); //NOLINT
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


static void send_to_uarts(const SerMsg &ser_msg);
static void send_to_uarts(const SerMsg &ser_msg) {
    if(ser_msg.has_data_msg() || ser_msg.has_voice_frame_msg()) { 
        kiss_sers_mtx.lock();
        for(auto & kiss_ser : kiss_sers) {
            auto ser_msg_sptr_en = make_shared<SerMsg>(ser_msg);
            kiss_ser->enqueue_msg(ser_msg_sptr_en);
        }
        kiss_sers_mtx.unlock();
    }
}


void rx_frame_ser_thread_fn() {
    for(;;) {
        auto rx_frame_sptr = dequeue_mail<std::shared_ptr<Frame>>(rx_frame_mail);
        // Handle KISS frames vs. "regular" QMesh frames vs. voice frames
        if(rx_frame_sptr->getDataMsgType() == DataMsg_Type_KISSRX || 
            rx_frame_sptr->getDataMsgType() == DataMsg_Type_KISSTX) {
            auto ser_msg_sptr = make_shared<SerMsg>();
            ser_msg_sptr->type(SerialMsg_Type_DATA);
            rx_frame_sptr->saveToPB(ser_msg_sptr->data_msg());
            ser_msg_sptr->data_msg().type = DataMsg_Type_KISSRX;
            auto frag = make_shared<DataMsg>(ser_msg_sptr->data_msg());
            auto pkt = handle_incoming_frag(frag);
            if(pkt != nullptr) {
                ser_msg_sptr->data_msg() = *pkt;
            }
            send_to_uarts(*ser_msg_sptr);
        } else if(rx_frame_sptr->getDataMsgType() == DataMsg_Type_VOICERX ||
                    rx_frame_sptr->getDataMsgType() == DataMsg_Type_VOICETX) {
            auto voice = make_shared<VoiceMsgProcessor>();
            vector<uint8_t> pld;
            rx_frame_sptr->getPayload(pld);
            vector<vector<uint8_t>> voice_frames = voice->getVoiceFrames(pld);
            for(auto & voice_frame : voice_frames) {
                auto voice_ser_msg_sptr = make_shared<SerMsg>();
                voice_ser_msg_sptr->type(SerialMsg_Type_VOICE_MSG);
                voice_ser_msg_sptr->voice_frame_msg().end_stream = false;
                voice_ser_msg_sptr->voice_frame_msg().payload.size = voice_frame.size();
                memcpy(voice_ser_msg_sptr->voice_frame_msg().payload.bytes, 
                        voice_frame.data(), voice_frame.size());
                send_to_uarts(*voice_ser_msg_sptr);
            }
        } else {
            auto ser_msg_sptr = make_shared<SerMsg>();
            ser_msg_sptr->type(SerialMsg_Type_DATA);
            rx_frame_sptr->saveToPB(ser_msg_sptr->data_msg());
            ser_msg_sptr->data_msg().type = DataMsg_Type_RX;
            send_to_uarts(*ser_msg_sptr);
        }
    }
}
