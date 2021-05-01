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

#ifndef SERIAL_DATA_HPP
#define SERIAL_DATA_HPP

#include "mbed.h"
#include <string>
#include "params.hpp"
#include "nv_settings.hpp"
#include "kiss_serial.hpp"
#include "fec.hpp"
#include "qmesh.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"


extern SysCfgMsg radio_cb;


// Special debug printf. Prepends "[-] " to facilitate using the same
//  UART for both AT commands as well as debug commands.
enum DBG_TYPES {
    DBG_INFO,
    DBG_WARN,
    DBG_ERR,
};

/**
 * Pretty-print a message to the debug output.
 * @param dbg_type Debug type.
 * @fmt The printf()-formatted string to be printed..
 */
int debug_printf(const enum DBG_TYPES, const char *fmt, ...);

/**
 * Pretty-print a message to the debug output.
 * @param dbg_type Debug type.
 * @fmt The printf()-formatted string to be printed..
 */
int debug_printf_clean(const enum DBG_TYPES, const char *fmt, ...);

/**
 * Outputs received frames over the UART.
 */
void rx_frame_ser_thread_fn(void);

typedef enum {
    PKT_OK = 0,
    PKT_FEC_FAIL,
    PKT_BAD_CRC,
    PKT_BAD_SIZE,
    PKT_UNITIALIZED,
} PKT_STATUS_ENUM;

#define BEACON_FRAME 0
#define PAYLOAD_FRAME 1
#define KISS_FRAME 2
/**
 * This class implements the QMesh Frame. It provides both the storage
 * of Frame data fields as well as various methods for performing
 * various functionality needed for QMesh, such as:
 * - Checksum (CRC) calculations
 * - Time-to-Live (TTL) handling
 * - FEC encoding/decoding
 * - Serialization/deserialization
 */
class Frame {
public:
    typedef uint8_t crc8_t;
    typedef struct {
        union {
            struct __attribute__((__packed__)) {
                uint32_t ttl : 3;
                uint32_t sender : 6;    
                uint32_t sym_offset : 3;
            } fields;
            uint8_t b[2]; 
        } var_subhdr;
        union {
            struct __attribute__((__packed__)) {
                uint32_t type : 2;
                uint32_t stream_id : 14;
            } fields;
            uint8_t b;
        } cons_subhdr;
    } frame_hdr;
    typedef union {
        struct __attribute__((__packed__)) {
            uint32_t size : 8;
            uint32_t cur_frame : 4;
            uint32_t tot_frames: 4;
        } fields;
        uint8_t b[2];
    } kiss_subhdr;
    shared_ptr<FEC> fec;
	bool tx_frame;
private:
    frame_hdr hdr;
    vector<uint8_t> data;
    crc8_t crc;
protected:
    // receive stats
    int16_t rssi;
    int8_t snr;
    uint16_t rx_size;
    PKT_STATUS_ENUM pkt_status;

public:
    /**
    * Get the combined, un-FEC'd size of the Frame.
    */
    static size_t size(void);

    DataMsg_Type getDataMsgType(void) {
        switch(hdr.cons_subhdr.fields.type) {
            case 0: return DataMsg_Type_TX; 
            case 1: return DataMsg_Type_RX;
            case 2: return DataMsg_Type_KISSTX;
            case 3: return DataMsg_Type_KISSRX;
            default: MBED_ASSERT(false);
        }
    }

    void setDataMsgType(const DataMsg_Type datamsg_type) {
        if(datamsg_type == DataMsg_Type_TX) {
            hdr.cons_subhdr.fields.type = 0;
        } else if(datamsg_type == DataMsg_Type_RX) {
            hdr.cons_subhdr.fields.type = 1;
        } else {
            MBED_ASSERT(false);
        }
    }

    /**
    * Default constructor. Constructs with a default FEC (one that does nothing).
    */
    Frame() : Frame(make_shared<FEC>(Frame::size())) { }

    /**
    * Constructor that loads a specific FEC.
    * @param my_fec shared_ptr to the FEC object.
    */
    Frame(shared_ptr<FEC> my_fec) {
        fec = my_fec;
    }

    void serialize(vector<uint8_t> &ser_frame);

    void serialize_pb(vector<uint8_t> &data_msg);

    void deserialize_pb(const vector<uint8_t> &buf);

    void createFromKISS(DataMsg &data_msg);

    static size_t getKISSMaxSize(void);

    /// Equality operator for Frames
    bool operator == (const Frame &L) {
        auto L_cpy = make_shared<Frame>(L);
        auto R_cpy = make_shared<Frame>(*this);
        //debug_printf(DBG_INFO, "Setting up the comparison\r\n");
        //ThisThread::sleep_for(1000); 
        vector<uint8_t> l_ser_data, r_ser_data;
        L_cpy->serialize(l_ser_data);
        R_cpy->serialize(r_ser_data);
        //debug_printf(DBG_INFO, "Getting serialized data\r\n");
        //ThisThread::sleep_for(1000); 
        return (l_ser_data == r_ser_data);
    }

    /// Inequality operator for Frames
    bool operator != (const Frame &L) {
        return !(*this == L);
    }

    /**
    * Create a frame with basic test values and load it into the Frame's data structure.
    * @param buf The payload bytes.
    */
    void loadTestFrame(vector<uint8_t> &buf);

    /**
    * Get the payload. Returns the number of bytes in the payload.
    * @param buf Where the payload bytes are put.
    */
    size_t getPayload(vector<uint8_t> &buf) {
        buf = data;
        return buf.size();
    }

    /**
    * Set the beacon string. Returns the size of the beacon string.
    * @param beacon_str The beacon string.
    */
    size_t setBeaconPayload(string &beacon_str) {
        data.resize(radio_cb.net_cfg.pld_len);
        size_t len = beacon_str.size() < data.size() ? beacon_str.size() : data.size();
        memcpy(data.data(), beacon_str.c_str(), len);        
        hdr.cons_subhdr.fields.type = BEACON_FRAME;
        hdr.cons_subhdr.fields.stream_id = 0;
        hdr.var_subhdr.fields.ttl = 0;
        hdr.var_subhdr.fields.sender = 0;
        hdr.var_subhdr.fields.sym_offset = 0;
        this->setCRC();
        return data.size();
    }

    void whiten(const vector<uint8_t> &buf, vector<uint8_t> &wht_buf, const uint16_t seed);

    /**
    * FEC-encode the Frame, and provide the encoded bytes. Returns size in
    * bytes of the FEC-encoded Frame.
    * @param buf The vector that will hold the encoded Frame.
    */
    size_t serializeCoded(vector<uint8_t> &buf);

    /**
    * Calculate the CRC of the payload.
    */
    uint8_t calcCRC(void);

    /**
    * Calculate the CRC for the "unique" information.
    * The unique information consists of the type, stream_id, and payload.
    * This method is primarily used to provide a unique "hash" for Frames
    * in order to determine whether the QMesh node has seen them before.
    */
    uint32_t calcUniqueCRC(void);

    /**
     * Check the payload CRC, returning True if a match, False if not.
     */
    bool checkCRC(void) {
        return (crc == calcCRC());
    }

    /**
     * Sets the payload CRC by computing it based on the curent payload data.
     * Also returns the computed CRC.
     */
    uint8_t setCRC(void) {
        uint8_t calc_crc = calcCRC();
        crc = calc_crc;
        return calc_crc;
    }

    /**
     * Sets the sender's address based on the value passed in.
     */
    void setSender(uint8_t sender_addr) {
        hdr.var_subhdr.fields.sender = sender_addr;
        setCRC();
    }

    /**
     * Gets the sender's address.
     */
    uint32_t getSender(void) {
        return hdr.var_subhdr.fields.sender;
    }

    /**
     * Sets the Stream ID
     */
    void setStreamID(uint8_t id) {
        hdr.cons_subhdr.fields.stream_id = id;
        setCRC();
    }

    /**
     * Returns the Stream ID
     */
    uint32_t getStreamID(void) {
        return hdr.cons_subhdr.fields.stream_id;
    }


    /**
    * Takes a vector of bytes and loads it into the Frame's internal
    * data structures. This is designed to take bytes received off the
    * air, so it performs FEC decoding, and will return a PKT_STATUS_ENUM
    * with the results (testing stuff out): \n
    *  - PKT_FEC_FAIL -- FEC decoding failed \n
    *  - PKT_BAD_SIZE -- Packet size is inconsistent with bytes decoded \n
    *  - PKT_BAD_CRC -- Bad packet payload CRC \n
    *  - PKT_OK -- Packet decoded successfully \n
    *
    * @param buf shared_ptr to a vector of received, encoded bytes to decode
    */
    PKT_STATUS_ENUM deserializeCoded(const shared_ptr<vector<uint8_t>> buf);

    /**
     * Increment the TTL, updating the header CRC in the process.
     */
    void incrementTTL(void) {
        hdr.var_subhdr.fields.ttl += 1;
        setCRC();
    }

    /**
     * Return the frame's current TTL value
     */
    uint8_t getTTL(void) {
        return hdr.var_subhdr.fields.ttl;
    }

    /**
     * Returns the size of a packet header
     */
    static size_t hdrSize(void) {
        return sizeof(hdr);
    }

    /**
    * Get the size, in bytes, of a Frame after FEC encoding.
    */
    size_t codedSize(void);

    /** 
     * Get the offsets from the packet header
     * @param pre_offset Number of preamble-length offsets
     * @param nsym_offset Number of symbol-length offsets
     * @param sym_offset Intra-symbol offset.
     */
    void getOffsets(uint8_t &pre_offset, uint8_t &nsym_offset, uint8_t &sym_offset) {
        pre_offset = 0;
        nsym_offset = 0;
        sym_offset = hdr.var_subhdr.fields.sym_offset;
    }

    /** 
     * Get the offsets in the packet header
     * @param pre_offset Number of preamble-length offsets
     * @param nsym_offset Number of symbol-length offsets
     * @param sym_offset Intra-symbol offset.
     */
    void setOffsets(const uint8_t pre_offset, const uint8_t nsym_offset, const uint8_t sym_offset) {
        hdr.var_subhdr.fields.sym_offset = sym_offset;
        setCRC();
    }

    /**
     * Get the receive statistics
     * @param rssi Full-packet RSSI
     * @param snr Full-packet SNR
     * @param rx_size Number of bytes received
     */
    void getRxStats(int16_t &rssi, int8_t &snr, uint16_t &rx_size) {
        rssi = this->rssi;
        snr = this->snr;
        rx_size = this->rx_size;
    }

    /**
     * Set the receive statistics
     * @param rssi Full-packet RSSI
     * @param snr Full-packet SNR
     * @param rx_size Number of bytes received
     */
    void setRxStats(const int16_t rssi, const int8_t snr, const uint16_t rx_size) {
        this->rssi = rssi;
        this->snr = snr;
        this->rx_size = rx_size;
    }

    /**
     * Returns the payload CRC stored within the Frame object.
     */
    uint8_t getCRC(void) {
        return crc;
    }

    /**
    * Pretty-print the Frame's fields to the debug output.
    * @param dbg_type Debug type to use with the calls to debug_printf().
    */
    void prettyPrint(const enum DBG_TYPES dbg_type);

    /**
    * Load the frame's fields from a parsed JSON object
    * @param json MbedJSONValue holding Frame's parameters.
    */
    void loadFromPB(const DataMsg &data_msg);

    /**
    * Load the JSON object with the Frame's fields.
    * @param json MbedJSONValue holding Frame's parameters.
    */
    void saveToPB(DataMsg &data_msg);
};


typedef enum {
    TX_FRAME_EVT,
    TX_DONE_EVT,
    TX_POCSAG_DONE_EVT,
    RX_DONE_EVT,
    TX_TIMEOUT_EVT,
    RX_TIMEOUT_EVT,
    RX_ERROR_EVT,
    TX_POCSAG_EVT,
} radio_evt_enum_t;

class RadioEvent {
public:
    radio_evt_enum_t evt_enum;
    shared_ptr<CalTimer> tmr_sptr;
    int16_t rssi;
    int8_t snr;
    shared_ptr<vector<uint8_t>> buf;
    shared_ptr<list<pair<uint32_t, uint8_t> > > rssi_list_sptr;
    shared_ptr<Frame> frame;
    string pocsag_msg;

    RadioEvent(const radio_evt_enum_t my_evt_enum);

    RadioEvent(const radio_evt_enum_t my_evt_enum, string &pocsag_msg);

    RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr);

    RadioEvent(const radio_evt_enum_t my_evt_enum, shared_ptr<CalTimer> my_tmr, const uint8_t *my_buf, 
                shared_ptr<list<pair<uint32_t, uint8_t> > > my_rssi_list_sptr,
                const size_t my_size, const int16_t my_rssi, const int8_t my_snr);

    RadioEvent(const radio_evt_enum_t my_evt_enum, const shared_ptr<Frame> &frame);
};

extern Mail<shared_ptr<RadioEvent>, QUEUE_DEPTH> unified_radio_evt_mail, tx_radio_evt_mail;


extern Mail<shared_ptr<Frame>, QUEUE_DEPTH> tx_frame_mail, rx_frame_mail, nv_logger_mail;

/** 
 * Enqueues a value onto an Mbed OS mailbox.
 * @param mail_queue The mailbox.
 * @param T The type of the value to be enqueued.
 * @param val The value to be enqueued.
 */
template <class T> 
void enqueue_mail(Mail<T, QUEUE_DEPTH> &mail_queue, T val) {
    auto mail_item = mail_queue.alloc_for(osWaitForever);
    MBED_ASSERT(mail_item != NULL);
    *mail_item = val;
    while(mail_queue.full());
    mail_queue.put(mail_item);
}


/** 
 * Enqueues a value onto an Mbed OS mailbox. Quietly drops the 
 * request if the queue is full.
 * @param mail_queue The mailbox.
 * @param T The type of the value to be enqueued.
 * @param val The value to be enqueued.
 */
template <class T> 
void enqueue_mail_nonblocking(Mail<T, QUEUE_DEPTH> &mail_queue, T val) {
    auto mail_item = mail_queue.alloc();
    if(mail_item == NULL) {
        return;
    }
    *mail_item = val;
    mail_queue.put(mail_item);
}

/** 
 * Dequeues a value from an Mbed OS mailbox. Returns the dequeued value.
 * @param mail_queue The mailbox.
 * @param T The type of the value to be dequeued.
 */
template <class T>
T dequeue_mail(Mail<T, QUEUE_DEPTH> &mail_queue) {
    T mail_item; 
    for(;;) {
        osEvent evt = mail_queue.get();
        if(evt.status == osEventMail) {
            mail_item = *((T *) evt.value.p);
            mail_queue.free((T *) evt.value.p);
            break;
        }
        else { MBED_ASSERT(false); }
    } 
    return mail_item;
}

template <class T>
T dequeue_mail_timeout(Mail<T, QUEUE_DEPTH> &mail_queue, const uint32_t timeout_ms, bool &timed_out) {
    T mail_item; 
    timed_out = false;
    osEvent evt = mail_queue.get(timeout_ms);
    if(evt.status == osEventMail) {
        mail_item = *((T *) evt.value.p);
        mail_queue.free((T *) evt.value.p);
    }
    else if(evt.status == osEventTimeout) {
        timed_out = true;
    }
    else { MBED_ASSERT(false); }
    return mail_item;
}

#endif /* SERIAL_DATA_HPP */