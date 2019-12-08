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

#ifndef SERIAL_DATA_HPP
#define SERIAL_DATA_HPP

#include "mbed.h"
#include "MbedJSONValue.h"
#include <string>
#include "params.hpp"
#include "nv_settings.hpp"
#include "json_serial.hpp"
#include "fec.hpp"

static uint8_t enc_buf[512], dec_buf[256];

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

typedef enum {
    PKT_OK = 0,
    PKT_FEC_FAIL,
    PKT_BAD_HDR_CRC,
    PKT_BAD_PLD_CRC,
    PKT_BAD_SIZE,
    PKT_UNITIALIZED,
} PKT_STATUS_ENUM;

#define BEACON_FRAME 0
#define PAYLOAD_FRAME 1
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
    typedef union {
        uint16_t crc;
        uint8_t crc_bytes[2];
    } crc16_t;
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
        crc16_t hdr_crc;
        crc16_t data_crc;
    } frame_pkt;
    // payload
    vector<uint8_t> data;
protected:
    frame_pkt pkt;
    // receive stats
    int16_t rssi;
    int8_t snr;
    uint16_t rx_size;
    PKT_STATUS_ENUM pkt_status;

public:
    shared_ptr<FEC> fec;

    /**
    * Get the combined, un-FEC'd size of the Frame.
    */
    static size_t size(void) {
        return radio_cb["Payload Length"].get<int>() + sizeof(pkt.hdr) + 
            sizeof(pkt.hdr_crc) + sizeof(pkt.data_crc);
    }

    /**
    * Default constructor. Constructs with a default FEC (one that does nothing).
    */
    Frame() : Frame(make_shared<FEC>()){ }

    /**
    * Constructor that loads a specific FEC.
    * @param my_fec shared_ptr to the FEC object.
    */
    Frame(shared_ptr<FEC> my_fec) {
        fec = my_fec;
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
        buf.resize(data.size());
        buf.assign(data.begin(), data.end());
        return data.size();
    }

    /**
    * Set the beacon string. Returns the size of the beacon string.
    * @param beacon_str The beacon string.
    */
    size_t setBeaconPayload(string &beacon_str) {
        data.resize(radio_cb["Payload Length"].get<int>());
        size_t len = beacon_str.size() < data.size() ? beacon_str.size() : data.size();
        memcpy(data.data(), beacon_str.c_str(), len);        
        pkt.hdr.type = BEACON_FRAME;
        pkt.hdr.stream_id = 0;
        pkt.hdr.ttl = 0;
        pkt.hdr.sender = 0;
        pkt.hdr.pre_offset = 0;
        pkt.hdr.nsym_offset = 0;
        pkt.hdr.sym_offset = 0;
        this->calculateHeaderCrc();
        this->calculatePayloadCrc();
        return data.size();
    }

    /**
    * FEC-encode the Frame, and provide the encoded bytes. Returns size in
    * bytes of the FEC-encoded Frame.
    * @param buf The vector that will hold the encoded Frame.
    */
    size_t serialize(vector<uint8_t> &buf);

    /**
    * Calculate the CRC of the header.
    */
    uint16_t calculateHeaderCrc(void);

    /**
    * Calculate the CRC of the payload.
    */
    uint16_t calculatePayloadCrc(void);

    /**
    * Calculate the CRC for the "unique" information.
    * The unique information consists of the type, stream_id, and payload.
    * This method is primarily used to provide a unique "hash" for Frames
    * in order to determine whether the QMesh node has seen them before.
    */
    uint32_t calculateUniqueCrc(void);

    /**
     * Check the header CRC, returning True if a match, False if not.
     */
    bool checkHeaderCrc(void) {
        return (pkt.hdr_crc.crc == calculateHeaderCrc());
    }

    /**
     * Check the payload CRC, returning True if a match, False if not.
     */
    bool checkPayloadCrc(void) {
        return (pkt.data_crc.crc == calculatePayloadCrc());
    }

    /**
     * Check the integrity of the packet by checking both the header and payload CRCs
     */
    bool checkIntegrity(void) {
        return (checkHeaderCrc() & checkPayloadCrc());
    }

    /**
     * Sets the header CRC by computing it based on the curent header data.
     * Also returns the computed CRC.
     */
    uint16_t setHeaderCrc(void) {
        uint16_t crc = calculateHeaderCrc();
        pkt.hdr_crc.crc = crc;
        return crc;
    }

    /**
     * Sets the payload CRC by computing it based on the curent payload data.
     * Also returns the computed CRC.
     */
    uint16_t setPayloadCrc(void) {
        uint16_t crc = calculatePayloadCrc();
        pkt.data_crc.crc = crc;
        return crc;
    }

    /**
    * Takes a vector of bytes and loads it into the Frame's internal
    * data structures. This is designed to take bytes received off the
    * air, so it performs FEC decoding, and will return a PKT_STATUS_ENUM
    * with the results (testing stuff out): \n
    *  - PKT_FEC_FAIL -- FEC decoding failed \n
    *  - PKT_BAD_SIZE -- Packet size is inconsistent with bytes decoded \n
    *  - PKT_BAD_HDR_CRC -- Bad packet header CRC \n
    *  - PKT_BAD_PLD_CRC -- Bad packet payload CRC \n
    *  - PKT_OK -- Packet decoded successfully \n
    *
    * @param buf shared_ptr to a vector of received, encoded bytes to decode
    */
    PKT_STATUS_ENUM deserialize(shared_ptr<vector<uint8_t>> buf); 

    /**
     * Increment the TTL, updating the header CRC in the process.
     */
    void incrementTTL(void) {
        pkt.hdr.ttl += 1;
        setHeaderCrc();
    }

    /**
     * Return the frame's current TTL value
     */
    uint8_t getTTL(void) {
        return pkt.hdr.ttl;
    }

    /** 
     * Returns the the size of a packet
     */
    static size_t getPktSize(void) {
        return sizeof(frame_pkt);
    }

    /**
     * Returns the size of a packet header
     */
    static size_t getHdrSize(void) {
        return sizeof(frame_hdr);
    }

    /**
    * Get the size, in bytes, of a Frame after FEC encoding.
    */
    size_t getFullPktSize(void);

    /** 
     * Get the offsets from the packet header
     * @param pre_offset Number of preamble-length offsets
     * @param nsym_offset Number of symbol-length offsets
     * @param sym_offset Intra-symbol offset.
     */
    void getOffsets(uint8_t *pre_offset, uint8_t *nsym_offset, uint8_t *sym_offset) {
        *pre_offset = pkt.hdr.pre_offset;
        *nsym_offset = pkt.hdr.nsym_offset;
        *sym_offset = pkt.hdr.sym_offset;
    }

    /** 
     * Get the offsets in the packet header
     * @param pre_offset Number of preamble-length offsets
     * @param nsym_offset Number of symbol-length offsets
     * @param sym_offset Intra-symbol offset.
     */
    void setOffsets(const uint8_t *pre_offset, const uint8_t *nsym_offset, const uint8_t *sym_offset) {
        pkt.hdr.pre_offset = *pre_offset;
        pkt.hdr.nsym_offset = *nsym_offset;
        pkt.hdr.sym_offset = *sym_offset;
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
    uint16_t getPayloadCrc(void) {
        return this->pkt.data_crc.crc;
    }

    /**
     * Returns the header CRC stored within the Frame object.
     */
    uint16_t getHeaderCrc(void) {
        return this->pkt.hdr_crc.crc;
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
    void loadFromJSON(MbedJSONValue &json);

    /**
    * Load the JSON object with the Frame's fields.
    * @param json MbedJSONValue holding Frame's parameters.
    */
    void saveToJSON(MbedJSONValue &json);
};


extern Mail<shared_ptr<Frame>, 16> tx_frame_mail, rx_frame_mail, nv_logger_mail;

/** 
 * Enqueues a value onto an Mbed OS mailbox.
 * @param mail_queue The mailbox.
 * @param T The type of the value to be enqueued.
 * @param val The value to be enqueued.
 */
template <class T> 
void enqueue_mail(Mail<T, 16> &mail_queue, T val) {
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
void enqueue_mail_nonblocking(Mail<T, 16> &mail_queue, T val) {
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
T dequeue_mail(Mail<T, 16> &mail_queue) {
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

#endif /* SERIAL_DATA_HPP */