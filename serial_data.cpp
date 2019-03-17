#include <mbed.h>
#include <ATCmdParser.h>
#include "serial_data.hpp"


bool FrameQueue::enqueue(frame_t *frame) {
    if(queue.full()) 
        return false;
    frame_t *tmp_frame = queue.alloc();
    memcpy(tmp_frame, frame, sizeof(frame_t));
    queue.put(tmp_frame);
    return true;
}

// Dequeue a frame. Copies the dequeued data into the frame if successful,
//  returning true. Returns false if unsuccessful (queue is empty).
bool FrameQueue::dequeue(frame_t *frame) {
    if(queue.empty())
        return false;
    osEvent evt = queue.get();
    if(evt.status == osEventMail) {
        frame_t *tmp_frame = (frame_t *) evt.value.p;
        memcpy(frame, tmp_frame, sizeof(frame_t));
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
char response[64];
char recv_data[256];
void processATCmds(ATCmdParser *at) {
    at->recv("AT+%s", response);
    if(!strcmp(response, "Freq?")) {
        at->send(radio->freq);
    } 
    else if(!strcmp(response, "Freq")) {
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
}
