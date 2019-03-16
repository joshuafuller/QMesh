#include "mbed.h"
#include "serial_data.hpp"

extern FrameQueue tx_queue, rx_queue;

static frame_t mesh_data;
static Timeout rx_mesh_time;
typedef enum {
    TX,
    RX,
} radio_state_t;
radio_state_t radio_state; 

#define TX_TIMEOUT 10
static int tx_timeout = 0;
void txCallback(void) {
    if(radio_state == TX) {
        // If there's something to send in the TX queue, then send it
        tx_timeout = 0;
        // Otherwise, wait until the next timeslot to try again
        tx_timeout += 1;
        if(tx_timeout > 10) {
            tx_timeout = 0;
            radio_state = RX;
        }
        else {
            
        }
    }
    else if(radio_state == RX) {
        // send(mesh_data);
    }
}


void rxCallback(frame_t *rx_frame) {
    rx_mesh_time.attach(txCallback, TIME_SLOT_SECONDS);
    rx_queue.enqueue(rx_frame);
    memcpy(&mesh_data, rx_frame, sizeof(frame_t));
}