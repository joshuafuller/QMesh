#include "mbed.h"

typedef struct {
    uint8_t type;    
    uint16_t stream_id;
    uint8_t ttl;
    uint8_t data[FRAME_PAYLOAD_LEN];
} frame_t;


class FrameQueue {
    protected:
        Mail<frame_t, 32> queue;
    public:
        // Enqueue a frame. Returns true if enqueue was successful,
        //  false if unsuccessful due to overflow
        bool enqueue(frame_t *frame);

        // Dequeue a frame. Copies the dequeued data into the frame if successful,
        //  returning true. Returns false if unsuccessful (queue is empty).
        bool dequeue(frame_t *frame);

        // Returns whether queue is empty
        bool getEmpty(void);

        // Returns whether queue is full
        bool getFull(void);
};