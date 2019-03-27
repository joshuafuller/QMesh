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

#ifndef SERIAL_DATA_HPP
#define SERIAL_DATA_HPP

#include "mbed.h"
#include "params.hpp"
#include "nv_settings.hpp"
#include "SX1272_LoRaRadio.h"

extern SX1272_LoRaRadio radio;

typedef struct frame_struct_t {
    uint8_t type;    
    uint16_t stream_id;
    uint8_t ttl;
    int16_t rssi;
    int8_t snr;
    uint16_t rx_size;
    uint8_t data[FRAME_PAYLOAD_LEN];
} frame_struct_t;

typedef union frame_t {
    frame_struct_t frame;
    uint8_t buf[sizeof(frame_struct_t)];
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


#endif /* SERIAL_DATA_HPP */