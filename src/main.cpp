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

#include "mbed.h"
#include "peripherals.hpp"
#include "params.hpp"
#include "serial_data.hpp"
#include "fec.hpp"
#include "json_serial.hpp"
#include "mesh_protocol.hpp"

FEC *fec;  
Serial pc(USBTX, USBRX);
JSONSerial rx_json_ser, tx_json_ser;
Thread tx_serial_thread(4096), rx_serial_thread(4096);
Thread mesh_protocol_thread(4096);
Thread beacon_thread(4096);
Thread nv_log_thread(4096);

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

// main() runs in its own thread in the OS
int main()
{
    // Set the RTC to zero. We just use it to track the age of 
    //  the packet tracker.
    set_time(0);

    // Start the WDT thread
//    wdt_thread.start(wdt_fn);

    // Set the UART comms speed
    pc.baud(115200);

    debug_printf(DBG_INFO, "hello\r\n");

    // Start the serial handler threads
    tx_serial_thread.start(tx_serial_thread_fn);
    rx_serial_thread.start(rx_serial_thread_fn);

    debug_printf(DBG_INFO, "serial threads started\r\n");

    // Mount the filesystem, load the configuration
    init_filesystem();
    load_settings_from_flash();
    saveSettingsToFlash();

    // Start a thread for blinking LEDs
    led1.LEDBlink();
    led2.LEDOff();
    led3.LEDOff();

while(1);

    // Set up and test the EEPROM
#ifdef TEST_EEPROM
    debug_printf(DBG_INFO, "Now testing the EEPROM\r\n");
    eeprom.testEEPROM();
#endif

    // Test the FEC
    debug_printf(DBG_INFO, "Now testing the FEC\r\n");
    Frame *fec_frame = new Frame();  
    debug_printf(DBG_INFO, "Size of fec_frame is %d\r\n", fec_frame->getPktSize());
#ifdef FEC_CONV
    fec = new FECConv(fec_frame->getPktSize(), 2, 9);
#elif defined FEC_RSV
    fec = new FECRSV(fec_frame->getPktSize(), 2, 9, 8);
#else
#error "Need to define either FEC_CONV or FEC_RSV\r\n"
#endif
    fec->benchmark(100);
    delete fec_frame;
    uint8_t *fec_enc_buf = new uint8_t[fec_frame->getPktSize()];
    uint8_t *fec_dec_buf = new uint8_t[fec->getEncSize(fec_frame->getPktSize())];
    static char test_msg[] = "0123456789\r\n";
    size_t enc_size = fec->getEncSize(13);
    debug_printf(DBG_INFO, "Encoded size is %d\r\n", enc_size);
    debug_printf(DBG_INFO, "Encoding %s", test_msg);
    fec->encode((uint8_t *) test_msg, 13, fec_enc_buf);
    fec->decode(fec_enc_buf, enc_size, fec_dec_buf);
    debug_printf(DBG_INFO, "Decoded %s", fec_dec_buf);

    // Set up the radio
    debug_printf(DBG_INFO, "Initializing radio\r\n");
    init_radio();

    // Start the NVRAM logging thread
    nv_log_thread.start(nv_log_fn);

    // Start the mesh protocol thread
    mesh_protocol_thread.start(mesh_protocol_fsm);

    // Start the beacon thread
    beacon_thread.start(beacon_fn);

    int count = 0;
    while (true) {
        wait_ms(SLEEP_TIME);
    }
}

