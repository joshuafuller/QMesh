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

I2C i2c(PB_9, PB_8);
EEPROM eeprom(&i2c);
NVSettings *nv_settings;
FEC *fec;  
Serial pc(USBTX, USBRX);
JSONSerial rx_json_ser, tx_json_ser;
Thread tx_serial_thread, rx_serial_thread;


#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

// main() runs in its own thread in the OS
int main()
{
    // Set the UART comms speed
    pc.baud(921600);

    // Start the serial handler threads
    tx_serial_thread.start(tx_serial_thread_fn);
    rx_serial_thread.start(rx_serial_thread_fn);

    // Start a thread for blinking LEDs
    led1.LEDBlink();
    led2.LEDOff();
    led3.LEDOff();

    wait(3);

    if(button.getPressed() == true) {
        led2.LEDBlink();
        led3.LEDBlink();
        ATSettings *at = new ATSettings(&pc, nv_settings);
        for(;;) { wait(1); }
    }
    led1.LEDSolid();

    // Set up and test the EEPROM
#ifdef TEST_EEPROM
    debug_printf(DBG_INFO, "Now testing the EEPROM\r\n");
    eeprom.testEEPROM();
#endif

    nv_settings = new NVSettings(&eeprom);

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

#ifdef TX_TEST_MODE
    // Start a thread for the radio
    debug_printf(DBG_INFO, "Starting Tx test thread\r\n");
    radio_thread.start(tx_test_radio);
#elif defined RX_TEST_MODE
    // Start a thread for the radio_thread
    debug_printf(DBG_INFO, "Starting Rx test thread\r\n");
    radio_thread.start(rx_test_radio);
#elif defined MESH_TEST_MODE_ORIGINATOR
    // Start a thread for the radio_thread
    debug_printf(DBG_INFO, "Starting Mesh test originator thread\r\n");
    radio_thread.start(mesh_originator_test);
#else
    debug_printf(DBG_INFO, "Starting the mesh network\r\n");
#endif

    int count = 0;
    while (true) {
        wait_ms(SLEEP_TIME);
    }
}

