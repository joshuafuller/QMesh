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

#include "mbed.h"
#include "peripherals.hpp"
#include "params.hpp"
#include "serial_data.hpp"
#include "fec.hpp"
#include "json_serial.hpp"
#include "mesh_protocol.hpp"

shared_ptr<FEC> fec;  
Serial pc(USBTX, USBRX);
JSONSerial rx_json_ser, tx_json_ser;
Thread tx_serial_thread(4096), rx_serial_thread(4096);
Thread mesh_protocol_thread(4096);
Thread beacon_thread(4096);
Thread nv_log_thread(4096);

system_state_t current_mode = BOOTING;
bool stay_in_management = false;

DigitalOut flash_pwr_ctl(MBED_CONF_APP_FLASH_PWR);
DigitalOut radio_pwr_ctl(MBED_CONF_APP_RADIO_PWR);

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

void print_memory_info() {
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) malloc(cnt * sizeof(mbed_stats_stack_t));
 
    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", (unsigned long) stats[i].thread_id, 
            (unsigned long) stats[i].max_size, (unsigned long) stats[i].reserved_size);
    }
    free(stats);
 
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes\r\n", (unsigned long) heap_stats.current_size, 
        (unsigned long) heap_stats.reserved_size);
}


// main() runs in its own thread in the OS
int main()
{
    led1.LEDSolid();

    // Set the RTC to zero. We just use it to track the age of 
    //  the packet tracker.
    set_time(0);

    // Start the WDT thread
//    wdt_thread.start(wdt_fn);

    // Set the UART comms speed
    pc.baud(921600);

    debug_printf(DBG_INFO, "Starting serial threads...\r\n"); // Removing this causes a hard fault???

    // Start the serial handler threads
    tx_serial_thread.start(tx_serial_thread_fn);
    rx_serial_thread.start(rx_serial_thread_fn);

    debug_printf(DBG_INFO, "serial threads started\r\n");

    // Power cycle the SPI flash chip and the RF module
    debug_printf(DBG_INFO, "Powering down the SPI flash and LoRa modules...\r\n");
    flash_pwr_ctl = 0;
    radio_pwr_ctl = 0;
    wait(0.25);
    debug_printf(DBG_INFO, "Powering up the SPI flash...\r\n");
    flash_pwr_ctl = 1;
    wait(0.25);
    debug_printf(DBG_INFO, "Powering up the LoRa module...\r\n");
    radio_pwr_ctl = 1;
    wait(0.25);
    debug_printf(DBG_INFO, "Both modules now powered up!\r\n");

    // Mount the filesystem, load the configuration
    init_filesystem();
    load_settings_from_flash();
    save_settings_to_flash();

    // Wait for 5 seconds in MANAGEMENT mode
    current_mode = MANAGEMENT;
    led1.LEDFastBlink();
    wait(5);
    while(stay_in_management) {
        wait(5);
    }
    current_mode = RUNNING;

    led1.LEDBlink();
    led2.LEDOff();
    led3.LEDOff();

    // Test the FEC
    debug_printf(DBG_INFO, "Now testing the FEC\r\n");
    auto fec_frame = make_shared<Frame>();  
    debug_printf(DBG_INFO, "Size of fec_frame is %d\r\n", fec_frame->getPktSize());

    fec = make_shared<FECConv>(2, 9);
    fec->benchmark(100);
    fec = make_shared<FECRSV>(2, 9, 8);
    fec->benchmark(100);
    fec = make_shared<FECPolar>(2*Frame::size(), Frame::size(), 32);
    fec->benchmark(100);
    
    print_memory_info();

    //uint8_t *fec_enc_buf = new uint8_t[fec_frame->getPktSize()];
    vector<uint8_t> fec_enc_buf;
    //uint8_t *fec_dec_buf = new uint8_t[fec->getEncSize(fec_frame->getPktSize())];
    vector<uint8_t> fec_dec_buf;
    string test_msg = "0123456789\r\n";
    vector<uint8_t> test_msg_vec(test_msg.begin(), test_msg.end());
    size_t enc_size = fec->getEncSize(13);
    debug_printf(DBG_INFO, "Encoded size is %d\r\n", enc_size);
    debug_printf(DBG_INFO, "Encoding %s", test_msg.c_str());
    fec->encode(test_msg_vec, fec_enc_buf);
    fec->decode(fec_enc_buf, fec_dec_buf);
    string fec_dec_str(fec_dec_buf.begin(), fec_dec_buf.end());
    debug_printf(DBG_INFO, "Decoded %s", fec_dec_str.c_str());

    // Set up the radio
    debug_printf(DBG_INFO, "Initializing radio\r\n");
    init_radio();

    // Start the NVRAM logging thread
    debug_printf(DBG_INFO, "Starting the NV logger\r\n");
    nv_log_thread.start(nv_log_fn);

    // Start the mesh protocol thread
    debug_printf(DBG_INFO, "Starting the mesh protocol thread\r\n");
    mesh_protocol_thread.start(mesh_protocol_fsm);

    // Start the beacon thread
    debug_printf(DBG_INFO, "Starting the beacon thread\r\n");
    beacon_thread.start(beacon_fn);

    debug_printf(DBG_INFO, "Started all threads\r\n");

    print_memory_info();

    int count = 0;
    while (true) {
        wait_ms(SLEEP_TIME);
    }
}

