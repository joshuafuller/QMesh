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

#include "mbed.h"
#include "peripherals.hpp"
#include "params.hpp"
#include "serial_data.hpp"
#include "fec.hpp"
#include "kiss_serial.hpp"
#include "mesh_protocol.hpp"
#include "mem_trace.hpp"
#include "Adafruit_SSD1306.h"
#include "SoftI2C.h"
#include "USBSerial.h"
#include "ble_serial.hpp"



constexpr int THREAD_STACK_SIZE = 4096;
Thread mesh_protocol_thread(osPriorityRealtime, THREAD_STACK_SIZE, nullptr, "MESH-FSM"); /// Handles the mesh protocol
Thread rx_frame_thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "RX-FRAME"); /// Processes and routes received Frames
Thread nv_log_thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "NV-LOG"); /// Logging to the QSPI flash

EventQueue background_queue;
Thread background_thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "BG"); /// Background thread

time_t boot_timestamp;

#ifdef APRS
Afsk my_afsk;
#endif

system_state_t current_mode = system_state_t::BOOTING;
atomic<bool> stay_in_management(false);

static constexpr int TENTH_SECOND = 100; //NOLINT
static constexpr int QUARTER_SECOND = 250;
static constexpr int HALF_SECOND = 500;
static constexpr int ONE_SECOND = 1000;
static constexpr int TWO_SECONDS = 2000;
static constexpr int FIVE_SECONDS = 5000;
static constexpr int I2C_FREQ = 400000;


void send_status();
#ifdef USER_BUTTON
DigitalIn user_button(USER_BUTTON);
#else // for the nRF52 board
DigitalIn user_button(BUTTON1);
#endif
//MBED_CONF_APP_QSPI_FLASH_IO0
SoftI2C oled_i2c(MBED_CONF_APP_OLED_SDA, MBED_CONF_APP_OLED_SCL);  // SDA, SCL
shared_ptr<Adafruit_SSD1306_I2c> oled;

void print_stats()
{
    {
    mbed_stats_cpu_t stats;
    mbed_stats_cpu_get(&stats);

    printf("Uptime: %-20lld", stats.uptime);
    printf("Idle time: %-20lld", stats.idle_time);
    printf("Sleep time: %-20lld", stats.sleep_time);
    printf("Deep sleep time: %-2F0lld\n", static_cast<double>(stats.deep_sleep_time));
    }
#if 0
    {
    mbed_stats_thread_t *stats = new mbed_stats_thread_t[20];
    int count = mbed_stats_thread_get_each(stats, 20);
    
    for(int i = 0; i < count; i++) {
        printf("ID: 0x%x \n", stats[i].id);
        printf("Name: %s \n", stats[i].name);
        printf("State: %d \n", stats[i].state);
        printf("Priority: %d \n", stats[i].priority);
        printf("Stack Size: %d \n", stats[i].stack_size);
        printf("Stack Space: %d \n", stats[i].stack_space);
        printf("\n");
    }
    }
#endif
}

#if MBED_CONF_APP_HAS_WATCHDOG == 1
constexpr uint32_t WDT_TIMEOUT_MS = 6000;
static void wdt_pet() { // pet the watchdog
    Watchdog::get_instance().kick();
    background_queue.call_in(WDT_TIMEOUT_MS/2, wdt_pet);
}
#endif


// main() runs in its own thread in the OS
auto main() -> int
{
    start_cal();
    time(&boot_timestamp);
#if MBED_CONF_APP_HAS_WATCHDOG == 1
    Watchdog &wdt = Watchdog::get_instance();
    wdt.start();
    wdt_pet();
#endif

    // Set up the LEDs
    ThisThread::sleep_for(HALF_SECOND);
    background_thread.start(callback(&background_queue, &EventQueue::dispatch_forever));
    led1.setEvtQueue(&background_queue);
    led2.setEvtQueue(&background_queue);
    led3.setEvtQueue(&background_queue);  

    oled_i2c.frequency(I2C_FREQ);
    oled_i2c.start();
    
    constexpr int OLED_NUM_LINES = 32;
    constexpr int OLED_NUM_COLS = 128;
    constexpr int OLED_CONST = 0x78;
    oled = make_shared<Adafruit_SSD1306_I2c>(oled_i2c, MBED_CONF_APP_FHSS_MON, OLED_CONST, OLED_NUM_LINES, OLED_NUM_COLS);

    oled->printf("Welcome to QMesh\r\n");
    oled->display();

    led1.LEDBlink();
    oled->printf("In rescue mode...\r\n");
    oled->display();
	ThisThread::sleep_for(ONE_SECOND);
	if(user_button != 0) {
		led1.LEDFastBlink();
		ThisThread::sleep_for(ONE_SECOND);
		if(user_button != 0) {
			rescue_filesystem();
		}
	}
	led1.LEDSolid();
#if MBED_CONF_APP_HAS_BLE == 1
    auto *push_button = new PushButton(BUTTON1);
#else
    auto *push_button = new PushButton(USER_BUTTON);
#endif
    push_button->SetQueue(background_queue);
    ThisThread::sleep_for(ONE_SECOND);

    // Mount the filesystem, load the configuration, log the bootup
    init_filesystem();
    load_settings_from_flash();
    log_boot();

    auto *disp_file = fopen("/fs/display.off", "r");
    if(disp_file == nullptr) {
        oled->displayOn();
        fclose(disp_file);
    } else {
        oled->displayOff();
    }

    Frame::seed_stream_id(radio_cb.address);
    // Start the serial handler threads
#ifdef MBED_CONF_APP_KISS_UART_TX
    ThisThread::sleep_for(HALF_SECOND);
    auto bt_ser = make_shared<KISSSerialUART>(MBED_CONF_APP_KISS_UART_TX, 
                                            MBED_CONF_APP_KISS_UART_RX, 
                                            string("BT"), DEBUG_PORT);
    MBED_ASSERT(bt_ser);
#endif /* MBED_CONF_APP_KISS_UART_TX */
#ifdef MBED_CONF_APP_KISS_UART_TX_ALT
    ThisThread::sleep_for(HALF_SECOND);
    auto bt_alt_ser = make_shared<KISSSerialUART>(MBED_CONF_APP_KISS_UART_TX_ALT, 
                                                MBED_CONF_APP_KISS_UART_RX_ALT,
                                                MBED_CONF_APP_KISS_UART_EN_ALT,
                                                MBED_CONF_APP_KISS_UART_ST_ALT, 
                                                string("BT-ALT"), DEBUG_PORT);
    MBED_ASSERT(bt_alt_ser);
#endif /* MBED_CONF_APP_KISS_UART_TX_ALT */
#if MBED_CONF_APP_HAS_BLE == 1
    ThisThread::sleep_for(HALF_SECOND);
    auto ble_ser = make_shared<BLESerial>();
    //auto ble_aprs_ser = make_shared<KISSSerialBLE>(string("BLE-APRS"), APRS_PORT);
    //auto ble_voice_ser = make_shared<KISSSerialBLE>(string("BLE-VOICE"), VOICE_PORT);
    //auto ble_dbg_ser = make_shared<KISSSerialBLE>(string("BLE-DBG"), DEBUG_PORT);
#endif /* MBED_CONF_APP_HAS_BLE */
    debug_printf(DBG_INFO, "Serial threads started");
    ThisThread::sleep_for(HALF_SECOND);
    send_status();

    rx_frame_thread.start(rx_frame_ser_thread_fn);

    // Start up the GPS code
#if 0
    if(radio_cb.gps_en) {
        gps_thread.start(gpsd_thread_fn);
    }
#endif

    // Wait for 2 seconds in MANAGEMENT mode
    current_mode = system_state_t::MANAGEMENT;
    oled->printf("MANAGEMENT mode...\r\n");
    oled->display();
    oled->display();
    send_status();
    led1.LEDFastBlink();
    ThisThread::sleep_for(TWO_SECONDS);
    while(stay_in_management) {
        ThisThread::sleep_for(FIVE_SECONDS);
    }
    current_mode = system_state_t::RUNNING;
    send_status();

    led1.LEDBlink();
    led2.LEDOff();
    led3.LEDOff();

    // Test the FEC
#if 0
    debug_printf(DBG_INFO, "Now testing the FEC\r\n");
    auto fec_frame = make_shared<Frame>();  
    debug_printf(DBG_INFO, "Size of fec_frame is %d\r\n", fec_frame->codedSize());
    print_memory_info();
    {
    auto fec_test_fec = make_shared<FEC>(Frame::size());
    fec_test_fec->benchmark(TENTH_SECOND);
    auto fec_test_interleave = make_shared<FECInterleave>(Frame::size());
    fec_test_interleave->benchmark(TENTH_SECOND);
    auto fec_test_conv = make_shared<FECConv>(Frame::size(), 2, 9);
    fec_test_conv->benchmark(TENTH_SECOND);
    ThisThread::sleep_for(HALF_SECOND);
    auto fec_test_rsv = make_shared<FECRSV>(Frame::size(), 2, 9, 8);
    fec_test_rsv->benchmark(TENTH_SECOND);
    ThisThread::sleep_for(HALF_SECOND);
    print_memory_info();
    auto fec_test_rsv_big = make_shared<FECRSV>(Frame::size(), 3, 9, 8);
    fec_test_rsv_big->benchmark(TENTH_SECOND);
    ThisThread::sleep_for(HALF_SECOND);
    print_memory_info();
    } 
print_memory_info();
ThisThread::sleep_for(500);
#endif
    // Start the NVRAM logging thread
    debug_printf(DBG_INFO, "Starting the NV logger\r\n");
    nv_log_thread.start(nv_log_fn);

    ThisThread::sleep_for(QUARTER_SECOND);

    // Set up the radio
    debug_printf(DBG_INFO, "Initializing the Radio\r\n");
    init_radio();
    ThisThread::sleep_for(QUARTER_SECOND);

    // Start the mesh protocol thread
    debug_printf(DBG_INFO, "Starting the mesh protocol thread\r\n");
    mesh_protocol_thread.start(mesh_protocol_fsm);

    debug_printf(DBG_INFO, "Time to chill...\r\n");
    ThisThread::sleep_for(QUARTER_SECOND);  

    // Start the beacon thread
    debug_printf(DBG_INFO, "Starting the beacon\r\n");
    background_queue.call_every(static_cast<int>(radio_cb.net_cfg.beacon_interval*ONE_SECOND), 
                            beacon_fn);
    ThisThread::sleep_for(QUARTER_SECOND);
 
    // Start the OLED monitoring
    background_queue.call(oled_mon_fn);

    // Enable the watchdog timer if configured to do so
#if 0
    if(radio_cb.watchdog_timer_en) {
        debug_printf(DBG_INFO, "Enabling watchdog timer\r\n");
        // Start the WDT thread
        wdt_thread.start(wdt_fn);
    }
#endif

    debug_printf(DBG_INFO, "Starting the memory usage tracker\r\n");
    start_max_memory_usage();

    debug_printf(DBG_INFO, "Started everything\r\n");

    for(;;) {
//        print_stats();
        ThisThread::sleep_for(FIVE_SECONDS);
    }
}

