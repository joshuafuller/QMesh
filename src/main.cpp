/*
QMesh
Copyright (C) 2022 Daniel R. Fay

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

#include "os_portability.hpp"
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

static constexpr int TENTH_SECOND = 100; //NOLINT
static constexpr int QUARTER_SECOND = 250;
static constexpr int HALF_SECOND = 500;
static constexpr int ONE_SECOND = 1000;
static constexpr int TWO_SECONDS = 2000;
static constexpr int FIVE_SECONDS = 5000;
static constexpr int I2C_FREQ = 400000;

portability::Thread *mesh_protocol_thread = nullptr;
portability::Thread *rx_frame_thread = nullptr;
portability::Thread *nv_log_thread = nullptr;
portability::Thread *background_thread = nullptr;
portability::EventQueue *background_queue = nullptr;


static void create_background_queue();
static void create_background_queue() {
    constexpr int THREAD_STACK_SIZE = 4096;
    background_queue = new portability::EventQueue();    
    background_thread = new portability::Thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "BG"); /// Background thread
    PORTABLE_ASSERT(background_thread != nullptr);
    PORTABLE_ASSERT(background_queue != nullptr);
    osStatus stat = background_thread->start(callback(background_queue, &portability::EventQueue::dispatch_forever));
    PORTABLE_ASSERT(stat == osOK);
}

static void create_threads();
static void create_threads() {
    constexpr int THREAD_STACK_SIZE = 4096;
    //background_queue = new portability::EventQueue();
    mesh_protocol_thread = new portability::Thread(osPriorityRealtime, THREAD_STACK_SIZE, nullptr, "MESH-FSM"); /// Handles the mesh protocol
    rx_frame_thread = new portability::Thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "RX-FRAME"); /// Processes and routes received Frames
    nv_log_thread = new portability::Thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "NV-LOG"); /// Logging to the QSPI flash
    //background_thread = new portability::Thread(osPriorityNormal, THREAD_STACK_SIZE, nullptr, "BG"); /// Background thread
} 


static void setup_uarts();
static void setup_uarts() {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(radio_cb.has_esp_cfg_msg);
    // Start the serial handler threads
#ifdef MBED_CONF_APP_DEBUG_UART_TX
    portability::sleep(HALF_SECOND);
    auto *debug_ser = new KISSSerialUART(MBED_CONF_APP_DEBUG_UART_TX, MBED_CONF_APP_DEBUG_UART_RX, 
                                DEBUG_PORT);
    PORTABLE_ASSERT(debug_ser);
#endif /* MBED_CONF_APP_DEBUG_UART_TX */

#ifdef MBED_CONF_APP_KISS_UART_TX_ESP32_0
    portability::sleep(HALF_SECOND);
    PORTABLE_ASSERT(radio_cb.esp_cfg_msg.has_esp0);
    auto *esp32_0_ser = new KISSSerialUART(KISS_UART_TX_ESP32_0, KISS_UART_RX_ESP32_0, 
                    radio_cb.esp_cfg_msg.esp0, DEBUG_PORT);
    PORTABLE_ASSERT(esp32_0_ser);
#endif /* MBED_CONF_APP_KISS_UART_TX_ESP32_0 */

#ifdef MBED_CONF_APP_KISS_UART_TX_ESP32_1
    portability::sleep(HALF_SECOND);
    PORTABLE_ASSERT(radio_cb.esp_cfg_msg.has_esp1);
    auto *esp32_1_ser = new KISSSerialUART(KISS_UART_TX_ESP32_1, KISS_UART_RX_ESP32_1, 
                    radio_cb.esp_cfg_msg.esp1, DEBUG_PORT);
    PORTABLE_ASSERT(esp32_1_ser);
#endif /* MBED_CONF_APP_KISS_UART_TX_ESP32_1 */
}

time_t boot_timestamp;

system_state_t current_mode = system_state_t::BOOTING;
atomic<bool> stay_in_management(false);

void send_status();

portability::DigitalIn *user_button = nullptr;
SoftI2C *oled_i2c = nullptr;
shared_ptr<Adafruit_SSD1306_I2c> oled;
void create_serial_data_objects();
void create_kiss_serial_data_objects();
void create_radio_timing_data_objects();
void create_nv_settings_objects();
void create_radio_objects();
void create_mesh_protocol_objects();
void create_led_objects();
static void create_peripherals();
static void create_peripherals() {
    #ifdef USER_BUTTON
    user_button = new portability::DigitalIn(USER_BUTTON);
    #else // for the nRF52 board
    user_button = new portability::DigitalIn(BUTTON1);
    #endif
    oled_i2c = new SoftI2C(OLED_SDA, OLED_SCL);  // SDA, SCL
}

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
// pet the watchdog
static void wdt_pet() { //NOLINT
    Watchdog::get_instance().kick();
    background_queue->call_in(WDT_TIMEOUT_MS/2, wdt_pet);
}
#endif


// main() runs in its own thread in the OS
auto main() -> int
{
    portability::special_init();
    create_background_queue();
    create_serial_data_objects();
    create_kiss_serial_data_objects();
    create_radio_objects();

    create_threads();
    create_radio_timing_data_objects();
    create_nv_settings_objects();
    create_mesh_protocol_objects();
    create_led_objects();
    create_peripherals();

#if MBED_CONF_APP_HAS_BLE == 1
    portability::sleep(HALF_SECOND);
    auto ble_ser = make_shared<BLESerial>();
    //auto ble_aprs_ser = make_shared<KISSSerialBLE>(string("BLE-APRS"), APRS_PORT);
    //auto ble_voice_ser = make_shared<KISSSerialBLE>(string("BLE-VOICE"), VOICE_PORT);
    //auto ble_dbg_ser = make_shared<KISSSerialBLE>(string("BLE-DBG"), DEBUG_PORT);
#endif /* MBED_CONF_APP_HAS_BLE */

    start_cal();
    time(&boot_timestamp);
#if 0
#if MBED_CONF_APP_HAS_WATCHDOG == 1
    Watchdog &wdt = Watchdog::get_instance();
    wdt.start();
    wdt_pet();
#endif
#endif

    // Set up the LEDs
    portability::sleep(HALF_SECOND);
    PORTABLE_ASSERT(led1 != nullptr);
    led1->setEvtQueue(background_queue);
    PORTABLE_ASSERT(led2 != nullptr);
    led2->setEvtQueue(background_queue);
    PORTABLE_ASSERT(led3 != nullptr);
    led3->setEvtQueue(background_queue);  

    PORTABLE_ASSERT(oled_i2c != nullptr);
    oled_i2c->frequency(I2C_FREQ);
    oled_i2c->start();
    
    constexpr int OLED_NUM_LINES = 32;
    constexpr int OLED_NUM_COLS = 128;
    constexpr int OLED_CONST = 0x78;
    oled = make_shared<Adafruit_SSD1306_I2c>(*oled_i2c, MBED_CONF_APP_FHSS_MON, OLED_CONST, OLED_NUM_LINES, OLED_NUM_COLS);

    oled->printf("Welcome to QMesh\r\n");
    oled->display();

    led1->LEDBlink();
    oled->printf("In rescue mode...\r\n");
    oled->display();
	portability::sleep(ONE_SECOND);
    PORTABLE_ASSERT(user_button != nullptr);
	if(*user_button != 0) {
		led1->LEDFastBlink();
		portability::sleep(ONE_SECOND);
		if(*user_button != 0) {
			rescue_filesystem();
		}
	}
	led1->LEDSolid();
#if MBED_CONF_APP_HAS_BLE == 1
    auto *push_button = new PushButton(BUTTON1);
#else
    auto *push_button = new PushButton(USER_BUTTON);
#endif
    PORTABLE_ASSERT(push_button != nullptr);
    push_button->SetQueue(*background_queue);
    portability::sleep(ONE_SECOND);

    // Mount the filesystem, load the configuration, log the bootup
    init_filesystem();
    load_settings_from_flash();
    log_boot();

    auto *disp_file = fopen("/fs/display.off", "re");
    if(disp_file == nullptr) {
        oled->displayOn();
        fclose(disp_file);
    } else {
        oled->displayOff();
    }

    Frame::seed_stream_id(radio_cb.address);

    setup_uarts();

    debug_printf(DBG_INFO, "Serial threads started");
    portability::sleep(HALF_SECOND);
    send_status();

    osStatus stat = rx_frame_thread->start(rx_frame_ser_thread_fn);
    PORTABLE_ASSERT(stat == osOK);

    // Start up the GPS code
#if 0
    if(radio_cb.gps_en) {
        stat = gps_thread.start(gpsd_thread_fn);
        PORTABLE_ASSERT(stat == osOK);
    }
#endif

    // Wait for 2 seconds in MANAGEMENT mode
    current_mode = system_state_t::MANAGEMENT;
    oled->printf("MANAGEMENT mode...\r\n");
    oled->display();
    oled->display();
    send_status();
    led1->LEDFastBlink();
    portability::sleep(TWO_SECONDS);
    while(stay_in_management) {
        portability::sleep(FIVE_SECONDS);
    }
    current_mode = system_state_t::RUNNING;
    send_status();

    led1->LEDBlink();
    led2->LEDOff();
    led3->LEDOff();

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
    sleep_portable(HALF_SECOND);
    auto fec_test_rsv = make_shared<FECRSV>(Frame::size(), 2, 9, 8);
    fec_test_rsv->benchmark(TENTH_SECOND);
    sleep_portable(HALF_SECOND);
    print_memory_info();
    auto fec_test_rsv_big = make_shared<FECRSV>(Frame::size(), 3, 9, 8);
    fec_test_rsv_big->benchmark(TENTH_SECOND);
    sleep_portable(HALF_SECOND);
    print_memory_info();
    } 
print_memory_info();
sleep_portable(500);
#endif
    // Start the NVRAM logging thread
    debug_printf(DBG_INFO, "Starting the NV logger\r\n");
    PORTABLE_ASSERT(nv_log_thread != nullptr);
    stat = nv_log_thread->start(nv_log_fn);
    PORTABLE_ASSERT(stat == osOK);

    portability::sleep(QUARTER_SECOND);

    // Set up the radio
    debug_printf(DBG_INFO, "Initializing the Radio\r\n");
    init_radio();
    portability::sleep(QUARTER_SECOND);

    // Start the mesh protocol thread
    debug_printf(DBG_INFO, "Starting the mesh protocol thread\r\n");
    PORTABLE_ASSERT(mesh_protocol_thread != nullptr);
    stat = mesh_protocol_thread->start(mesh_protocol_fsm);
    PORTABLE_ASSERT(stat == osOK);

    debug_printf(DBG_INFO, "Time to chill...\r\n");
    portability::sleep(QUARTER_SECOND);  

    // Start the beacon thread
    debug_printf(DBG_INFO, "Starting the beacon\r\n");
    PORTABLE_ASSERT(background_queue != nullptr);
    background_queue->call_every(static_cast<int>(radio_cb.net_cfg.beacon_interval*ONE_SECOND), 
                            beacon_fn);
    portability::sleep(QUARTER_SECOND);
 
    // Start the OLED monitoring
    background_queue->call(oled_mon_fn);

    // Enable the watchdog timer if configured to do so
#if 0
    if(radio_cb.watchdog_timer_en) {
        debug_printf(DBG_INFO, "Enabling watchdog timer\r\n");
        // Start the WDT thread
        wdt_thread.start(wdt_fn);
    }
#endif

    debug_printf(DBG_INFO, "Starting the memory usage tracker\r\n");
    //start_max_memory_usage();

    debug_printf(DBG_INFO, "Started everything\r\n");

    for(;;) {
//        print_stats();
        portability::sleep(FIVE_SECONDS);
    }
}

