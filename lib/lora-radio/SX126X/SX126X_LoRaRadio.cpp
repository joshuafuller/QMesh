/**
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRaWAN stack layer that controls both MAC and PHY underneath

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian & Gilbert Menth

Copyright (c) 2019, Arm Limited and affiliates.

SPDX-License-Identifier: BSD-3-Clause
*/

#include "os_portability.hpp"
#include <cmath>
#include "mbed_wait_api.h"
#include "Timer.h"
#include "radio_timing.hpp"
#include "SX126X_LoRaRadio.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <sstream>
#include "kiss_serial.hpp"
#include "peripherals.hpp"
#include "serial_data.hpp"

// Squash a warning about wait_ms being deprecated
#define wait_ms(x) portability::wait_us(x*1000)

#ifdef MBED_CONF_SX126X_LORA_DRIVER_SPI_FREQUENCY
#define SPI_FREQUENCY    MBED_CONF_SX126X_LORA_DRIVER_SPI_FREQUENCY
#else
#define SPI_FREQUENCY    16000000
#endif


/**
 * Signals
 */
#define SIG_INTERRUPT        0x02

//static portability::Thread *lora_irq_thread;

/*!
 * FSK bandwidth definition
 */
typedef struct //NOLINT
{
    uint32_t bandwidth;
    uint8_t  register_value;
} fsk_bw_t; 

static const fsk_bw_t fsk_bandwidths[] = //NOLINT
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};

const uint8_t sync_word[] = {0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00,0x00}; //NOLINT

// in ms                                 SF12    SF11    SF10    SF9    SF8    SF7
const float lora_symbol_time[3][6] = {{ 32.768, 16.384, 8.192, 4.096, 2.048, 1.024 },  // 125 KHz //NOLINT
                                       { 16.384, 8.192,  4.096, 2.048, 1.024, 0.512 },  // 250 KHz
                                       { 8.192,  4.096,  2.048, 1.024, 0.512, 0.256 }}; // 500 KHz


DigitalOut fhss_mon_sig(MBED_CONF_APP_FHSS_MON);


SX126X_LoRaRadio::SX126X_LoRaRadio(PinName mosi,
                                    PinName miso,
                                    PinName sclk,
                                    PinName nss,
                                    PinName rxen,
                                    PinName txen,
                                    PinName reset,
                                    PinName dio1,
                                    PinName  /*dio2*/,
                                    PinName  /*nrst*/,
                                    PinName busy,
                                    PinName pwrctl)
    : _spi(mosi, miso, sclk),
      _chip_select(nss, 1),
      _reset_ctl(reset),
      _dio1_ctl(dio1, PullNone),
      _busy(busy, PullNone),
      _rxen(rxen),
      _txen(txen),
      _pwr_ctl(pwrctl),
      irq_thread(osPriorityRealtime, 4096, nullptr, "LR-SX126X")
{
    _radio_events = nullptr;
    _reset_ctl = 1;
    _image_calibrated = false;
    _force_image_calibration = false;
    _active_modem = MODEM_LORA;

    //lora_irq_thread = &irq_thread;

    irq_thread.start(callback(this, &SX126X_LoRaRadio::rf_irq_task));
}

SX126X_LoRaRadio::~SX126X_LoRaRadio() = default;

/**
 * Acquire lock
 */
void SX126X_LoRaRadio::lock()
{
    mutex.lock();
}

/**
 * Release lock
 */
void SX126X_LoRaRadio::unlock()
{
    mutex.unlock();
}

/**
 * Thread task handling IRQs
 */
void SX126X_LoRaRadio::rf_irq_task()
{
    for (;;) {
        uint32_t flags = ThisThread::flags_wait_any(0x7FFFFFFF);

        lock();
        if ((flags & SIG_INTERRUPT) != 0U) {
            handle_dio1_irq();
        }
        unlock();
    }
}

void SX126X_LoRaRadio::dio1_irq_isr()
{
    fhss_mon_sig = static_cast<int>(fhss_mon_sig == 0);
    #warning investigate whether we really need this critical section lock
    CriticalSectionLock lock;
    // Start timing the duration since the packet was receive
    cur_tmr->start();
    *rx_int_mon = 0;
    *tx_int_mon = 0;

    irq_thread.flags_set(SIG_INTERRUPT);
}

auto SX126X_LoRaRadio::get_irq_status() -> uint16_t
{
    array<uint8_t, 2> status{};

    read_opmode_command(static_cast<uint8_t>(RADIO_GET_IRQSTATUS), status.data(), 2);
    constexpr uint16_t SHIFT_ONE_BYTE = 8;
    return (status[0] << SHIFT_ONE_BYTE) | status[1]; //NOLINT
}

void SX126X_LoRaRadio::clear_irq_status(uint16_t irq)
{
    array<uint8_t, 2> buf{};

    constexpr uint16_t LOWER_BYTE = 0x00FF;
    constexpr uint16_t SHIFT_ONE_BYTE = 8;
    buf[0] = static_cast<uint8_t>((irq >> SHIFT_ONE_BYTE) & LOWER_BYTE); //NOLINT
    buf[1] = static_cast<uint8_t>(irq & LOWER_BYTE);
    write_opmode_command(static_cast<uint8_t>(RADIO_CLR_IRQSTATUS), buf.data(), 2);
}

// TODO(unknown): - Better do CAD here. CAD code is already part of the driver
//       It needs to be hooked up to the stack (this API will need change
//       and the stack will need changes too)
bool SX126X_LoRaRadio::perform_carrier_sense(const radio_modems_t modem,
                                             const uint32_t freq,
                                             const int16_t rssi_threshold,
                                             const uint32_t max_carrier_sense_time,
                                             const bool locking)
{
    if(locking) { lock(); }

    bool status = true;
    int16_t rssi = 0;

    set_modem(modem);
    set_channel(freq);
    _reception_mode = RECEPTION_MODE_OTHER;
    _rx_timeout = 0x00000000;
    receive();

    // hold on a bit, radio turn-around time
    wait_ms(1);

    LowPowerTimer elapsed_time;
    elapsed_time.start();

    // Perform carrier sense for maxCarrierSenseTime
    while (elapsed_time.read_ms() < static_cast<int>(max_carrier_sense_time)) {
        rssi = get_rssi(); //NOLINT

        if (rssi > rssi_threshold) {
            status = false;
            break;
        }
    }

    sleep();

    if(locking) { unlock(); }
    return status;
}

void SX126X_LoRaRadio::start_cad(const bool  /*locking*/)
{
    // TODO(unknown): CAD is more advanced in SX126X. We will need API change in LoRaRadio
    //       for this to act properly
    write_opmode_command(static_cast<uint8_t>(RADIO_SET_CAD), nullptr, 0);
}


void SX126X_LoRaRadio::configure_freq_hop(AntiInterference *my_anti_inter) 
{
    anti_inter = my_anti_inter;
}


void SX126X_LoRaRadio::configure_freq_hop_freqs(vector<uint32_t>& my_hop_freqs) 
{
    hop_freqs = my_hop_freqs;
    cur_hop_freq = hop_freqs.begin();
}


// Sequentially scan through the hop frequencies
void SX126X_LoRaRadio::rx_hop_frequency()
{
    PORTABLE_ASSERT(radio_cb.valid);
    if(radio_cb.radio_cfg.frequencies_count > 1) {
        set_channel(*cur_hop_freq);
        if(cur_hop_freq == hop_freqs.end() || ++cur_hop_freq == hop_freqs.end()) {
            cur_hop_freq = hop_freqs.begin();
            fhss_mon_sig = static_cast<int>(fhss_mon_sig == 0);
        }
    }
}


// Randomly select a hop frequency
void SX126X_LoRaRadio::tx_hop_frequency(const bool locking)
{
    if(locking) { lock(); }
    set_channel(*(hop_freqs.begin()+anti_inter->nextChannel()) + anti_inter->freqOffset());
    if(locking) { unlock(); }
}


/**
 * TODO: The purpose of this API is unclear.
 *       Need to start an internal discussion.
 */
auto SX126X_LoRaRadio::check_rf_frequency(const uint32_t  /*frequency*/) -> bool
{
    // Implement check. Currently all frequencies are supported ? What band ?
    return true;
}


void SX126X_LoRaRadio::set_tx_continuous_wave(const uint32_t  /*freq*/, const int8_t  /*power*/, 
                                                const uint16_t  /*time*/, const bool locking)
{
    if(locking) { lock(); }
    set_tx_power(_tx_power);

    if(_rxen.is_connected() != 0) {
        _rxen = 0;
    }
    if(_txen.is_connected() != 0) {
        _txen = 1;
    }

    write_opmode_command(static_cast<uint8_t>(RADIO_SET_TXCONTINUOUSWAVE), nullptr, 0);
    if(locking) { lock(); }
}


void SX126X_LoRaRadio::set_tx_continuous_preamble(const bool locking) {
    if(locking) { lock(); }
    set_tx_power(_tx_power);

    if(_rxen.is_connected() != 0) {
        _rxen = 0;
    }
    if(_txen.is_connected() != 0) {
        _txen = 1;
    }

    write_opmode_command(static_cast<uint8_t>(RADIO_SET_TXCONTINUOUSPREAMBLE), nullptr, 0);
    if(locking) { unlock(); }
}


void SX126X_LoRaRadio::read_rssi_thread_fn() {
    collect_rssi.store(true);
    CalTimer tmr;
    rssi_list_sptr = make_shared<list<pair<uint32_t, uint8_t> > >();
    tmr.start();
    while(collect_rssi.load()) {
        rssi_list_sptr->push_back(pair<uint32_t, uint8_t>(tmr.read(), get_rssi()));
    }
}


void SX126X_LoRaRadio::start_read_rssi() {
    rssi_mon->write(0x1);
    soft_dec_thread.start(callback(this,  &SX126X_LoRaRadio::read_rssi_thread_fn));
}


void SX126X_LoRaRadio::stop_read_rssi() {
    collect_rssi.store(false);
    soft_dec_thread.join();
    rssi_mon->write(0x0);
}


void SX126X_LoRaRadio::tx_timeout_handler() {
    _radio_events->tx_timeout();
}


void SX126X_LoRaRadio::handle_dio1_irq()
{
    uint16_t irq_status = get_irq_status();
    fhss_mon_sig = static_cast<int>(fhss_mon_sig == 0);

    //debug_printf(DBG_INFO, "IRQ is %8x\r\n", irq_status);
    if ((irq_status & IRQ_TX_DONE) == IRQ_TX_DONE) {
        tx_timeout.detach();
        _radio_events->tx_done_tmr(cur_tmr_sptr);
    }
    if ((irq_status & IRQ_RX_DONE) == IRQ_RX_DONE) {
        // Implicit header timeout workaround
        constexpr uint16_t IMPL_WORKAROUND_REG0 = 0x0920;
        constexpr uint16_t IMPL_WORKAROUND_REG1 = 0x0944;
        write_to_register(IMPL_WORKAROUND_REG0, 0x00);
        uint8_t val = read_register(IMPL_WORKAROUND_REG1);
        val |= 0x02UL;
        write_to_register(IMPL_WORKAROUND_REG1, 0x02);
        // --------------------
        radio->cad_pending.store(false);
        //stop_read_rssi();
        if ((irq_status & IRQ_CRC_ERROR) == IRQ_CRC_ERROR) {
            if ((_radio_events != nullptr) && _radio_events->rx_error) {
                _radio_events->rx_error();
            }
        } else {
            uint8_t offset = 0;
            uint8_t payload_len = 0;
            int16_t rssi = 0;
            int8_t snr = 0;
            packet_status_t pkt_status;
            get_rx_buffer_status(&payload_len, &offset);
            read_fifo(_data_buffer, payload_len, offset);
            get_packet_status(&pkt_status);
            if (pkt_status.modem_type == MODEM_FSK) {
                rssi = pkt_status.params.gfsk.rssi_sync; //NOLINT 
            } else {
                rssi = pkt_status.params.lora.rssi_pkt; //NOLINT
                snr = pkt_status.params.lora.snr_pkt;
            }
            _radio_events->rx_done_tmr(_data_buffer, rssi_list_sptr, cur_tmr_sptr, 
                            payload_len, rssi, snr);
            debug_printf(DBG_INFO, "Frequency received on was %d\r\n", *cur_hop_freq);
        }
    }
    else if ((irq_status & IRQ_CAD_DONE) == IRQ_CAD_DONE) {
        if((irq_status & IRQ_CAD_ACTIVITY_DETECTED) != 0) {
            if(!stop_cad.load() && !cad_rx_running) {
                radio->receive_cad_rx();
            } else {
                cad_pending.store(false);
            }
        }
        else {  
            if(!stop_cad.load()) {
                rx_hop_frequency();
                fhss_mon_sig = static_cast<int>(fhss_mon_sig == 0); 
                radio->receive_cad_rx();
                fhss_mon_sig = static_cast<int>(fhss_mon_sig == 0);
            } else {
                cad_pending.store(false);
            }
        }
    }
    else if ((irq_status & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT) {
        if ((_radio_events->tx_timeout) && (_operation_mode == MODE_TX)) {
            tx_timeout.detach();
            debug_printf(DBG_WARN, "Tx Timeout\r\n");
            _radio_events->tx_timeout();
        } else if (((_radio_events != nullptr) && _radio_events->rx_timeout) && 
                        (_operation_mode == MODE_RX)) {
            debug_printf(DBG_WARN, "Rx Timeout\r\n");
            _radio_events->rx_timeout();
        }
        if(!stop_cad.load()) {
            PORTABLE_ASSERT(radio_cb.valid);
            if(radio_cb.radio_cfg.frequencies_count > 1) {
                radio->rx_hop_frequency();
                radio->receive_cad_rx();
            } else {
                radio->receive_sel();
            }
        }
        else {
            radio->cad_pending.store(false);
        }
    }

    clear_irq_status(IRQ_RADIO_ALL);
    cur_tmr_sptr = make_shared<CalTimer>();
    cur_tmr = cur_tmr_sptr.get();
}

void SX126X_LoRaRadio::set_device_ready()
{
    if (_operation_mode == MODE_SLEEP) {
        wakeup();
    }
}

void SX126X_LoRaRadio::calibrate_image(const uint32_t freq, const bool locking)
{
    if(locking) { lock(); }

    array<uint8_t, 2> cal_freq{};
    PORTABLE_ASSERT(freq <= 960000000);
    PORTABLE_ASSERT(freq >= 150000000);

    if (freq > 900000000) {
        cal_freq[0] = 0xE1;
        cal_freq[1] = 0xE9;
    } else if (freq > 850000000) {
        cal_freq[0] = 0xD7;
        cal_freq[1] = 0xD8;
    } else if (freq > 770000000) {
        cal_freq[0] = 0xC1;
        cal_freq[1] = 0xC5;
    } else if (freq > 460000000) {
        cal_freq[0] = 0x75;
        cal_freq[1] = 0x81;
    } else if (freq > 425000000) {
        cal_freq[0] = 0x6B;
        cal_freq[1] = 0x6F;
    }

    write_opmode_command(static_cast<uint8_t>(RADIO_CALIBRATEIMAGE), cal_freq.data(), 2);

    _image_calibrated = true;

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_channel(const uint32_t frequency, const bool locking)
{
    if(locking) { lock(); }

    PORTABLE_ASSERT(frequency <= 960000000);
    PORTABLE_ASSERT(frequency >= 150000000);

    array<uint8_t, 4> buf{};
    uint32_t freq = 0;

#if 0
    if ( _force_image_calibration || !_image_calibrated) {
        debug_printf(DBG_INFO, "Calibrating the radio\r\n");
        calibrate_image(frequency);
        _image_calibrated = true;
    }
#endif
    constexpr uint32_t LOWER_BYTE = 0xFF;
    constexpr uint32_t THREE_BYTES = 24;
    constexpr uint32_t TWO_BYTES = 16;
    constexpr uint32_t ONE_BYTE = 8;
    freq = static_cast<uint32_t>(ceil((static_cast<float>(frequency) / static_cast<float>(FREQ_STEP))));
    buf[0] = static_cast<uint8_t>((freq >> THREE_BYTES) & LOWER_BYTE);
    buf[1] = static_cast<uint8_t>((freq >> TWO_BYTES) & LOWER_BYTE);
    buf[2] = static_cast<uint8_t>((freq >> ONE_BYTE) & LOWER_BYTE);
    buf[3] = static_cast<uint8_t>(freq & LOWER_BYTE);

    write_opmode_command(static_cast<uint8_t>(RADIO_SET_RFFREQUENCY), buf.data(), 4);

    if(locking) { unlock(); }
}

/**
 * Put radio in Standby mode
 */
void SX126X_LoRaRadio::standby(const bool locking)
{
    if(locking) { lock(); }

    if (_operation_mode == MODE_STDBY_RC || _operation_mode == MODE_STDBY_XOSC) {
        return;
    }

    set_device_ready();
    uint8_t standby_mode = STDBY_XOSC;
    write_opmode_command(static_cast<uint8_t>(RADIO_SET_STANDBY), &standby_mode, 1);

    if (standby_mode == STDBY_RC) {
        _operation_mode = MODE_STDBY_RC;
    } else {
        _operation_mode = MODE_STDBY_XOSC;
    }

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_dio2_as_rfswitch_ctrl(const uint8_t enable, const bool locking)
{
    if(locking) { lock(); }
    PORTABLE_ASSERT(enable == 0 || enable == 1);

    uint8_t en = enable;
    write_opmode_command(RADIO_SET_RFSWITCHMODE, &en, 1);

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_dio3_as_tcxo_ctrl(const radio_TCXO_ctrl_voltage_t voltage,
                                             const uint32_t timeout,
                                             const bool locking)
{
    if(locking) { lock(); }

    uint8_t buf[4];

    buf[0] = voltage & 0x07;
    buf[1] = static_cast<uint8_t>((timeout >> 16) & 0xFF);
    buf[2] = static_cast<uint8_t>((timeout >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>(timeout & 0xFF);

    write_opmode_command(RADIO_SET_TCXOMODE, buf, 4);

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::init_radio(radio_events_t *events, const bool locking)
{
    if(locking) { lock(); }

    
    if(_txen.is_connected() != 0) {
        _txen = 0;
    }
    if(_rxen.is_connected() != 0) {
        _rxen = 1;
    }

    PORTABLE_ASSERT(events != nullptr);
    _radio_events = events;
    // attach DIO1 interrupt line to its respective ISR
    _dio1_ctl.rise(callback(this, &SX126X_LoRaRadio::dio1_irq_isr));

    // Allocate the first timer
    tmr_sem_ptr = new Semaphore(1);
    PORTABLE_ASSERT(tmr_sem_ptr != nullptr);
    tmr_sem_ptr->acquire();
    cur_tmr_sptr = make_shared<CalTimer>();
    cur_tmr = cur_tmr_sptr.get();
    tmr_sem_ptr->release();

    // Hold chip-select high
    _chip_select = 1;
    _spi.format(8, 0);
    _spi.frequency(SPI_FREQUENCY);
    // 100 us wait to settle down
    portability::wait_us(100);

    radio_reset();

#if MBED_CONF_LORA_PUBLIC_NETWORK
    _network_mode_public = true;
#else
    _network_mode_public = false;
#endif
    // this is a POR sequence
    cold_start_wakeup();

    
    if(_txen.is_connected() != 0) {
        _txen = 0;
    }
    if(_rxen.is_connected() != 0) {
        _rxen = 0;
    }

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::cold_start_wakeup(const bool locking)
{
    if(locking) { lock(); }

    uint8_t regulator_mode = MBED_CONF_SX126X_LORA_DRIVER_REGULATOR_MODE;
    write_opmode_command(RADIO_SET_REGULATORMODE, &regulator_mode, 1);
    set_buffer_base_addr(0x00, 0x00);

#ifdef USES_TCXO
    caliberation_params_t calib_param;
    constexpr float US_PER_UNIT = 15.625F;
    PORTABLE_ASSERT(radio_cb.valid);
    uint32_t tcxo_time = ceilf(radio_cb.radio_cfg.tcxo_time_us / US_PER_UNIT);
    if(tcxo_time == 0) { // 0 just hangs the system, so make it slightly bigger
        tcxo_time = 1;
    }
    debug_printf(DBG_INFO, "Setting TCXO setup time to %f us or %u units\r\n", 
                radio_cb.radio_cfg.tcxo_time_us, tcxo_time);
    set_dio3_as_tcxo_ctrl(TCXO_VOLTAGE, tcxo_time);
    constexpr uint8_t CALIB_VAL = 0x7F;
    calib_param.value = CALIB_VAL;
    write_opmode_command(RADIO_CALIBRATE, &calib_param.value, 1);
#endif

    set_dio2_as_rfswitch_ctrl(1U); // it takes a int, so use that instead of a bool

    _operation_mode = MODE_STDBY_RC;

    set_modem(_active_modem);

    if (_active_modem == MODEM_LORA) {
        set_public_network(_network_mode_public);
    }

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_public_network(const bool enable, const bool locking)
{
    if(locking) { lock(); }
    if (enable) {
        // Change LoRa modem SyncWord
        write_to_register(REG_LR_SYNCWORD, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
        write_to_register(REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
    } else {
        // Change LoRa modem SyncWord
        write_to_register(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
        write_to_register(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
    }
    if(locking) { unlock(); }
}

auto SX126X_LoRaRadio::time_on_air(const radio_modems_t modem, const uint8_t pkt_len) -> uint32_t
{
    uint32_t air_time = 0;

    switch (modem) {
        case MODEM_FSK: {
            air_time = rint((8 * (_packet_params.params.gfsk.preamble_length
                            + (_packet_params.params.gfsk.syncword_length >> 3)
                            + ((_packet_params.params.gfsk.header_type
                                    == RADIO_PACKET_FIXED_LENGTH) ? 0.0F : 1.0F) + pkt_len
                                    + ((_packet_params.params.gfsk.crc_length == RADIO_CRC_2_BYTES) ? 2.0F : 0.0f))
                            / _mod_params.params.gfsk.bit_rate) * 1000);
        }
            break;
        case MODEM_LORA: {
            float ts = lora_symbol_time[_mod_params.params.lora.bandwidth - 4][12
                            - _mod_params.params.lora.spreading_factor];
            // time of preamble
            float t_preamble = (_packet_params.params.lora.preamble_length + 4.25F) * ts;
            // Symbol length of payload and time
            float tmp = ceil((8 * pkt_len - 4 * _mod_params.params.lora.spreading_factor
                    + 28 + 16 * _packet_params.params.lora.crc_mode
                    - ((_packet_params.params.lora.header_type == LORA_PACKET_FIXED_LENGTH) ? 20 : 0))
                             / (float) (4 * (_mod_params.params.lora.spreading_factor
                                     - ((_mod_params.params.lora.low_datarate_optimization > 0) ? 2 : 0))))
                                             * ((_mod_params.params.lora.coding_rate % 4) + 4);
            float n_payload = 8 + ((tmp > 0) ? tmp : 0);
            float t_payload = n_payload * ts;
            // Time on air
            float tOnAir = t_preamble + t_payload;
            // return milliseconds (as ts is in milliseconds)
            air_time = floor(tOnAir + 0.999);
        }
            break;
    }

    return air_time;
}

void SX126X_LoRaRadio::radio_reset(const bool locking)
{
    if(locking) { lock(); }

    // Do the reset
    _reset_ctl.output();
    _reset_ctl = 0;
    // should be enough, required is 50-100 us
    wait_ms(2);
    _reset_ctl.input();
    // give some time for automatic image calibration
    wait_ms(6);

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::wakeup(const bool locking)
{
    if(locking) { lock(); }
    // hold the NSS low, this should wakeup the chip.
    // now we should wait for the _busy line to go low
    if (_operation_mode == MODE_SLEEP) {
        _chip_select = 0;
        portability::wait_us(100);
        _chip_select = 1;
        portability::wait_us(100);
#if 0
#if MBED_CONF_SX126X_LORA_DRIVER_SLEEP_MODE == 1
        portability::wait_us(3500);
        // whenever we wakeup from Cold sleep state, we need to perform
        // image calibration
        _force_image_calibration = true;
        cold_start_wakeup();
#endif
#endif
    }

    // Set up the TX clamping workaround
    uint8_t val = read_register(0x08D8);
    val |= 0x1E;
    write_to_register(0x08D8, val);

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::sleep(const bool locking)
{
    if(locking) { lock(); }
    // warm start, power consumption 600 nA
    uint8_t sleep_state = 0x04;
    _operation_mode = MODE_SLEEP;

    write_opmode_command(RADIO_SET_SLEEP, &sleep_state, 1);
    wait_ms(2);

    if(locking) { unlock(); }
}

auto SX126X_LoRaRadio::random(const bool locking) -> uint32_t 
{
    if(locking) { lock(); }

    set_modem(MODEM_LORA);
    uint8_t buf[] = {0, 0, 0, 0};

    // Set radio in continuous reception
    _reception_mode = RECEPTION_MODE_OTHER;
    _rx_timeout = 0xFFFFFFFF;
    receive();
    wait_ms(1);
    read_register(RANDOM_NUMBER_GENERATORBASEADDR, buf, 4);
    standby();

    int val = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    if(locking) { unlock(); }
    return val;
}

void SX126X_LoRaRadio::write_opmode_command(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    _spi.lock();
    _chip_select = 0;

    while (_busy != 0) {
        // do nothing
    }

    _spi.write(cmd);

    for (int i = 0; i < size; i++) {
        _spi.write(buffer[i]);
    }

    _chip_select = 1;
    _spi.unlock();
#ifdef TRACE_SX1262_SPI
    stringstream reg_info;
    reg_info << "SX1262: CMD WRITE 0x" << setw(2) << setfill('0') << std::hex << (uint32_t) cmd << "; ";
    for(int i = 0; i < size; i++) {
        reg_info << "0x" << setw(2) << setfill('0') << std::hex << (uint32_t) buffer[i] << " ";
    }
    reg_info << "\r\n";
    debug_printf(DBG_WARN, reg_info.str().c_str());
#endif /* TRACE_SX1262_SPI */
}

void SX126X_LoRaRadio::write_opmode_command_dangling(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    _spi.lock();
    _chip_select = 0;

    while (_busy != 0) {
        // do nothing
    }

    _spi.write(cmd);

    for (int i = 0; i < size; i++) {
        _spi.write(buffer[i]);
    }

    //_chip_select = 1;
    //_spi.unlock();
#ifdef TRACE_SX1262_SPI
    stringstream reg_info;
    reg_info << "SX1262: CMD WRITE 0x" << setw(2) << setfill('0') << std::hex << (uint32_t) cmd << "; ";
    for(int i = 0; i < size; i++) {
        reg_info << "0x" << setw(2) << setfill('0') << std::hex << (uint32_t) buffer[i] << " ";
    }
    reg_info << "\r\n";
    debug_printf(DBG_WARN, reg_info.str().c_str());
#endif /* TRACE_SX1262_SPI */
}

void SX126X_LoRaRadio::write_opmode_command_finish()
{
//_chip_select = 1;
    _spi.unlock();
}

void SX126X_LoRaRadio::read_opmode_command(uint8_t cmd,
                                           uint8_t *buffer, uint16_t size)
{
    _spi.lock();
    PORTABLE_ASSERT(size > 0);
    PORTABLE_ASSERT(size <= 16);

    _chip_select = 0;

    while (_busy != 0) {
        // do nothing
    }

    _spi.write(cmd);
    _spi.write(0);

    for (int i = 0; i < size; i++) {
        buffer[i] = _spi.write(0);
    }

    _chip_select = 1;
    _spi.unlock();
}

void SX126X_LoRaRadio::write_to_register(uint16_t addr, uint8_t data)
{
    write_to_register(addr, &data, 1);
}

void SX126X_LoRaRadio::write_to_register(uint16_t addr, uint8_t *data,
                                         uint8_t size)
{
    _spi.lock();
    PORTABLE_ASSERT(size > 0);
    PORTABLE_ASSERT(size <= 16);
    _chip_select = 0;

    _spi.write(RADIO_WRITE_REGISTER);
    _spi.write((addr & 0xFF00) >> 8);
    _spi.write(addr & 0x00FF);

    for (int i = 0; i < size; i++) {
        _spi.write(data[i]);
    }

    _chip_select = 1;
    _spi.unlock();

#ifdef TRACE_SX1262_SPI
    stringstream reg_info;
    reg_info << "SX1262: REG WRITE 0x" << setw(4) << setfill('0') << std::hex << addr << "; ";
    for(int i = 0; i < size; i++) {
        reg_info << "0x" << setw(2) << setfill('0') << std::hex << (uint32_t) data[i] << " ";
    }
    reg_info << "\r\n";
    debug_printf(DBG_WARN, reg_info.str().c_str());
#endif /* TRACE_SX1262_SPI */
}

auto SX126X_LoRaRadio::read_register(uint16_t addr) -> uint8_t
{
    uint8_t data = 0;
    read_register(addr, &data, 1);
    return data;
}

void SX126X_LoRaRadio::read_register(uint16_t addr, uint8_t *buffer,
                                     uint8_t size)
{
    _spi.lock();
    
    PORTABLE_ASSERT(size > 0);
    PORTABLE_ASSERT(size <= 16);

    _chip_select = 0;

    _spi.write(RADIO_READ_REGISTER);
    _spi.write((addr & 0xFF00) >> 8);
    _spi.write(addr & 0x00FF);
    _spi.write(0);

    for (int i = 0; i < size; i++) {
        buffer[i] = _spi.write(0);
    }

    _chip_select = 1;
    _spi.unlock();
}

void SX126X_LoRaRadio::write_fifo(const uint8_t *const buffer, const uint8_t size, const bool locking)
{
    if(locking) { lock(); }

    _spi.lock();
    _chip_select = 0;

    _spi.write(RADIO_WRITE_BUFFER);
    _spi.write(0);

    for (int i = 0; i < size; i++) {
        _spi.write(buffer[i]);
    }

    _chip_select = 1;
    _spi.unlock();

#ifdef TRACE_SX1262_SPI
    stringstream reg_info;
    reg_info << "SX1262: FIFO WRITE ";
    for(int i = 0; i < size; i++) {
        reg_info << "0x" << setw(2) << setfill('0') << std::hex << (uint32_t) buffer[i] << " ";
    }
    reg_info << "\r\n";
    debug_printf(DBG_WARN, reg_info.str().c_str());
#endif /* TRACE_SX1262_SPI */

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_modem(const uint8_t modem, const bool locking)
{
    if(locking) { lock(); }

    PORTABLE_ASSERT(modem == 0 || modem == 1);
    _active_modem = modem;

    // setting modem type must happen in stnadby mode
    if (_operation_mode != MODE_STDBY_RC && _operation_mode != MODE_STDBY_XOSC) {
        standby();
    }

    write_opmode_command(RADIO_SET_PACKETTYPE, &_active_modem, 1);
    if(locking) { unlock(); }
}

auto SX126X_LoRaRadio::get_modem() -> uint8_t
{
    return _active_modem;
}

void SX126X_LoRaRadio::read_fifo(uint8_t *buffer, uint8_t size, uint8_t offset)
{
    _spi.lock();

    _chip_select = 0;

    _spi.write(RADIO_READ_BUFFER);
    _spi.write(offset);
    _spi.write(0);

    for (int i = 0; i < size; i++) {
        buffer[i] = _spi.write(0);
    }

    _chip_select = 1;
    _spi.unlock();
}

auto SX126X_LoRaRadio::get_device_variant() -> uint8_t
{
    return LORA_DEVICE;
}

auto SX126X_LoRaRadio::get_fsk_bw_reg_val(const uint32_t bandwidth, const bool locking) -> uint8_t
{
    if(locking) { lock(); }

    uint8_t i;

    for (i = 0; i < (sizeof(fsk_bandwidths) / sizeof(fsk_bw_t)) - 1; i++) {
        if ((bandwidth >= fsk_bandwidths[i].bandwidth)
                && (bandwidth < fsk_bandwidths[i + 1].bandwidth)) {
            if(locking) { unlock(); }
            return fsk_bandwidths[i].register_value;
        }
    }
    // ERROR: Value not found
    // This should never happen
    while (1);
}

void SX126X_LoRaRadio::set_max_payload_length(const radio_modems_t modem, const uint8_t max, 
                                                const bool locking)
{
    if(locking) { lock(); }

    if (modem == MODEM_LORA) {
        _packet_params.params.lora.payload_length = max;
    } else {
        _packet_params.params.gfsk.payload_length = max;
    }

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_tx_config_pocsag(const int8_t power, const bool locking)
{
    if(locking) { lock(); }

    _mod_params.modem_type = MODEM_FSK;
    _mod_params.params.gfsk.bit_rate = 1200;

    _mod_params.params.gfsk.modulation_shaping = MOD_SHAPING_OFF;
    _mod_params.params.gfsk.bandwidth = get_fsk_bw_reg_val(9000);
    _mod_params.params.gfsk.fdev = 4500;

    _packet_params.modem_type = MODEM_FSK;
    _packet_params.params.gfsk.preamble_length = (1 << 3); // convert byte into bit
    _packet_params.params.gfsk.preamble_min_detect = RADIO_PREAMBLE_DETECTOR_OFF;
    _packet_params.params.gfsk.syncword_length = 0 << 3; // convert byte into bit
    _packet_params.params.gfsk.addr_comp = RADIO_ADDRESSCOMP_FILT_OFF;
    _packet_params.params.gfsk.header_type = RADIO_PACKET_FIXED_LENGTH;

    _packet_params.params.gfsk.crc_length = RADIO_CRC_OFF;
    _packet_params.params.gfsk.whitening_mode = RADIO_DC_FREE_OFF;

    set_modem(MODEM_FSK);

    write_to_register(REG_LR_SYNCWORDBASEADDRESS, (uint8_t *) sync_word, 8);
    set_whitening_seed(0x01FF);

    _tx_power = power;
    _tx_timeout = 0;

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_tx_config(const radio_modems_t modem,
                                     const int8_t power,
                                     const uint32_t fdev,
                                     const uint32_t bandwidth,
                                     const uint32_t datarate,
                                     const uint8_t coderate,
                                     const uint16_t preamble_len,
                                     const bool fix_len,
                                     const bool crc_on,
                                     const bool  /*freq_hop_on*/,
                                     const uint8_t  /*hop_period*/,
                                     const bool iq_inverted,
                                     const uint32_t timeout,
                                     const bool locking)
{
    if(locking) { lock(); }

    auto modem_type = static_cast<uint8_t>(modem);
    switch (modem_type) {
        case MODEM_FSK:
            _mod_params.modem_type = MODEM_FSK;
            _mod_params.params.gfsk.bit_rate = datarate;

           _mod_params.params.gfsk.modulation_shaping = MOD_SHAPING_G_BT_1;
           _mod_params.params.gfsk.bandwidth = get_fsk_bw_reg_val(bandwidth);
           _mod_params.params.gfsk.fdev = fdev;

            _packet_params.modem_type = MODEM_FSK;
            _packet_params.params.gfsk.preamble_length = (preamble_len << 3); // convert byte into bit
            _packet_params.params.gfsk.preamble_min_detect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            _packet_params.params.gfsk.syncword_length = 3 << 3; // convert byte into bit
            _packet_params.params.gfsk.addr_comp = RADIO_ADDRESSCOMP_FILT_OFF;
            _packet_params.params.gfsk.header_type = (fix_len == true) ?
                            RADIO_PACKET_FIXED_LENGTH :
                            RADIO_PACKET_VARIABLE_LENGTH;

            if (crc_on) {
                _packet_params.params.gfsk.crc_length = RADIO_CRC_2_BYTES_CCIT;
            } else {
                _packet_params.params.gfsk.crc_length = RADIO_CRC_OFF;
            }
            _packet_params.params.gfsk.whitening_mode = RADIO_DC_FREEWHITENING;

            set_modem(MODEM_FSK);

            write_to_register(REG_LR_SYNCWORDBASEADDRESS, (uint8_t *) sync_word, 8);
            set_whitening_seed(0x01FF);
            break;

        case MODEM_LORA:
            _mod_params.modem_type = MODEM_LORA;
            _mod_params.params.lora.spreading_factor = static_cast<lora_spread_factors_t>(datarate);
            _mod_params.params.lora.bandwidth = static_cast<lora_bandwidths_t>(lora_bandwidhts[bandwidth]);
            _mod_params.params.lora.coding_rate = static_cast<lora_coding_tates_t>(coderate);

            if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12)))
                    || ((bandwidth == 1) && (datarate == 12))) {
                _mod_params.params.lora.low_datarate_optimization = 0x01;
            } else {
                _mod_params.params.lora.low_datarate_optimization = 0x00;
            }

           _packet_params.modem_type = MODEM_LORA;

            if ((_mod_params.params.lora.spreading_factor == LORA_SF5)
                    || (_mod_params.params.lora.spreading_factor == LORA_SF6)) {
                if (preamble_len < 12) {
                    _packet_params.params.lora.preamble_length = 12;
                } else {
                    _packet_params.params.lora.preamble_length = preamble_len;
                }
            } else {
                _packet_params.params.lora.preamble_length = preamble_len;
            }

            _packet_params.params.lora.header_type = static_cast<lora_pkt_length_t>(fix_len);
            _packet_params.params.lora.crc_mode = static_cast<lora_crc_mode_t>(crc_on);
            _packet_params.params.lora.invert_IQ = static_cast<lora_IQ_mode_t>(iq_inverted);

            set_modem(MODEM_LORA);

            break;
    }

    _tx_power = power;
    _tx_timeout = timeout;

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_rx_config(const radio_modems_t modem,
                                     const uint32_t bandwidth,
                                     const uint32_t datarate,
                                     const uint8_t coderate,
                                     const uint32_t  /*bandwidthAfc*/,
                                     const uint16_t preamble_len,
                                     const uint16_t symb_timeout,
                                     const bool fix_len,
                                     const uint8_t payload_len,
                                     const bool crc_on,
                                     const bool freq_hop_on,
                                     const uint8_t hop_period,
                                     const bool iq_inverted,
                                     const bool rx_continuous,
                                     const bool locking)
{
    if(locking) { unlock(); }
    uint8_t max_payload_len = 0;
    (void) freq_hop_on;
    (void) hop_period;

    if (rx_continuous) {
        _reception_mode = RECEPTION_MODE_CONTINUOUS;
    }  else {
        _reception_mode = RECEPTION_MODE_SINGLE;
    }

    if (fix_len == true) {
        max_payload_len = payload_len;
    } else {
        max_payload_len = 0xFF;
    }

    auto modem_type = static_cast<uint8_t>(modem);

    switch (modem_type) {
        case MODEM_FSK: {
            _mod_params.modem_type = MODEM_FSK;
            _mod_params.params.gfsk.bit_rate = datarate;
            _mod_params.params.gfsk.modulation_shaping = MOD_SHAPING_G_BT_1;
            _mod_params.params.gfsk.bandwidth = get_fsk_bw_reg_val(bandwidth);

            _packet_params.modem_type = MODEM_FSK;
            _packet_params.params.gfsk.preamble_length = (preamble_len << 3); // convert byte into bit
            _packet_params.params.gfsk.preamble_min_detect =
                    RADIO_PREAMBLE_DETECTOR_08_BITS;
            _packet_params.params.gfsk.syncword_length = 3 << 3; // convert byte into bit
            _packet_params.params.gfsk.addr_comp = RADIO_ADDRESSCOMP_FILT_OFF;
            _packet_params.params.gfsk.header_type =
                    (fix_len == true) ?
                            RADIO_PACKET_FIXED_LENGTH :
                            RADIO_PACKET_VARIABLE_LENGTH;
            _packet_params.params.gfsk.payload_length = max_payload_len;

            if (crc_on) {
                _packet_params.params.gfsk.crc_length = RADIO_CRC_2_BYTES_CCIT;
            } else {
                _packet_params.params.gfsk.crc_length = RADIO_CRC_OFF;
            }

            _packet_params.params.gfsk.whitening_mode = RADIO_DC_FREEWHITENING;

            set_modem(MODEM_FSK);

            write_to_register(REG_LR_SYNCWORDBASEADDRESS, (uint8_t *) sync_word, 8);
            set_whitening_seed(0x01FF);

            _rx_timeout = static_cast<uint32_t>(symb_timeout
                    * ((1.0 / static_cast<float>(datarate)) * 8.0) * 1000);

            break;
        }

        case MODEM_LORA: {
            _rx_timeout_in_symbols = symb_timeout;
            _mod_params.modem_type = MODEM_LORA;
            _mod_params.params.lora.spreading_factor =
                    static_cast<lora_spread_factors_t>(datarate);
            _mod_params.params.lora.bandwidth = static_cast<lora_bandwidths_t>(lora_bandwidhts[bandwidth]);
            _mod_params.params.lora.coding_rate =
                    static_cast<lora_coding_tates_t>(coderate);

            if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12)))
                    || ((bandwidth == 1) && (datarate == 12))) {
                _mod_params.params.lora.low_datarate_optimization = 0x01;
            } else {
                _mod_params.params.lora.low_datarate_optimization = 0x00;
            }

            _packet_params.modem_type = MODEM_LORA;

            if ((_mod_params.params.lora.spreading_factor == LORA_SF5)
                    || (_mod_params.params.lora.spreading_factor == LORA_SF6)) {
                if (preamble_len < 12) {
                    _packet_params.params.lora.preamble_length = 12;
                } else {
                    _packet_params.params.lora.preamble_length = preamble_len;
                }
            } else {
                _packet_params.params.lora.preamble_length = preamble_len;
            }

            _packet_params.params.lora.header_type = static_cast<lora_pkt_length_t>(fix_len);
            _packet_params.params.lora.payload_length = max_payload_len;
            _packet_params.params.lora.crc_mode = static_cast<lora_crc_mode_t>(crc_on);
            _packet_params.params.lora.invert_IQ = static_cast<lora_IQ_mode_t>(iq_inverted);

            set_modem(MODEM_LORA);

            if (_reception_mode == RECEPTION_MODE_CONTINUOUS) {
                _rx_timeout = 0xFFFFFFFF;
            } else {
                _rx_timeout = 0x00000000;
            }

            break;
        }

        default:
            break;
    }
    
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::configure_dio_irq(const uint16_t irq_mask, const uint16_t dio1_mask,
                                         const uint16_t dio2_mask, const uint16_t dio3_mask,
                                         const bool locking)
{
    if(locking) { lock(); }
    vector<uint8_t> buf;
    constexpr uint16_t BYTE_MASK = 0x00FF;
    constexpr uint16_t SHIFT_BYTE = 8U;
    buf.push_back(static_cast<uint8_t>((irq_mask >> SHIFT_BYTE) & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>(irq_mask & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>((dio1_mask >> SHIFT_BYTE) & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>(dio1_mask & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>((dio2_mask >> SHIFT_BYTE) & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>(dio2_mask & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>((dio3_mask >> SHIFT_BYTE) & BYTE_MASK));
    buf.push_back(static_cast<uint8_t>(dio3_mask & BYTE_MASK));

    write_opmode_command(static_cast<uint8_t>(RADIO_CFG_DIOIRQ), buf.data(), buf.size());
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::send(const uint8_t *const buffer, const uint8_t size, const bool locking)
{
    if(locking) { lock(); }
    set_tx_power(_tx_power);

    if(_rxen.is_connected() != 0) {
        _rxen = 0;
    }
    if(_txen.is_connected() != 0) {
        _txen = 1;
    }

    configure_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                      IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                      IRQ_RADIO_NONE,
                      IRQ_RADIO_NONE );

    set_modulation_params(&_mod_params);
    _packet_params.params.gfsk.payload_length = size;
    set_packet_params(&_packet_params);

    // 500KHz transmit workaround
    constexpr uint16_t REG_500KHZ = 0x0889;
    if(_mod_params.params.lora.bandwidth == 9) {
        uint8_t val = read_register(REG_500KHZ);
        val &= 0xFEU;
        write_to_register(REG_500KHZ, val);
    } else {
        uint8_t val = read_register(REG_500KHZ);
        val |= 0x04U;
        write_to_register(REG_500KHZ, val);
    }

    write_fifo(buffer, size);
    vector<uint8_t> buf(3);

    // _tx_timeout in ms should be converted to us and then divided by
    // 15.625 us. Check data-sheet 13.1.4 SetTX() section.
    constexpr int MS_IN_S = 1000;
    constexpr float US_PER_UNIT = 15.625F;
    uint32_t timeout_scaled = ceilf((static_cast<float>(_tx_timeout) * MS_IN_S) / US_PER_UNIT);

    constexpr uint8_t BYTE_MASK = 0xFFU;
    constexpr uint32_t TWO_BYTE_SHIFT = 16U;
    constexpr uint32_t ONE_BYTE_SHIFT = 8U;
    buf[0] = static_cast<uint8_t>((timeout_scaled >> TWO_BYTE_SHIFT) & BYTE_MASK);
    buf[1] = static_cast<uint8_t>((timeout_scaled >> ONE_BYTE_SHIFT) & BYTE_MASK);
    buf[2] = static_cast<uint8_t>(timeout_scaled & BYTE_MASK);

    write_opmode_command(RADIO_SET_TX, buf.data(), 3);
    *tx_int_mon = 1;
    *rx_int_mon = 0;

    _operation_mode = MODE_TX;
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::dangle_timeout_handler() {
    #warning Make sure this is the right thing to do
    //CriticalSectionLock lock;
    _chip_select = 1;
    *tx_int_mon = 1;
    *rx_int_mon = 0;
    dangling_flags.set(0x1);
}

void SX126X_LoRaRadio::send_with_delay(const uint8_t *const buffer, const uint8_t size, 
                                    RadioTiming &radio_timing, const bool locking)
{
    if(locking) { lock(); }
    //set_tx_power(_tx_power);

    if(_rxen.is_connected() != 0) {
        _rxen = 0;
    }
    if(_txen.is_connected() != 0) {
        _txen = 1;
    }
   
    configure_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                      IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                      IRQ_RADIO_NONE,
                      IRQ_RADIO_NONE );

    set_modulation_params(&_mod_params);
    set_packet_params(&_packet_params);
    write_fifo(buffer, size);
    vector<uint8_t> buf(3);

    // _tx_timeout in ms should be converted to us and then divided by
    // 15.625 us. Check data-sheet 13.1.4 SetTX() section.
    constexpr int MS_IN_S = 1000;
    constexpr float US_PER_UNIT = 15.625F;
    uint32_t timeout_scaled = ceilf((static_cast<float>(_tx_timeout) * MS_IN_S) / US_PER_UNIT);

    constexpr uint8_t BYTE_MASK = 0xFFU;
    constexpr uint32_t TWO_BYTE_SHIFT = 16U;
    constexpr uint32_t ONE_BYTE_SHIFT = 8U;
    buf[0] = static_cast<uint8_t>((timeout_scaled >> TWO_BYTE_SHIFT) & BYTE_MASK);
    buf[1] = static_cast<uint8_t>((timeout_scaled >> ONE_BYTE_SHIFT) & BYTE_MASK);
    buf[2] = static_cast<uint8_t>(timeout_scaled & BYTE_MASK);

    // A "dangling transaction". Write out the bytes to start the transaction, but
    //  don't raise the SPI select line right away. Instead, set a Timeout, and raise
    //  the line when the Timeout handler gets triggered.
    write_opmode_command_dangling(RADIO_SET_TX, buf.data(), 3);  
    CalTimeout dangle_timeout;
    dangle_timeout.attach_us(callback(this, &SX126X_LoRaRadio::dangle_timeout_handler), 
                                radio_timing.getWaitNoWarn());
    dangling_flags.wait_any(0x1);
    write_opmode_command_finish();
    led3->LEDSolid();
    static constexpr int THREE_SECS_MS = 3000;
    radio->tx_timeout.attach(callback(this, &SX126X_LoRaRadio::tx_timeout_handler), THREE_SECS_MS);
    _operation_mode = MODE_TX;
    if(locking) { unlock(); }
}


void SX126X_LoRaRadio::receive_sel(const bool locking) {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(radio_cb.radio_cfg.frequencies_count > 0);
    if(radio_cb.radio_cfg.frequencies_count == 1) {
        radio->rx_hop_frequency(); 
        #warning check this to see if it's correct
        receive_cad_rx(locking);
    } else {
        radio->rx_hop_frequency(); 
        receive_cad_rx(locking);
    }
}


void SX126X_LoRaRadio::receive(const bool locking)
{
    if(locking) { lock(); }
    //PORTABLE_ASSERT(false);
    if (get_modem() == MODEM_LORA && _reception_mode != RECEPTION_MODE_CONTINUOUS) {
        // Data-sheet Table 13-11: StopOnPreambParam
        // We will use radio's internal timer to mark no reception. This behaviour
        // is different from SX1272/SX1276 where we are relying on radio to stop
        // at preamble detection.
        // 0x00 means Timer will be stopped on SyncWord(FSK) or Header (LoRa) detection
        // 0x01 means Timer is stopped on preamble detection
        uint8_t stop_at_preamble = 0x01;
        write_opmode_command(RADIO_SET_STOPRXTIMERONPREAMBLE, &stop_at_preamble, 1);
        // Data-sheet 13.4.9 SetLoRaSymbNumTimeout
        write_opmode_command(RADIO_SET_LORASYMBTIMEOUT, &_rx_timeout_in_symbols, 1);
    }

    if(_txen.is_connected() != 0) {
        _txen = 0;
    }
    if(_rxen.is_connected() != 0) {
        _rxen = 1;
    }

    if (_reception_mode != RECEPTION_MODE_OTHER) {
        configure_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR, //NOLINT
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR, //NOLINT
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);
        set_modulation_params(&_mod_params);
        set_packet_params(&_packet_params);
    }

    vector<uint8_t> buf(3);
    constexpr uint32_t DEFAULT_RX_GAIN_VALUE = 0x96;
    write_to_register(REG_RX_GAIN, DEFAULT_RX_GAIN_VALUE);
    constexpr uint8_t LOWER_BYTE = 0xFF;
    constexpr uint32_t SHIFT_TWO_BYTES = 16;
    constexpr uint32_t SHIFT_ONE_BYTE = 8;
    buf[0] = static_cast<uint8_t>((_rx_timeout >> SHIFT_TWO_BYTES) & LOWER_BYTE);
    buf[1] = static_cast<uint8_t>((_rx_timeout >> SHIFT_ONE_BYTE) & LOWER_BYTE);
    buf[2] = static_cast<uint8_t>(_rx_timeout & LOWER_BYTE);

    write_opmode_command(RADIO_SET_RX, buf.data(), 3);
    *rx_int_mon = 1;
    *tx_int_mon = 0;

    _operation_mode = MODE_RX;
    if(locking) { unlock(); }
}


void SX126X_LoRaRadio::receive_cad(const bool locking)
{
    if(locking) { lock(); }
    cad_pending.store(true);
    cad_rx_running = false;
    if(_txen.is_connected() != 0) {
        _txen = 0;
    }
    if(_rxen.is_connected() != 0) {
        _rxen = 1;
    }

    configure_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                        IRQ_RADIO_NONE,
                        IRQ_RADIO_NONE);
    set_modulation_params(&_mod_params);
    set_packet_params(&_packet_params);
    lora_spread_factors_t sf = _mod_params.params.lora.spreading_factor;
    lora_bandwidths_t bw = _mod_params.params.lora.bandwidth;
    constexpr int SF_BASE_ADJ = 7;
    lora_cad_params_t my_cad_params = cad_params[bw][sf-SF_BASE_ADJ];
    constexpr int ONE_SYM = 1;
    constexpr int TWO_SYM = 2;
    constexpr int FOUR_SYM = 4;
    constexpr int EIGHT_SYM = 8;
    constexpr int SIXTEEN_SYM = 16;
    lora_cad_symbols_t num_syms = LORA_CAD_01_SYMBOL;
    switch(my_cad_params.num_sym) {
        case ONE_SYM:     num_syms = LORA_CAD_01_SYMBOL; break;
        case TWO_SYM:     num_syms = LORA_CAD_02_SYMBOL; break;
        case FOUR_SYM:    num_syms = LORA_CAD_04_SYMBOL; break;
        case EIGHT_SYM:   num_syms = LORA_CAD_08_SYMBOL; break;
        case SIXTEEN_SYM: num_syms = LORA_CAD_16_SYMBOL; break; 
        default: PORTABLE_ASSERT(false); break;
    }
    set_cad_params(num_syms, my_cad_params.det_max+cad_det_peak_adj[*cur_hop_freq], 
                    my_cad_params.det_min,
                    LORA_CAD_ONLY, cad_rx_timeout);
    constexpr uint8_t RX_GAIN_VAL = 0x96;
    write_to_register(REG_RX_GAIN, RX_GAIN_VAL);
    write_opmode_command(RADIO_SET_CAD, nullptr, 0);
    *rx_int_mon = 1;
    *tx_int_mon = 0;

    _operation_mode = MODE_CAD;
    if(locking) { unlock(); }
}


void SX126X_LoRaRadio::receive_cad_rx(const bool locking)
{
    if(locking) { lock(); }
    cad_pending.store(true);
    cad_rx_running = true;
    if(_txen.is_connected() != 0) {
        _txen = 0;
    }
    if(_rxen.is_connected() != 0) {
        _rxen = 1;
    }
    configure_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                        IRQ_RADIO_NONE,
                        IRQ_RADIO_NONE);
    uint8_t val = 0x01;
    write_opmode_command(RADIO_SET_STOPRXTIMERONPREAMBLE, &val, 1);
    // Data-sheet 13.4.9 SetLoRaSymbNumTimeout
    constexpr uint8_t EIGHT_SYMS = 0x08;
    val = EIGHT_SYMS;
    write_opmode_command(RADIO_SET_LORASYMBTIMEOUT, &val, 1);
    set_modulation_params(&_mod_params);
    set_packet_params(&_packet_params);
    lora_spread_factors_t sf = _mod_params.params.lora.spreading_factor;
    lora_bandwidths_t bw = _mod_params.params.lora.bandwidth;
//    debug_printf(DBG_INFO, "LoRa BW here is %d\r\n", bw);
    constexpr int SF_BASE_ADJ = 7;
    PORTABLE_ASSERT(sf-SF_BASE_ADJ >= 0);
    PORTABLE_ASSERT(sf-SF_BASE_ADJ <= 5);
    lora_cad_params_t my_cad_params = cad_params[0][sf-SF_BASE_ADJ]; // just to keep the linter happy
    switch(bw) {
        case LORA_BW_125: my_cad_params = cad_params[0][sf-SF_BASE_ADJ]; break;
        case LORA_BW_250: my_cad_params = cad_params[1][sf-SF_BASE_ADJ]; break;
        case LORA_BW_500: my_cad_params = cad_params[2][sf-SF_BASE_ADJ]; break;
        default: PORTABLE_ASSERT(false); break;
    }
    lora_cad_symbols_t num_syms = LORA_CAD_01_SYMBOL;
    constexpr int ONE_SYM = 1;
    constexpr int TWO_SYM = 2;
    constexpr int FOUR_SYM = 4;
    constexpr int EIGHT_SYM = 8;
    constexpr int SIXTEEN_SYM = 16;
    switch(my_cad_params.num_sym) {
        case ONE_SYM:     num_syms = LORA_CAD_01_SYMBOL; break;
        case TWO_SYM:     num_syms = LORA_CAD_02_SYMBOL; break;
        case FOUR_SYM:    num_syms = LORA_CAD_04_SYMBOL; break;
        case EIGHT_SYM:   num_syms = LORA_CAD_08_SYMBOL; break;
        case SIXTEEN_SYM: num_syms = LORA_CAD_16_SYMBOL; break; 
        default: PORTABLE_ASSERT(false); break;
    }
    set_cad_params(num_syms, my_cad_params.det_max, 
                    my_cad_params.det_min,
                    LORA_CAD_RX, cad_rx_timeout);
    *rx_int_mon = 1;
    *tx_int_mon = 0;
    write_opmode_command(RADIO_SET_CAD, nullptr, 0);

    _operation_mode = MODE_CAD;
    if(locking) { unlock(); }
}


// check data-sheet 13.1.14.1 PA optimal settings
void SX126X_LoRaRadio::set_tx_power(const int8_t my_power, const bool locking)
{
    if(locking) { lock(); }
    int8_t power = my_power;
    uint8_t buf[2] = {0, 0};

    if (get_device_variant() == SX1261) {
        if (power >= 14) {
            set_pa_config(0x04, 0x00, 0x01, 0x01);
            power = 14;
        } else if (power < 14){
            set_pa_config(0x01, 0x00, 0x01, 0x01);
        }

        if (power < -3) {
            power = -3;
        }
        write_to_register(REG_OCP, 0x18); // current max is 80 mA for the whole device
    } else {
        // sx1262 or sx1268
        if (power > 22) {
            power = 22;
        } else if (power < -3) {
            power = -3;
        }

        if (power <= 14) {
            set_pa_config(0x02, 0x02, 0x00, 0x01);
        } else {
            set_pa_config(0x04, 0x07, 0x00, 0x01);
        }

        write_to_register(REG_OCP, 0x38); // current max 160mA for the whole device
    }

    buf[0] = power;

#ifdef USES_TCXO
    // TCXO
    buf[1] = RADIO_RAMP_200_US;
#else
    // XTAL
    buf[1] = RADIO_RAMP_20_US;
#endif

    write_opmode_command(RADIO_SET_TXPARAMS, buf, 2);
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_modulation_params(const modulation_params_t *const params, const bool locking)
{
    if(locking) { lock(); }
    uint8_t n = 0;
    uint32_t temp = 0;
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if (_active_modem != params->modem_type) {
        set_modem(params->modem_type);
    }

    constexpr uint8_t BYTE_MASK = 0xFFUL;
    constexpr uint32_t SHIFT_TWO_BYTES = 16UL;
    constexpr uint32_t SHIFT_ONE_BYTE = 8UL;
    if(params->modem_type == MODEM_LORA) {
        PORTABLE_ASSERT(params->params.lora.spreading_factor <= 0x0C);
        PORTABLE_ASSERT(params->params.lora.spreading_factor >= 0x05);
        PORTABLE_ASSERT(params->params.lora.bandwidth >= 4);
        PORTABLE_ASSERT(params->params.lora.bandwidth <= 6);
        PORTABLE_ASSERT(params->params.lora.coding_rate >= 0);
        PORTABLE_ASSERT(params->params.lora.coding_rate <= 4);   
        PORTABLE_ASSERT(params->params.lora.low_datarate_optimization == 0 ||
                        params->params.lora.low_datarate_optimization == 1); 
    }
    switch (params->modem_type) {
        case MODEM_FSK:
            n = 8;
            temp = static_cast<uint32_t>(32 * (static_cast<float>(XTAL_FREQ) / static_cast<float>(params->params.gfsk.bit_rate)));
            buf[0] = (temp >> SHIFT_TWO_BYTES) & BYTE_MASK;
            buf[1] = (temp >> SHIFT_ONE_BYTE) & BYTE_MASK;
            buf[2] = temp & BYTE_MASK;
            buf[3] = params->params.gfsk.modulation_shaping;
            buf[4] = params->params.gfsk.bandwidth;
            temp = static_cast<uint32_t>(static_cast<float>(params->params.gfsk.fdev) / static_cast<float>(FREQ_STEP));
            buf[5] = (temp >> SHIFT_TWO_BYTES) & BYTE_MASK;
            buf[6] = (temp >> SHIFT_ONE_BYTE) & BYTE_MASK;
            buf[7] = (temp & BYTE_MASK);
            write_opmode_command(RADIO_SET_MODULATIONPARAMS, buf, n);
            break;

        case MODEM_LORA:
            n = 4;
            buf[0] = params->params.lora.spreading_factor;
            buf[1] = params->params.lora.bandwidth;
            buf[2] = params->params.lora.coding_rate;
            buf[3] = params->params.lora.low_datarate_optimization;

            write_opmode_command(RADIO_SET_MODULATIONPARAMS, buf, n);
            break;

        default:
            return;
    }
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_pa_config(const uint8_t pa_DC, const uint8_t hp_max,
                                     const uint8_t device_type, const uint8_t pa_LUT,
                                     const bool locking)
{
    if(locking) { lock(); }
    uint8_t buf[4] = {0, 0, 0, 0};

    buf[0] = pa_DC;
    buf[1] = hp_max;
    buf[2] = device_type;
    buf[3] = pa_LUT;
    write_opmode_command(RADIO_SET_PACONFIG, buf, 4);
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_crc_seed(const uint16_t seed, const bool locking)
{
    if(locking) { lock(); }
    if (_active_modem == MODEM_FSK) {
        uint8_t buf[2] = {0, 0};
        buf[0] = static_cast<uint8_t>((seed >> 8) & 0xFF);
        buf[1] = static_cast<uint8_t>(seed & 0xFF);
        write_to_register(REG_LR_CRCSEEDBASEADDR, buf, 2);
    }
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_crc_polynomial(const uint16_t polynomial, const bool locking)
{
    if(locking) { lock(); }
    if (_active_modem == MODEM_FSK) {
        uint8_t buf[2] = {0, 0};
        buf[0] = static_cast<uint8_t>((polynomial >> 8) & 0xFF);
        buf[1] = static_cast<uint8_t>(polynomial & 0xFF);
        write_to_register(REG_LR_CRCPOLYBASEADDR, buf, 2);
    }
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_whitening_seed(const uint16_t seed, const bool locking)
{
    if(locking) { lock(); }
    if (_active_modem == MODEM_FSK) {
        uint8_t reg_value = read_register(REG_LR_WHITSEEDBASEADDR_MSB) & 0xFE;
        reg_value = ((seed >> 8) & 0x01) | reg_value;
        write_to_register(REG_LR_WHITSEEDBASEADDR_MSB, reg_value); // only 1 bit.
        write_to_register(REG_LR_WHITSEEDBASEADDR_LSB, static_cast<uint8_t>(seed));
    }
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_packet_params(const packet_params_t *const packet_params, 
                                            const bool locking)
{
    if(locking) { lock(); }
    uint8_t n;
    uint8_t crc_val = 0;
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if (_active_modem != packet_params->modem_type) {
        set_modem(packet_params->modem_type);
    }

    // Inverted IQ workaround
    constexpr uint16_t INVERT_IQ_REG = 0x0736;
    constexpr uint8_t INVERT_IQ_MASK = 0xFBUL;
    if(_active_modem == MODEM_LORA && (packet_params->params.lora.invert_IQ != 0UL)) {
        uint8_t val = read_register(INVERT_IQ_REG);
        val &= INVERT_IQ_MASK;
        write_to_register(INVERT_IQ_REG, val);
    } else {
        uint8_t val = read_register(INVERT_IQ_REG);
        val |= 0x04UL;
        write_to_register(INVERT_IQ_REG, val);
    }

    switch (packet_params->modem_type) {
        case MODEM_FSK:
            if (packet_params->params.gfsk.crc_length == RADIO_CRC_2_BYTES_IBM) {
                set_crc_seed(CRC_IBM_SEED);
                set_crc_polynomial(CRC_POLYNOMIAL_IBM);
                crc_val = RADIO_CRC_2_BYTES;
            } else if (packet_params->params.gfsk.crc_length == RADIO_CRC_2_BYTES_CCIT) {
                set_crc_seed(CRC_CCITT_SEED);
                set_crc_polynomial(CRC_POLYNOMIAL_CCITT);
                crc_val = RADIO_CRC_2_BYTES_INV;
            } else {
                crc_val = packet_params->params.gfsk.crc_length;
            }
            n = 9;
            buf[0] = (packet_params->params.gfsk.preamble_length >> 8) & 0xFF;
            buf[1] = packet_params->params.gfsk.preamble_length;
            buf[2] = packet_params->params.gfsk.preamble_min_detect;
            buf[3] = (packet_params->params.gfsk.syncword_length /*<< 3*/); // convert from byte to bit
            buf[4] = packet_params->params.gfsk.addr_comp;
            buf[5] = packet_params->params.gfsk.header_type;
            buf[6] = packet_params->params.gfsk.payload_length;
            buf[7] = crc_val;
            buf[8] = packet_params->params.gfsk.whitening_mode;
            break;

        case MODEM_LORA:
            n = 6;
            buf[0] = (packet_params->params.lora.preamble_length >> 8) & 0xFF;
            buf[1] = packet_params->params.lora.preamble_length;
            buf[2] = packet_params->params.lora.header_type;
            buf[3] = packet_params->params.lora.payload_length;
            buf[4] = packet_params->params.lora.crc_mode;
            buf[5] = packet_params->params.lora.invert_IQ;
            break;
        default:
            return;
    }
    write_opmode_command(RADIO_SET_PACKETPARAMS, buf, n);
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_cad_params(const lora_cad_symbols_t nb_symbols,
                                      const uint8_t det_peak, const uint8_t det_min,
                                      const cad_exit_modes_t exit_mode,
                                      const uint32_t timeout,
                                      const bool locking)
{
    if(locking) { lock(); }
    uint8_t buf[7] = {0, 0, 0, 0, 0, 0, 0};

    buf[0] = static_cast<uint8_t>(nb_symbols);
    buf[1] = det_peak;
    buf[2] = det_min;
    buf[3] = static_cast<uint8_t>(exit_mode);
    buf[4] = static_cast<uint8_t>((timeout >> 16) & 0xFF);
    buf[5] = static_cast<uint8_t>((timeout >> 8) & 0xFF);
    buf[6] = static_cast<uint8_t>(timeout & 0xFF);
    write_opmode_command(static_cast<uint8_t>(RADIO_SET_CADPARAMS), buf, 7);

    _operation_mode = MODE_CAD;
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::set_buffer_base_addr(const uint8_t tx_base_addr, const uint8_t rx_base_addr,
                                            const bool locking)
{
    if(locking) { lock(); }
    uint8_t buf[2] = {0, 0};

    buf[0] = tx_base_addr;
    buf[1] = rx_base_addr;
    write_opmode_command(static_cast<uint8_t>(RADIO_SET_BUFFERBASEADDRESS), buf, 2);
    if(locking) { unlock(); }
}

uint8_t SX126X_LoRaRadio::get_status(const bool locking)
{
    PORTABLE_ASSERT(false);
    return 0;
}

int8_t SX126X_LoRaRadio::get_rssi(const bool locking)
{
    if(locking) { lock(); }
    uint8_t buf[1] = {0};
    int8_t rssi = 0;

    read_opmode_command(static_cast<uint8_t>(RADIO_GET_RSSIINST), buf, 1);
    rssi = -buf[0] >> 1;

    if(locking) { unlock(); }
    return rssi;
}

void SX126X_LoRaRadio::get_rx_buffer_status(uint8_t *payload_len,
                                            uint8_t *start_ptr,
                                            const bool locking)
{
    if(locking) { lock(); }
    uint8_t status[2] = {0, 0};

    read_opmode_command(static_cast<uint8_t>(RADIO_GET_RXBUFFERSTATUS), status, 2);

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if ((get_modem() == MODEM_LORA) &&
            (read_register(REG_LR_PACKETPARAMS) >> 7 == 1)) {
        *payload_len = read_register(REG_LR_PAYLOADLENGTH);
    } else {
        *payload_len = status[0];
    }

    *start_ptr = status[1];

    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::get_packet_status(packet_status_t *pkt_status, const bool locking)
{
    if(locking) { lock(); }
    uint8_t status[3] = {0, 0, 0};

    read_opmode_command(static_cast<uint8_t>(RADIO_GET_PACKETSTATUS), status, 3);

    pkt_status->modem_type = static_cast<radio_modems_t>(get_modem());
    switch (pkt_status->modem_type) {
        case MODEM_FSK:
            pkt_status->params.gfsk.rx_status = status[0];
            pkt_status->params.gfsk.rssi_sync = -status[1] >> 1;
            pkt_status->params.gfsk.rssi_avg = -status[2] >> 1;
            pkt_status->params.gfsk.freq_error = 0;
            break;

        case MODEM_LORA:
            pkt_status->params.lora.rssi_pkt = -status[0] >> 1;
            // Returns SNR value [dB] rounded to the nearest integer value
            pkt_status->params.lora.snr_pkt = ((static_cast<int8_t>(status[1])) + 2) >> 2;
            pkt_status->params.lora.signal_rssi_pkt = -status[2] >> 1;
            break;

        default:
            // In that specific case, we set everything in the pkt_status to zeros
            // and reset the packet type accordingly
            memset(pkt_status, 0, sizeof(packet_status_t));
            break;
    }

    if(locking) { unlock(); }
}

radio_error_t SX126X_LoRaRadio::get_device_errors(const bool locking)
{
    if(locking) { lock(); }
    radio_error_t error;

    read_opmode_command(static_cast<uint8_t>(RADIO_GET_ERROR), (uint8_t *)&error, 2);

    if(locking) { unlock(); }
    return error;
}

void SX126X_LoRaRadio::clear_device_errors(const bool locking)
{
    if(locking) { lock(); }
    uint8_t buf[2] = {0x00, 0x00};
    write_opmode_command(static_cast<uint8_t>(RADIO_CLR_ERROR), buf, 2);
    if(locking) { unlock(); }
}

void SX126X_LoRaRadio::cad_process_detect(const bool is_false_det, const uint32_t freq) {
    list<int> &vals = cad_det_succ[freq];
    if(is_false_det) {
        vals.push_back(0);
    } else {
        vals.push_back(1);
    }
    constexpr int MAX_NUM_VALS = 200;
    // Keep the list no bigger than 200 entries
    while(vals.size() > MAX_NUM_VALS) {
        vals.pop_front();
    }
    if(vals.size() == MAX_NUM_VALS) {
        constexpr float MIN_DET_SUCC_RATE = 0.95F;
        float cad_det_succ_rate = static_cast<float>(accumulate(vals.begin(), vals.end(), 0))/static_cast<float>(vals.size());
        if(cad_det_succ_rate < MIN_DET_SUCC_RATE) {
            cad_det_peak_adj[freq] += 1;
            cad_det_min_adj[freq] -= 0;
            debug_printf(DBG_WARN, "cad_det_peak_adj %d is + now %d\r\n", freq, cad_det_peak_adj[freq]);
            vals.clear();
        } 
        constexpr int MAX_PEAK_ADJ = 10;
        if(cad_det_peak_adj[freq] < -MAX_PEAK_ADJ) {
            cad_det_peak_adj[freq] = -MAX_PEAK_ADJ;
        }
        if(cad_det_peak_adj[freq] > MAX_PEAK_ADJ) {
            cad_det_peak_adj[freq] = MAX_PEAK_ADJ;
        }            
    }
}


