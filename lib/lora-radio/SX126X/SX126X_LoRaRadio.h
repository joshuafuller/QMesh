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

#ifndef MBED_LORA_RADIO_DRV_SX126X_LORARADIO_H_
#define MBED_LORA_RADIO_DRV_SX126X_LORARADIO_H_

#include "mbed_critical.h"
#include "PinNames.h"
#include "InterruptIn.h"
#include "DigitalOut.h"
#include "DigitalInOut.h"
#include "DigitalIn.h"
#include "AnalogIn.h"
#include "SPI.h"
#include "platform/PlatformMutex.h"
#ifdef MBED_CONF_RTOS_PRESENT
#include "rtos/Thread.h"
#include "rtos/ThisThread.h"
#endif
#include "sx126x_ds.h"
#include "lorawan/LoRaRadio.h"
#include "mbed.h"
#include "radio_timing.hpp"
#include "params.hpp"
#include "cal_timer.hpp"
#include <atomic>
#include <list>
#include <map>
#include <random>
#include "anti_interference.hpp"

#ifdef MBED_CONF_SX126X_LORA_DRIVER_BUFFER_SIZE
#define MAX_DATA_BUFFER_SIZE_SX126X                        MBED_CONF_SX126X_LORA_DRIVER_BUFFER_SIZE
#else
#define MAX_DATA_BUFFER_SIZE_SX126X                        255
#endif

class SX126X_LoRaRadio : public LoRaRadio {

public:
    SX126X_LoRaRadio(PinName mosi,
                       PinName miso,
                       PinName sclk,
                       PinName nss,
                       PinName rxen,
                       PinName txen,
                       PinName reset,
                       PinName dio1,
                       PinName dio2,
                       PinName nrst,
                       PinName busy,
                       PinName pwrctl);

    virtual ~SX126X_LoRaRadio();

    /**
     * Registers radio events with the Mbed LoRaWAN stack and
     * undergoes initialization steps if any
     *
     *  @param events Structure containing the driver callback functions
     */
     void init_radio(radio_events_t *events, bool locking = false) override;

    /**
     * Resets the radio module
     */
     void radio_reset(bool locking = false) override;

    /**
     *  Put the RF module in sleep mode
     */
     void sleep(bool locking = false) override;

    /**
     *  Sets the radio in standby mode
     */
     void standby(bool locking = false) override;

    /**
     *  Sets the reception parameters
     *
     *  @param modem         Radio modem to be used [0: FSK, 1: LoRa]
     *  @param bandwidth     Sets the bandwidth
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     *  @param datarate      Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     *  @param coderate      Sets the coding rate ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     *  @param bandwidth_afc Sets the AFC Bandwidth ( FSK only )
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: N/A ( set to 0 )
     *  @param preamble_len  Sets the Preamble length ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Length in symbols ( the hardware adds 4 more symbols )
     *  @param symb_timeout  Sets the RxSingle timeout value
     *                          FSK : timeout number of bytes
     *                          LoRa: timeout in symbols
     *  @param fixLen        Fixed length packets [0: variable, 1: fixed]
     *  @param payload_len   Sets payload length when fixed lenght is used
     *  @param crc_on        Enables/Disables the CRC [0: OFF, 1: ON]
     *  @param freq_hop_on   Enables disables the intra-packet frequency hopping  [0: OFF, 1: ON] (LoRa only)
     *  @param hop_period    Number of symbols bewteen each hop (LoRa only)
     *  @param iq_inverted   Inverts IQ signals ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     *  @param rx_continuous Sets the reception in continuous mode
     *                          [false: single mode, true: continuous mode]
     */
     void set_rx_config (radio_modems_t modem, uint32_t bandwidth,
                               uint32_t datarate, uint8_t coderate,
                               uint32_t bandwidth_afc, uint16_t preamble_len,
                               uint16_t symb_timeout, bool fix_len,
                               uint8_t payload_len,
                               bool crc_on, bool freq_hop_on, uint8_t hop_period,
                               bool iq_inverted, bool rx_continuous,
                               bool locking = false) override;

    /**
     *  Sets the transmission parameters
     *
     *  @param modem         Radio modem to be used [0: FSK, 1: LoRa]
     *  @param power         Sets the output power [dBm]
     *  @param fdev          Sets the frequency deviation ( FSK only )
     *                          FSK : [Hz]
     *                          LoRa: 0
     *  @param bandwidth     Sets the bandwidth ( LoRa only )
     *                          FSK : 0
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     *  @param datarate      Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     *  @param coderate      Sets the coding rate ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     *  @param preamble_len  Sets the preamble length
     *  @param fix_len       Fixed length packets [0: variable, 1: fixed]
     *  @param crc_on        Enables disables the CRC [0: OFF, 1: ON]
     *  @param freq_hop_on   Enables disables the intra-packet frequency hopping  [0: OFF, 1: ON] (LoRa only)
     *  @param hop_period    Number of symbols bewteen each hop (LoRa only)
     *  @param iq_inverted   Inverts IQ signals ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     *  @param timeout       Transmission timeout [ms]
     */
     void set_tx_config(radio_modems_t modem, 
                            int8_t power, 
                            uint32_t fdev,
                            uint32_t bandwidth, 
                            uint32_t datarate,
                            uint8_t coderate, 
                            uint16_t preamble_len,
                            bool fix_len, 
                            bool crc_on, 
                            bool freq_hop_on,
                            uint8_t hop_period, 
                            bool iq_inverted, 
                            uint32_t timeout,
                            bool locking = false) override;
    
    /*
     * Sets the transmission parameters for 1200bps POCSAG transmissions.
     * @param power transmit power, in dBm.
     */
    virtual void set_tx_config_pocsag(int8_t power, bool locking = false);

    /**
     *  Sends the buffer of size
     *
     *  Prepares the packet to be sent and sets the radio in transmission
     *
     *  @param buffer        Buffer pointer
     *  @param size          Buffer size
     */
    void send(const uint8_t *buffer, uint8_t size, bool locking = false);
    virtual void send_with_delay(const uint8_t *buffer, uint8_t size, 
                                    RadioTiming &radio_timing,
                                    bool locking = false);

    /**
     * Sets the radio to receive
     *
     * All necessary configuration options for reception are set in
     * 'set_rx_config(parameters)' API.
     */
     void receive(bool locking = false) override;

    /**
     *  Sets the carrier frequency
     *
     *  @param freq          Channel RF frequency
     */
     void set_channel(uint32_t freq, bool locking = false) override;

    /**
     *  Generates a 32 bits random value based on the RSSI readings
     *
     *  Remark this function sets the radio in LoRa modem mode and disables
     *         all interrupts.
     *         After calling this function either Radio.SetRxConfig or
     *         Radio.SetTxConfig functions must be called.
     *
     *  @return             32 bits random value
     */
     auto random(bool locking = false) -> uint32_t override;

    /**
     *  Get radio status
     *
     *  @param status        Radio status [RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
     *  @return              Return current radio status
     */
     auto get_status(bool locking = false) -> uint8_t override;

    /**
     *  Sets the maximum payload length
     *
     *  @param modem         Radio modem to be used [0: FSK, 1: LoRa]
     *  @param max           Maximum payload length in bytes
     */
     void set_max_payload_length(radio_modems_t modem, uint8_t max, bool locking = false) override;

    /**
     *  Sets the network to public or private
     *
     *  Updates the sync byte. Applies to LoRa modem only
     *
     *  @param enable        if true, it enables a public network
     */
     void set_public_network(bool enable, bool locking = false) override;

    /**
     *  Computes the packet time on air for the given payload
     *
     *  Remark can only be called once SetRxConfig or SetTxConfig have been called
     *
     *  @param modem         Radio modem to be used [0: FSK, 1: LoRa]
     *  @param pkt_len       Packet payload length
     *  @return              Computed airTime for the given packet payload length
     */
     auto time_on_air(radio_modems_t modem, uint8_t pkt_len) -> uint32_t override;

    /**
     * Perform carrier sensing
     *
     * Checks for a certain time if the RSSI is above a given threshold.
     * This threshold determines if there is already a transmission going on
     * in the channel or not.
     *
     * @param modem                     Type of the radio modem
     * @param freq                      Carrier frequency
     * @param rssi_threshold            Threshold value of RSSI
     * @param max_carrier_sense_time    time to sense the channel
     *
     * @return                          true if there is no active transmission
     *                                  in the channel, false otherwise
     */
     auto perform_carrier_sense(radio_modems_t modem,
                                    uint32_t freq,
                                    int16_t rssi_threshold,
                                    uint32_t max_carrier_sense_time,
                                    bool locking = false) -> bool override;

    /**
     *  Sets the radio in CAD mode
     *
     */
    void start_cad(bool locking = false) override;


    void configure_freq_hop(AntiInterference *my_anti_inter);


    void configure_freq_hop_freqs(vector<uint32_t>& my_hop_freqs);


    /**
     *  Check if the given RF is in range
     *
     *  @param frequency       frequency needed to be checked
     */
     auto check_rf_frequency(uint32_t frequency) -> bool override;

    /** Sets the radio in continuous wave transmission mode
     *
     *  @param freq          Channel RF frequency
     *  @param power         Sets the output power [dBm]
     *  @param time          Transmission mode timeout [s]
     */
     void set_tx_continuous_wave(uint32_t freq, int8_t power, uint16_t time, bool locking = false) override;

    /** Sets the radio in continuous preamble transmission mode */
    virtual void set_tx_continuous_preamble(bool locking = false);

    /**
     * Acquire exclusive access
     */
     void lock() override;

    /**
     * Release exclusive access
     */
     void unlock() override;

    void set_tx_power(int8_t power, bool locking = false);

    void receive_cad(bool locking = false);
    void receive_cad_rx(bool locking = false);
    void receive_sel(bool locking = false);

    void tx_hop_frequency(bool locking = false);

    void rx_hop_frequency();

    /**
     * Timers for tracking when the Rx interrupt was thrown
     */
    Semaphore *tmr_sem_ptr;
    CalTimer *cur_tmr;
    shared_ptr<CalTimer> cur_tmr_sptr;
    uint32_t cad_rx_timeout;
    atomic<bool> cad_pending, stop_cad;

private:

    // SPI and chip select control
    mbed::SPI _spi;
    mbed::DigitalOut _chip_select;

    // module rest control
    mbed::DigitalInOut _reset_ctl;

    // module power control
    mbed::DigitalOut _pwr_ctl;

    // Interrupt controls
    mbed::InterruptIn _dio1_ctl;;

    // module busy control
    mbed::DigitalIn _busy;

    // RF switch controls, for the CDEBytes E22
    mbed::DigitalOut _rxen;
    mbed::DigitalOut _txen;

    // Structure containing function pointers to the stack callbacks
    radio_events_t *_radio_events;

    // Data buffer used for both TX and RX
    // Size of this buffer is configurable via Mbed config system
    // Default is 255 bytes
    uint8_t _data_buffer[MAX_DATA_BUFFER_SIZE_SX126X];

#ifdef MBED_CONF_RTOS_PRESENT
    // Thread to handle interrupts
    rtos::Thread irq_thread;
#endif

    // Access protection
    PlatformMutex mutex;

    // helper functions
    void wakeup(bool locking = false);
    void read_opmode_command(uint8_t cmd, uint8_t *buffer, uint16_t size);
    /**
     * Write out a command to the SX126X.
     *  @param cmd -- the SX126X command to be written
     *  @param buffer -- pointer to the buffer containing the command's parameters.
     *  @param size -- the size of the rest of the SX126X command.
     */
    void write_opmode_command(uint8_t cmd, uint8_t *buffer, uint16_t size);
    /**
     * Write out a "dangling command" to the SX126X as a way to precisely control
     *  the timing of certain radio commands. Basically, this function writes out
     *  an SX126X command, but does not immediately raise the SPI select line, but
     *  instead sets a Timeout for when retransmission should occur based on when
     *  it is time to retransmit. The handler then raises the line once it's 
     *  triggered. 
     *  @param cmd -- the SX126X command to be written
     *  @param buffer -- pointer to the buffer containing the command's parameters.
     *  @param size -- the size of the rest of the SX126X command.
     */
    void write_opmode_command_dangling(uint8_t cmd, uint8_t *buffer, uint16_t size);
    /** 
     * Unlocks the SPI bus after the "dangling command" finishes. Has to be a separate
     *  function because the SPI lock/unlock functions cannot be called from and ISR
     *  context.
     */
    void write_opmode_command_finish();
    /**
     * Interrupt handler to raise the SPI select select line once the Timeout is 
     *  triggered.
     */
    void dangle_timeout_handler();
    void set_dio2_as_rfswitch_ctrl(uint8_t enable, bool locking = false);
    void set_dio3_as_tcxo_ctrl(radio_TCXO_ctrl_voltage_t voltage, uint32_t timeout,
                                bool locking = false);
    auto get_device_variant() -> uint8_t;
    void set_device_ready();
    auto get_rssi(bool locking = false) -> int8_t;
    auto get_fsk_bw_reg_val(uint32_t bandwidth, bool locking = false) -> uint8_t;
    void write_to_register(uint16_t addr, uint8_t data);
    void write_to_register(uint16_t addr, uint8_t *data, uint8_t size);
    auto read_register(uint16_t addr) -> uint8_t;
    void read_register(uint16_t addr, uint8_t *buffer, uint8_t size);
    void write_fifo(const uint8_t * buffer, uint8_t size, bool locking = false);
    void read_fifo(uint8_t *buffer, uint8_t size, uint8_t offset);
    void rf_irq_task();
    void set_modem(uint8_t modem, bool locking = false);
    auto get_modem() -> uint8_t;
    auto get_irq_status() -> uint16_t;
    auto get_frequency_support() -> uint8_t;
    void start_read_rssi();
    void stop_read_rssi();

    // ISR
    void dio1_irq_isr();

    // Handler called by thread in response to signal
    void handle_dio1_irq();

    void set_modulation_params(const modulation_params_t *const modulationParams, bool locking = false);
    void set_packet_params(const packet_params_t *packet_params, bool locking = false);
    void set_cad_params(lora_cad_symbols_t nb_symbols, uint8_t det_peak,
                        uint8_t det_min, cad_exit_modes_t exit_mode,
                        uint32_t timeout, bool locking = false);
    void set_buffer_base_addr(uint8_t tx_base_addr, uint8_t rx_base_addr, bool locking = false);
    void get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_buffer_ptr, bool locking = false);
    void get_packet_status(packet_status_t *pkt_status, bool locking = false);
    auto get_device_errors(bool locking = false) -> radio_error_t;
    void clear_device_errors(bool locking = false);
    void clear_irq_status(uint16_t irq);
    void set_crc_seed(uint16_t seed, bool locking = false);
    void set_crc_polynomial(uint16_t polynomial, bool locking = false);
    void set_whitening_seed(uint16_t seed, bool locking = false);
    void set_pa_config( uint8_t pa_DC, uint8_t hp_max, uint8_t device_type,
                        uint8_t pa_LUT, bool locking = false );
    void calibrate_image(uint32_t freq, bool locking = false);
    void configure_dio_irq(uint16_t irq_mask, uint16_t dio1_mask,
                           uint16_t dio2_mask, uint16_t dio3_mask,
                           bool locking = false);
    void cold_start_wakeup(bool locking = false);
    void read_rssi_thread_fn();

    uint8_t _active_modem;
    uint8_t _standby_mode;
    uint8_t _operation_mode;
    uint8_t _reception_mode;
    uint32_t _tx_timeout;
    uint32_t _rx_timeout;
    uint8_t _rx_timeout_in_symbols;
    int8_t _tx_power;
    bool _image_calibrated;
    bool _force_image_calibration;
    bool _network_mode_public;

    // Structure containing all user and network specified settings
    // for radio module
    modulation_params_t _mod_params;
    packet_params_t _packet_params;

    EventFlags dangling_flags;

    //atomic<bool> stop_cad;
    EventFlags cad_running;
    bool cad_rx_running;
    map<uint32_t, int8_t> cad_det_peak_adj;
    map<uint32_t, int8_t> cad_det_min_adj;
    map<uint32_t, list<int>> cad_det_succ;
    map<uint32_t, list<int>> cad_rx_succ;

    void cad_process_detect(bool is_false_det, uint32_t freq); 

    // Components to poll the RSSI to implement soft decoding
    atomic<bool> collect_rssi;
    Thread soft_dec_thread;
    shared_ptr<list<pair<uint32_t, uint8_t> > > rssi_list_sptr;

    // Components to implement frequency hopping
    AntiInterference *anti_inter;
    
    vector<uint32_t> hop_freqs;
    vector<uint32_t>::iterator cur_hop_freq;

    // Timeout function to reset modem if it hangs on a transmit
    Timeout tx_timeout;
    void tx_timeout_handler();
};

using lora_cad_params_t = struct {
    uint8_t num_sym;
    uint8_t det_min;
    uint8_t det_max;
};
const array<array<lora_cad_params_t, 6>, 3> cad_params = { //NOLINT 
    array<lora_cad_params_t, 6>{{{2, 10, 22}, {2, 10, 22}, {4, 10, 23}, {4, 10, 24}, {4, 10, 25}, {4, 10, 28}}},  // 125KHz -- from appnote
    array<lora_cad_params_t, 6>{{{4, 10, 21}, {4, 10, 22}, {4, 10, 22}, {4, 10, 23}, {4, 10, 25}, {4, 10, 28}}},  // 250KHz -- interpolated
    array<lora_cad_params_t, 6>{{{4, 10, 21}, {4, 10, 22}, {4, 10, 22}, {4, 10, 23}, {4, 10, 25}, {8, 10, 29}}}}; // 500KHz -- from appnote

#endif /* MBED_LORA_RADIO_DRV_SX126X_LORARADIO_H_ */
