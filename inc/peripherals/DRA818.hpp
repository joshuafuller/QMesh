#ifndef DRA818_HPP
#define DRA818_HPP

#include "mbed.h"

class DRA818 {
private:
    PinName uart_tx_pin_name, uart_rx_pin_name;
    PinName sleep_pin_name, ptt_pin_name, rf_pwr_pin_name, sq_pin_name;
    shared_ptr<DigitalOut> sleep_pin_sptr, ptt_pin_sptr;
    shared_ptr<DigitalInOut> rf_pwr_pin_sptr;
    shared_ptr<InterruptIn> sq_pin_sptr;
    UARTSerial *ser;
    FILE *ser_fh;
    bool sleeping;
    EventQueue *evt_queue;

public:
    /**
     * Constuctor. Needs a UART to communicate with the module.
     * @param my_uart_tx_pin_name Pin name of the TX pin of the UART
     * @param my_uart_rx_pin_name Pin name of the RX pin of the UART
     * @param my_sleep_pin_name GPIO output pin for the sleep/wake control
     * @param my_ptt_pin_name GPIO output pin for the PTT control
     * @param my_rf_pwr_pin_name GPIO output pin for the power control
     * @param my_sq_pin_name GPIO input output pin for the squelch pin
     */
    DRA818(PinName my_uart_tx_pin_name, PinName my_uart_rx_pin_name, 
            PinName my_sleep_pin_name, PinName my_ptt_pin_name,
            PinName my_rf_pwr_pin_name, PinName my_sq_pin_name);

    /// Destructor.
    ~DRA818();

    /// Puts the module to sleep. Also deletes the UARTSerial object
    ///  so that the STM32 can enter deep sleep.
    void sleep(void);

    /// Sets high (~1W) or low (~0.5W) Tx power for the module.
    /// @param high_power true for 1W, false for 0.5W.
    void setPower(const bool high_power);

    /// Enable transmit via a the PTT signal.
    void tx_en(void);

    /// Stop transmitting via the PTT signal.
    void tx_dis(void);

    /// Take the module out of sleep.
    void wake(void);

    /// Perform a handshake command with the module.
    bool handshake(void);

    /// Interrupt handler that gets triggered when squelch gets broken.
    void squelch_handler(void);

    /// Set the transmit and receive frequencies.
    /// @param tx_freq The desired transmit frequency
    /// @param rx_freq The desired receive frequency
    void setFrequency(const float tx_freq, const float rx_freq);
};


#endif /* DRA818_HPP */