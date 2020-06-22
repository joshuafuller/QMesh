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
    DRA818(PinName my_uart_tx_pin_name, PinName my_uart_rx_pin_name, 
            PinName my_sleep_pin_name, PinName my_ptt_pin_name,
            PinName my_rf_pwr_pin_name, PinName my_sq_pin_name);

    ~DRA818();

    void sleep(void);

    void setPower(bool high_power);

    void tx_en(void);

    void tx_dis(void);

    void wake(void);

    bool handshake(void);

    void squelch_handler(void);

    void setFrequency(const float tx_freq, const float rx_freq);
};


#endif /* DRA818_HPP */