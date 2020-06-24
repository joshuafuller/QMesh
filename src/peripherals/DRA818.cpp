#include "mbed.h"
#include "peripherals.hpp"


DRA818::DRA818(PinName my_uart_tx_pin_name, PinName my_uart_rx_pin_name, 
        PinName my_sleep_pin_name, PinName my_ptt_pin_name,
        PinName my_rf_pwr_pin_name, PinName my_sq_pin_name) {

    evt_queue = mbed_event_queue();

    // Set up the various signals for the DRA818 module
    uart_tx_pin_name = my_uart_tx_pin_name;
    uart_rx_pin_name = my_uart_rx_pin_name;
    sleep_pin_name = my_sleep_pin_name;
    ptt_pin_name = my_ptt_pin_name;
    rf_pwr_pin_name = my_rf_pwr_pin_name;
    sq_pin_name = my_sq_pin_name;
    
    sleep_pin_sptr = make_shared<DigitalOut>(sleep_pin_name);
    sleep_pin_sptr->write(1);
    ptt_pin_sptr = make_shared<DigitalOut>(ptt_pin_name);
    ptt_pin_sptr->write(0);
    rf_pwr_pin_sptr = make_shared<DigitalInOut>(rf_pwr_pin_name);
    rf_pwr_pin_sptr->write(1);
    sq_pin_sptr = make_shared<InterruptIn>(sq_pin_name);

    // wake() just sets up the UART
    wake();

    // Set up the squelch pin handler
    sq_pin_sptr->fall(evt_queue->event(callback(this, &DRA818::squelch_handler)));
}


void DRA818::squelch_handler(void) {
    debug_printf(DBG_INFO, "Squelch detected!\r\n");
}


DRA818::~DRA818() {
    delete ser;
    sleep_pin_sptr->write(0);
}


void DRA818::sleep(void) {
    delete ser;
    sleep_pin_sptr->write(0);
    sleeping = true;
}


void DRA818::setPower(bool high_power) {
    rf_pwr_pin_sptr->output();
    rf_pwr_pin_sptr->mode(OpenDrain);
    rf_pwr_pin_sptr->write(high_power ? 0 : 1);
}


void DRA818::tx_en(void) {
    ptt_pin_sptr->write(1);
}
    

void DRA818::tx_dis(void) {
    ptt_pin_sptr->write(0);
} 


void DRA818::wake(void) {
    if(sleeping) {
        ser = new UARTSerial(uart_tx_pin_name, uart_rx_pin_name, 9600);
        ser_fh = fdopen(ser, "rw");
        sleep_pin_sptr->write(1);
        sleeping = false;
    }
}


bool DRA818::handshake(void) {
    fprintf(ser_fh, "AT+DMOCONNECT \n\r");\
    debug_printf(DBG_INFO, "Attempting to handshake with DRA818...\r\n");
    char ret_str[30];
    fgets(ret_str, sizeof(ret_str), ser_fh);
    debug_printf(DBG_INFO, "Received %s from module\r\n", ret_str);
    return true;
}


void DRA818::setFrequency(const float tx_freq, const float rx_freq) {
    debug_printf(DBG_INFO, "Setting DRA818 Tx/Rx frequency to %f/%f...\r\n");
    fprintf(ser_fh, "AT+DMOSETGROUP=0,%f,%f,0000,4,0000 \n\r", tx_freq, rx_freq);
    char ret_str[30];
    fgets(ret_str, sizeof(ret_str), ser_fh);
    debug_printf(DBG_INFO, "Received %s from module\r\n", ret_str);
}


