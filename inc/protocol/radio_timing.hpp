#ifndef RADIO_TIMING_HPP
#define RADIO_TIMING_HPP

#include "mbed.h"

extern DigitalOut rx_int_mon, tx_int_mon, int_trig_mon;

const float lora_bw[] = {125e3f, 250e3f, 500e3f};

/**
 * A synchronized flooded mesh has all sort of timing-related things it needs to 
 * handle. This class handles all of these different timing things. Specifically, it
 * handles:
 * - Calculating LoRa timing values, such as symbol duration, preamble duration, 
 * payload duration, etc.
 * - Starting and stopping the fine-grained (millisecond or less) timer(s) necessary
 * to make sure transmissions are properly synchronized.
 * - Waiting the precise amount of time before e.g. retransmitting.
 */
// Class that stores the different radio timing values.
class RadioTiming {
public:
    float sym_time_s;
    float pre_time_s;
    float pld_time_s;
    float pkt_time_s;
    float sym_frac_s;

    uint32_t sym_time_ms;
    uint32_t pre_time_ms;
    uint32_t pld_time_ms;
    uint32_t pkt_time_ms;
    uint32_t sym_frac_ms;

    uint32_t sym_time_us;
    uint32_t pre_time_us;
    uint32_t pld_time_us;
    uint32_t pkt_time_us;
    uint32_t sym_frac_us;
    
    uint32_t n_sym_pre;
    uint32_t n_sym_pld;
    uint32_t n_sym_pkt;

    uint32_t sym_wait_factor;
    uint32_t sym_wait_us;
    uint32_t pre_wait_factor;
    uint32_t pre_wait_us;

    shared_ptr<LowPowerTimer> tmr_sptr;
    int32_t wait_duration_us;

/**
 * Given the various LoRa parameters, pre-compute all of the different timing
 * parameters.
 * @param bw LoRa bandwidth.
 * @param sf LoRa spreading factor.
 * @param cr LoRa coding rate.
 * @param n_pre_sym Number of preamble symbols.
 * @param n_pld_bytes Number of payload bytes.
 */
void computeTimes(const uint32_t bw, const uint8_t sf, const uint8_t cr, 
        const uint32_t n_pre_sym, uint8_t n_pld_bytes);

/**
 * Wait for a number of transmit slots. Note that this function uses the timer's
 * start time as the basis for how long to wait. Since the timer is typically
 * started right after a LoRa packet is received, this function allows us to 
 * prepare a packet for retransmission, wait the remainder of one timeslot, and
 * then retransmit it.
 * @param num_slots Number of slots to wait.
 */
void waitFullSlots(const size_t num_slots);


/**
 * Wait for a fraction of a symbol. 
 * @param symb_frac The fraction of the symbol, in eigths of half a symbol
 *  duration, to wait.
 * @papram direction should be either 1.0f or -1.0f, used to set the direction
 */
void waitSymOffset(const uint8_t symb_frac, const float direction);

/**
 * Wait for the "remaining time" that includes various timing offsets
 * used to implement QMesh's collision resistance:
 * - Number of Preamble durations to wait
 * - Intra-symbol delay fraction
 * @param sym_wait How many eighths of a symbol to delay
 * @param pre_wait Number of preambles to wait
 */
void waitRxRemainder(const size_t sym_wait, const size_t pre_wait);

/**
 * Calculate a random intra-symbol wait duration. Chooses between [0,7]
 * eights of the symbol duration. Stores the result internally within
 * the object.
 */
void calcWaitSymbol(void);

/**
 * Calculate the random preamble-length wait period. Chooses between [0,3]
 * different preamble offsets.
 */
void calcWaitPreamble(void);

/**
 * Wait for the symbol and preamble offset durations.
 */
void waitTx(void);

/** 
 * Starts the protocol timer.
 */
void setTimer(shared_ptr<LowPowerTimer> my_tmr_sptr) {
    tmr_sptr = my_tmr_sptr;
}

/**
 * Start the preset waiting period.
 */
void wait(void);
void waitNoWarn(void);
int32_t getWaitNoWarn(void);

};

extern RadioTiming radio_timing;


#endif /* RADIO_TIMING_HPP */