#include <cstring>
#include "AFSK.h"


extern unsigned long custom_preamble;
extern unsigned long custom_tail;
extern int LibAPRS_vref;
extern bool LibAPRS_open_squelch;

bool hw_afsk_dac_isr = false;
Afsk *AFSK_modem;


// Forward declerations
int afsk_getchar(void);
void afsk_putchar(char c);

void AFSK_hw_refDetect(void) { }


//AnalogIn audio_in(A3);
#if defined (TARGET_NUCLEO_F746ZG) || defined (TARGET_NUCLEO_H743ZI2)
#warning Board has a DAC! Yay!
AnalogOut audio_out(PA_4);
#else
#warning Board does not have a DAC! Using dummy variable instead!
uint16_t audio_out;
#endif


void AFSK_hw_init(void) { }


void AFSK_init(Afsk *afsk) {
    // Allocate modem struct memory
    memset(afsk, 0, sizeof(*afsk));
    AFSK_modem = afsk;
    // Set phase increment
    afsk->phaseInc = MARK_INC;
    // Initialise FIFO buffers
    fifo_init(afsk->delayFifo, (uint8_t *)afsk->delayBuf, sizeof(afsk->delayBuf));
    fifo_init(afsk->rxFifo, afsk->rxBuf, sizeof(afsk->rxBuf));
    fifo_init(afsk->txFifo, afsk->txBuf, sizeof(afsk->txBuf));

    // Fill delay FIFO with zeroes
    for (int i = 0; i<SAMPLESPERBIT / 2; i++) {
        fifo_push(afsk->delayFifo, 0);
    }

    AFSK_hw_init();

}


static void AFSK_txStart(Afsk *afsk) {
    if (!afsk->sending) {
        afsk->phaseInc = MARK_INC;
        afsk->phaseAcc = 0;
        afsk->bitstuffCount = 0;
        afsk->sending = true;
        //LED_TX_ON();
        afsk->preambleLength = DIV_ROUND(custom_preamble * BITRATE, 8000);
        AFSK_DAC_IRQ_START();
    }
    {
      CriticalSectionLock lock;
      afsk->tailLength = DIV_ROUND(custom_tail * BITRATE, 8000);
    }
}


void afsk_putchar(char c) {
    AFSK_txStart(AFSK_modem);
    while(fifo_isfull_locked(AFSK_modem->txFifo)) { /* Wait */ }
    fifo_push_locked(AFSK_modem->txFifo, c);
}


void AFSK_transmit(char *buffer, size_t size) {
    fifo_flush(AFSK_modem->txFifo);
    int i = 0;
    while (size--) {
        afsk_putchar(buffer[i++]);
    }
}


uint8_t AFSK_dac_isr(Afsk *afsk) {
    if (afsk->sampleIndex == 0) {
        if (afsk->txBit == 0) {
            if (fifo_isempty(afsk->txFifo) && afsk->tailLength == 0) {
                AFSK_DAC_IRQ_STOP();
                afsk->sending = false;
                return 0;
            } else {
                if (!afsk->bitStuff) afsk->bitstuffCount = 0;
                afsk->bitStuff = true;
                if (afsk->preambleLength == 0) {
                    if (fifo_isempty(afsk->txFifo)) {
                        afsk->tailLength--;
                        afsk->currentOutputByte = HDLC_FLAG;
                    } else {
                        afsk->currentOutputByte = fifo_pop(afsk->txFifo);
                    }
                } else {
                    afsk->preambleLength--;
                    afsk->currentOutputByte = HDLC_FLAG;
                }
                if (afsk->currentOutputByte == AX25_ESC) {
                    if (fifo_isempty(afsk->txFifo)) {
                        AFSK_DAC_IRQ_STOP();
                        afsk->sending = false;
                        //LED_TX_OFF();
                        return 0;
                    } else {
                        afsk->currentOutputByte = fifo_pop(afsk->txFifo);
                    }
                } else if (afsk->currentOutputByte == HDLC_FLAG || afsk->currentOutputByte == HDLC_RESET) {
                    afsk->bitStuff = false;
                }
            }
            afsk->txBit = 0x01;
        }

        if (afsk->bitStuff && afsk->bitstuffCount >= BIT_STUFF_LEN) {
            afsk->bitstuffCount = 0;
            afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
        } else {
            if (afsk->currentOutputByte & afsk->txBit) {
                afsk->bitstuffCount++;
            } else {
                afsk->bitstuffCount = 0;
                afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
            }
            afsk->txBit <<= 1;
        }

        afsk->sampleIndex = SAMPLESPERBIT;
    }

    afsk->phaseAcc += afsk->phaseInc;
    afsk->phaseAcc %= SIN_LEN;
    afsk->sampleIndex--;

    return sinSample(afsk->phaseAcc);
}


/// Use a Ticker to determine when to send out the next sample
Ticker sample_timer;
void dac_isr(void) {
    audio_out = AFSK_dac_isr(AFSK_modem); 
}