#ifndef I2C_HPP
#define I2C_HPP


#if defined(MBED_OS)
#include "mbed.h"
class SoftI2C;
using PORTABLE_I2C = SoftI2C;

#elif defined(ESP_IDF)
#include "Arduino.h" // Using the Arduino library here
#include <spi.h>
class PORTABLE_SPI {
private:
    SPISettings settings;
public:
    SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel) {     
        SPI.begin();
    }

    ~SPI() {
        SPI.end();
    }

    void format(int bits, int mode = 0) {
        if(mode == 0) {
            settings.dataMode = SPI_MODE0;
        } else if(mode == 1) {
            settings.dataMode = SPI_MODE1;
        } else if(mode == 2) {
            settings.dataMode = SPI_MODE2;
        } else if(mode == 3) {
            settings.dataMode = SPI_MODE3;
        } else {
            MBED_ASSERT(false);
        }

        settings.dataOrder = MSGFIRST;
    }

    void frequency(int hz=1000000) { 
        settings.speedMaximum = hz;
    }

    void lock() { }

    void unlock() { }

    write(int val) -> int {
        SPI.beginTransaction(settings);
        int ret_val = spi.transfer(val);
        SPI.endTransaction(settings);
        return ret_val;
    }
};
#else
#error Need to define either MBED_OS or ESP_IDF
#endif


#endif /* I2C_HPP */