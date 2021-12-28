#include "SoftI2C.h"

static constexpr uint32_t NUM_SHIFTS = 7;
static constexpr int DEFAULT_FREQUENCY = 100000;
static constexpr int US_IN_S = 1000000;
static constexpr int FREQ_100KHZ = 100000;
static constexpr int FREQ_400KHZ = 400000;

SoftI2C::SoftI2C(PinName sda, PinName scl) : _sda(sda), _scl(scl) {
    // Set defaults
    _sda.mode(PullNone);
    _scl.mode(PullNone);
    _sda.input();
    _scl.input();
    delay_us = 0;
    frequency(DEFAULT_FREQUENCY);
    
    active = false;
    
    }

void SoftI2C::frequency(const int hz) {
    PORTABLE_ASSERT(hz == FREQ_100KHZ || hz == FREQ_400KHZ);
    delay_us = US_IN_S / hz / 4; //delay is a quarter of the total period
}

auto SoftI2C::read(const uint32_t address, char *data, const int length, 
                    const bool repeated) -> int {
    PORTABLE_ASSERT(address < 128);
    PORTABLE_ASSERT(data != nullptr);
    PORTABLE_ASSERT(length > 0 && length < 256);
    start();
    
    // Write address with LSB to one
    if (write(static_cast<int32_t>(address | 0x01U)) == 0) {
        return 1;
    }  
    
    // Read the data
    for(int i = 0; i<length - 1; i++) {
        data[i] = read(1); //NOLINT
    }
    data[length-1] = read(0); //NOLINT
    
    if (repeated == false) {
        stop();
    }
    return 0;
}

auto SoftI2C::write(const uint32_t address, const char *data, const int length, 
                    const bool repeated) -> int {
    PORTABLE_ASSERT(address < 128);
    PORTABLE_ASSERT(data != nullptr);
    PORTABLE_ASSERT(length > 0 && length < 256);
    start();
    
    // Write address with LSB to zero
    static constexpr uint32_t ZERO_LSB_MASK = 0xFEU;
    if(write(static_cast<int32_t>(address & ZERO_LSB_MASK)) == 0) {
        return 1;
    }  
    
    // Write the data
    for(int i = 0; i<length; i++) {
        if(write(data[i]) == 0) { //NOLINT
            return 1;
        }
    }
    
    if (repeated == false) {
        stop();
    }
    return 0;
}

auto SoftI2C::read(const int ack) -> int {
    int retval = 0;
    _scl.output();
    
    // Shift the bits out, msb first
    for (int i = NUM_SHIFTS; i >= 0; i--) {
        //SCL low
        _scl.write(0);
        _sda.input();
        portability::wait_us(delay_us);
        
        //read SDA
        uint32_t retval_tmp = 0U;
        retval_tmp = static_cast<uint32_t>(_sda.read()) << static_cast<uint32_t>(i);
        retval_tmp |= static_cast<uint32_t>(retval);
        retval = retval_tmp;
        //retval |= static_cast<uint32_t>(_sda.read()) << i;
        portability::wait_us(delay_us);
        
        //SCL high again
        _scl.write(1);
        portability::wait_us(delay_us << 1U); //wait two delays
    }
    
    // Last cycle to set the ACK
    _scl.write(0);
    if ( ack != 0 ) {
        _sda.output();
        _sda.write(0);
    } else {
        _sda.input();
    }
    portability::wait_us(delay_us << 1U);
    
    _scl.write(1);
    portability::wait_us(delay_us << 1U);

    
    return retval;
}

auto SoftI2C::write(const int data) -> int {
     _scl.output();
     
    // Shift the bits out, msb first
    for (int i = NUM_SHIFTS; i >= 0; i--) {
        //SCL low
        _scl.write(0);
        portability::wait_us(delay_us);
        
        //Change SDA depending on the bit
        if ( ((static_cast<uint32_t>(data) >> static_cast<uint32_t>(i)) & 0x01U) != 0 ) {
            _sda.input();
        } else {
            _sda.output();
            _sda.write(0);
        }
        portability::wait_us(delay_us);
        
        //SCL high again
        _scl.write(1);
        portability::wait_us(delay_us << 1U); //wait two delays
    }
    
    // Last cycle to get the ACK
    _scl.write(0);
    portability::wait_us(delay_us);
    
    _sda.input();
    portability::wait_us(delay_us);
    
    _scl.write(1);
    portability::wait_us(delay_us);
    int retval = ~static_cast<uint32_t>(_sda.read()); //Read the ack
    portability::wait_us(delay_us);
    
    return retval;
}

void SoftI2C::start() {
    if (active) { //if repeated start
        //Set SDA high, toggle scl
        _sda.input();
        _scl.output();
        _scl.write(0);
        portability::wait_us(delay_us << 1U);
        _scl.write(1);
        portability::wait_us(delay_us << 1U);
    }
    // Pull SDA low
    _sda.output();
    _sda.write(0);
    portability::wait_us(delay_us);
    active = true;
}

void SoftI2C::stop() {
    // Float SDA high
    _scl.output();
    _scl.write(0);
    _sda.output();
    _sda.write(0);
    portability::wait_us(delay_us);
    _scl.input();
    portability::wait_us(delay_us);
    _sda.input();
    portability::wait_us(delay_us);
    
    active = false;
}