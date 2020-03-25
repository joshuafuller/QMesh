#include "SoftI2C.h"

SoftI2C::SoftI2C(PinName sda, PinName scl) : _sda(sda), _scl(scl) {
    // Set defaults
    _sda.mode(PullNone);
    _scl.mode(PullNone);
    _sda.input();
    _scl.input();
    frequency(100000);
    
    active = false;
    
    }
    
void SoftI2C::frequency(int hz) {
    delay_us = 1000000 / hz / 4; //delay is a quarter of the total period
}

int SoftI2C::read(int address, char *data, int length, bool repeated) {
    start();
    
    // Write address with LSB to one
    if (write(address | 0x01) == 0) {
        return 1;
    }  
    
    // Read the data
    for(int i = 0; i<length - 1; i++) {
        data[i] = read(1);
    }
    data[length-1] = read(0);
    
    if (repeated == false) {
        stop();
    }
    return 0;
}

int SoftI2C::write(int address, const char *data, int length, bool repeated) {
    start();
    
    // Write address with LSB to zero
    if (write(address & 0xFE) == 0) {
        return 1;
    }  
    
    // Write the data
    for(int i = 0; i<length; i++) {
        if(write(data[i]) == 0) {
            return 1;
        }
    }
    
    if (repeated == false) {
        stop();
    }
    return 0;
}

int SoftI2C::read(int ack) {
    int retval = 0;
    _scl.output();
    
    // Shift the bits out, msb first
    for (int i = 7; i>=0; i--) {
        //SCL low
        _scl.write(0);
        _sda.input();
        wait_us(delay_us);
        
        //read SDA
        retval |= _sda.read() << i;
        wait_us(delay_us);
        
        //SCL high again
        _scl.write(1);
        wait_us(delay_us << 1); //wait two delays
    }
    
    // Last cycle to set the ACK
    _scl.write(0);
    if ( ack ) {
        _sda.output();
        _sda.write(0);
    } else {
        _sda.input();
    }
    wait_us(delay_us << 1);
    
    _scl.write(1);
    wait_us(delay_us << 1);

    
    return retval;
}

int SoftI2C::write(int data) {
     _scl.output();
     
    // Shift the bits out, msb first
    for (int i = 7; i>=0; i--) {
        //SCL low
        _scl.write(0);
        wait_us(delay_us);
        
        //Change SDA depending on the bit
        if ( (data >> i) & 0x01 ) {
            _sda.input();
        } else {
            _sda.output();
            _sda.write(0);
        }
        wait_us(delay_us);
        
        //SCL high again
        _scl.write(1);
        wait_us(delay_us << 1); //wait two delays
    }
    
    // Last cycle to get the ACK
    _scl.write(0);
    wait_us(delay_us);
    
    _sda.input();
    wait_us(delay_us);
    
    _scl.write(1);
    wait_us(delay_us);
    int retval = ~_sda.read(); //Read the ack
    wait_us(delay_us);
    
    return retval;
}

void SoftI2C::start(void) {
    if (active) { //if repeated start
        //Set SDA high, toggle scl
        _sda.input();
        _scl.output();
        _scl.write(0);
        wait_us(delay_us << 1);
        _scl.write(1);
        wait_us(delay_us << 1);
    }
    // Pull SDA low
    _sda.output();
    _sda.write(0);
    wait_us(delay_us);
    active = true;
}

void SoftI2C::stop(void) {
    // Float SDA high
    _scl.output();
    _scl.write(0);
    _sda.output();
    _sda.write(0);
    wait_us(delay_us);
    _scl.input();
    wait_us(delay_us);
    _sda.input();
    wait_us(delay_us);
    
    active = false;
}