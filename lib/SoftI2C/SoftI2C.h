#include "mbed.h"

#ifndef SOFTI2C_H
#define SOFTI2C_H

/**
* A software I2C class in case you run out of hardware I2C pins or cannot use
* those for another reason.
* 
* The class is a drop-in replacement of the classic mbed I2C class: include 
* this file in your program, replace I2C with SoftI2C when creating the object
* and it should work properly without other modifications.
*/
class SoftI2C {
    public:
    /**
    * Create an software I2C Master interface, connected to the specified pins
    *
    * @param sda I2C data pin 
    * @param scl I2C clock pin
    */
    SoftI2C(PinName sda, PinName scl);
    
    /**
    * Set the frequency of the I2C interface. 
    *
    * Note that the delay of the DigitalInOuts is not compensated: the real
    * frequency will be lower than the one set. However since I2C is a
    * synchronous protocol this shouldn't affect functionality
    *
    * @param hz The bus frequency in hertz 
    */ 
    void frequency(int hz);
    
    /**
    * Read from an I2C slave. 
    * 
    * Performs a complete read transaction. The bottom bit of the address is forced to 1 to indicate a read.
    *
    * @param address 8-bit I2C slave address [ addr | 1 ] 
    * @param Pointer to the byte-array data to read to 
    * @param length Number of bytes to read
    * @param repeated Repeated start, true - do not send stop at end
    * @return 0 on success (ack), non-0 on failure (nack) 
    */
    int read(int address, char *data, int length, bool repeated=false);  
    
    /**
    * Read a single byte from the I2C bus
    *
    * @param ack indicates if the byte is to be acknowledged (1 = acknowledge)
    * @return the byte read
    */
    int read(int ack);
    
    /**
    * Write to an I2C slave. 
    * 
    * Performs a complete write transaction. The bottom bit of the address is forced to 0 to indicate a write.
    *
    * @param address 8-bit I2C slave address [ addr | 0 ] 
    * @param Pointer to the byte-array data to send 
    * @param length Number of bytes to send
    * @param repeated Repeated start, true - do not send stop at end
    * @return 0 on success (ack), non-0 on failure (nack) 
    */
    int write(int address, const char *data, int length, bool repeated=false);  
    
    /**
    * Write single byte out on the I2C bus
    *
    * @param data data to write on the bus
    * @return '1' if an ACK is received, '0' otherwise
    */
    int write(int data);
    
    /**
    * Create a (re-)start condition on the I2C bus
    */
    void start(void);
    
    /**
    * Create a stop condition on the I2C bus
    */
    void stop(void);
    
    protected:
    DigitalInOut _sda;
    DigitalInOut _scl;
    int delay_us;
    bool active;
};


#endif