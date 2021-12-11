/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*
 *  Modified by Neal Horman 7/14/2012 for use in mbed
 */

#ifndef _ADAFRUIT_SSD1306_H_
#define _ADAFRUIT_SSD1306_H_

#include "mbed.h"
#include "Adafruit_GFX.h"
#include "SoftI2C.h"

#include <vector>
#include <algorithm>

// A DigitalOut sub-class that provides a constructed default state
class DigitalOut2 : public DigitalOut
{
public:
	explicit DigitalOut2(PinName pin, bool active = false) : DigitalOut(pin) { write(static_cast<int>(active)); };
	auto operator= (int value) -> DigitalOut2& { write(value); return *this; };
	auto operator= (DigitalOut2 && rhs)  noexcept -> DigitalOut2& { write(rhs.read()); return *this; };
	explicit operator int() { return read(); };
    ~DigitalOut2() = default;
    DigitalOut2(const DigitalOut2 &rhs) = delete;
    auto operator= (const DigitalOut2 &) -> DigitalOut2 & = delete; 	
    DigitalOut2 (DigitalOut2 &&) = delete;
};

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

static constexpr uint32_t HEIGHT = 32;
static constexpr uint32_t WIDTH = 128;
static constexpr uint8_t DATA_MODE = 0x40;

/** The pure base class for the SSD1306 display driver.
 *
 * You should derive from this for a new transport interface type,
 * such as the SPI and I2C drivers.
 */
class Adafruit_SSD1306 : public Adafruit_GFX
{
public:
	explicit Adafruit_SSD1306(PinName RST, uint8_t rawHeight = HEIGHT, uint8_t rawWidth = WIDTH)
		: Adafruit_GFX(rawWidth,rawHeight)
		, rst(RST,false)
	{
        static constexpr uint8_t BITS_PER_BYTE = 8;
		buffer.resize(rawHeight * rawWidth / BITS_PER_BYTE);
	};

	void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC);
	
	// These must be implemented in the derived transport driver
	virtual void command(uint8_t c) = 0;
	virtual void data(uint8_t c) = 0;
	void drawPixel(int16_t x, int16_t y, uint16_t color) override;

    // Turn the display on and off
    void displayOff();
    void displayOn();

	/// Clear the display buffer    
	void clearDisplay();
	void invertDisplay(bool i) override;

	/// Cause the display to be updated with the buffer content.
	void display();
	/// Fill the buffer with the AdaFruit splash screen.
	virtual void splash();
    
    auto get_buffer() -> std::vector<uint8_t> & {
        return buffer;
    }

private:
	virtual void sendDisplayBuffer() = 0;
	DigitalOut2 rst;

	// the memory buffer for the LCD
	std::vector<uint8_t> buffer;
};


/** This is the SPI SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_Spi : public Adafruit_SSD1306
{
public:
	/** Create a SSD1306 SPI transport display driver instance with the specified DC, RST, and CS pins, as well as the display dimentions
	 *
	 * Required parameters
	 * @param spi - a reference to an initialized SPI object
	 * @param DC (Data/Command) pin name
	 * @param RST (Reset) pin name
	 * @param CS (Chip Select) pin name
	 *
	 * Optional parameters
	 * @param rawHeight - the vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - the horizonal number of pixels for the display, defaults to 128
	 */
	Adafruit_SSD1306_Spi(SPI &spi, PinName DC, PinName RST, PinName CS, uint8_t rawHieght = HEIGHT, uint8_t rawWidth = WIDTH)
	    : Adafruit_SSD1306(RST, rawHieght, rawWidth)
	    , cs(CS,true)
	    , dc(DC,false)
	    , mspi(spi)
	    {
		    begin();
		    splash();
		    display();
	    };

	void command(uint8_t c) override
	{
	    cs = 1;
	    dc = 0;
	    cs = 0;
	    mspi.write(c);
	    cs = 1;
	};

	void data(uint8_t c) override
	{
	    cs = 1;
	    dc = 1;
	    cs = 0;
	    mspi.write(c);
	    cs = 1;
	};

private:
	void sendDisplayBuffer() override
	{
		cs = 1;
		dc = 1;
		cs = 0;

		for(unsigned char i : get_buffer()) {
			mspi.write(i);
        }

		if(height() == HEIGHT)
		{
			for(uint16_t i = 0, q = get_buffer().size(); i < q; i++) {
				mspi.write(0);
            }
		}

		cs = 1;
	};

	DigitalOut2 cs, dc;
	SPI &mspi;
};

/** This is the I2C SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_I2c : public Adafruit_SSD1306
{
public:
	#define SSD_I2C_ADDRESS     0x78
	/** Create a SSD1306 I2C transport display driver instance with the specified RST pin name, the I2C address, as well as the display dimensions
	 *
	 * Required parameters
	 * @param i2c - A reference to an initialized I2C object
	 * @param RST - The Reset pin name
	 *
	 * Optional parameters
	 * @param i2cAddress - The i2c address of the display
	 * @param rawHeight - The vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - The horizonal number of pixels for the display, defaults to 128
	 */
	Adafruit_SSD1306_I2c(SoftI2C &i2c, PinName RST, uint8_t i2cAddress = SSD_I2C_ADDRESS, uint8_t rawHeight = HEIGHT, uint8_t rawWidth = WIDTH)
	    : Adafruit_SSD1306(RST, rawHeight, rawWidth)
	    , mi2c(i2c)
	    , mi2cAddress(i2cAddress)
	    {
		    begin();
		    splash();
		    display();
	    };

	void command(uint8_t c) override
	{
		vector<char> buff(2);
		buff[0] = 0; // Command Mode
		buff[1] = c;
		mi2c.write(mi2cAddress, buff.data(), buff.size());
	}

	void data(uint8_t c) override
	{
		vector<char> buff(2);
		buff[0] = DATA_MODE; // Data Mode
		buff[1] = c;
		mi2c.write(mi2cAddress, buff.data(), buff.size());
	};

private:
	void sendDisplayBuffer() override
	{
        static constexpr uint32_t BUFF_SIZE = 17;
		vector<char> buff(BUFF_SIZE);
		buff[0] = DATA_MODE; // Data Mode

		// send display buffer in 16 byte chunks
        static constexpr uint16_t CHUNK_SIZE = 16;
		for(uint16_t i = 0, q = get_buffer().size(); i < q; i += CHUNK_SIZE) 
		{	
			// TODO(unknown): - this will segfault if buffer.size() % 16 != 0
			for(uint32_t x = 1; x < buff.size(); x++) {
				buff[x] = get_buffer()[i+x-1];
            }
			mi2c.write(mi2cAddress, buff.data(), buff.size());
		}
	};

	SoftI2C &mi2c;
	uint8_t mi2cAddress;
};

#endif