/******************************************************************************
u-blox_GNSS.h
u-blox GNSS Arduino
Leonardo Bispo
Mar 03, 2019
https://github.com/ldab/u-blox_GNSS

Distributed as-is; no warranty is given.
******************************************************************************/

#include "ublox_GNSS.h"
#include <cstdint>
#include "mbed.h"

void GNSS::clearUART( void )
{
    while( stream.readable() > 0 ) {
        uint8_t my_byte[1];
        stream.read(my_byte, 1);
    }
}

void GNSS::bits_char( uint32_t c, uint8_t *_c )
{
  for( int i = 0; i < 4; i++)
  {
    _c[i] = ( c >> 8*i ) & 0xFF;
  }
}

bool GNSS::init( psmMode_t m )
{
  DBG("\nStart GNSS Configuration");
  
  uint8_t my_byte;
  stream.write(&my_byte, 1);          // Send something to wake GNSS
  ThisThread::sleep_for(600);

  // Disable NMEA, UBX-CFG-PRT -> Enable UBX over UART1 and Baud rate 9600
  uint8_t message[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                    0x80, 0x25, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79};

  DBG("UBX-CFG-PRT -> Disable NMEA and enable UBX");

  bool ret = sendUBX(message, sizeof(message));

  // Clear NMEA messages
  clearUART();

  if( m == PSM_1HZ )
  {
    DBG("UBX-CFG-PSM -> Power save mode");
    uint8_t confmessage[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x97, 0x6F};

    ret &= sendUBX( confmessage, sizeof(confmessage) );
  }
  else if( m == CONTINOUS )
  {
    DBG("UBX-CFG-PSM -> Default, balanced mode");
    uint8_t confmessage[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x95, 0x61};

    ret &= sendUBX( confmessage, sizeof(confmessage) );
  }
  

  DBG("UBX-CFG-CFG -> Saving configuration");

  ret &= saveCFG();

  gnss_init = ret;

	return gnss_init;
}

bool GNSS::init( psmMode_t m, uint32_t sleep, uint32_t onTime )
{
  if( m != ON_OFF)
    return false;

  init( ON_OFF );

  // Convert sleep time to ms, 4 bytes
  uint8_t _sleep[4];
  bits_char(sleep, _sleep);

  // Convert sleep time and on time to seconds, 2 unsigned bytes:
  uint16_t period = sleep/1000;

  uint8_t _ontime[2];
  _ontime[0] = onTime & 0xFF;
  _ontime[1] = (onTime >> 8) & 0xFF;

  uint8_t _period[2];
  _period[0] = period & 0xFF;
  _period[1] = (period >> 8) & 0xFF ;

  DBG("UBX-CFG-PSM -> Power save mode ON/OFF");

  uint8_t pms[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x02, _period[0], _period[1], _ontime[0], _ontime[1], 0x00, 0x00,
                0x00, 0x00};

  uint8_t CK_A = 0, CK_B = 0;
  for( uint8_t i = 2; i < (sizeof(pms) - 2) ; i++ )
  {
    CK_A = CK_A + pms[i];
    CK_B = CK_B + CK_A;
  }

  CK_A &= 0xFF;
  CK_B &= 0xFF;

  pms[14] = CK_A;
  pms[15] = CK_B;
  
  bool ret = sendUBX( pms, sizeof(pms));

  DBG("UBX-CFG-PS2 -> Configure search time");

  uint8_t onoff[] = {0xB5, 0x62, 0x06, 0x3B, 0x30, 0x00, 0x02, 0x06, 0x00, 0x00, 0x00, 0x10, 0x40, 0x01, _sleep[0], _sleep[1],
                  _sleep[2], _sleep[3], 0xC0, 0x45, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, _ontime[0], _ontime[1], 0x78, 0x00,
                  0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 
                  0x64, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  CK_A = 0;
  CK_B = 0;
  for( uint8_t i = 2; i < (sizeof(onoff) - 2) ; i++ )
  {
    CK_A = CK_A + onoff[i];
    CK_B = CK_B + CK_A;
  }

  CK_A &= 0xFF;
  CK_B &= 0xFF;

  onoff[54] = CK_A;
  onoff[55] = CK_B;
  
  ret &= sendUBX( onoff, sizeof(onoff));

  DBG("UBX-CFG-CFG -> Saving configuration");

  ret &= saveCFG();

  gnss_init = ret;

	return gnss_init;
}

bool GNSS::sendUBX( uint8_t *msg, uint32_t size )
{
  //DBG("Sending UBX");
  stream.write( msg, size);

  int i = 0;
  uint8_t _msg[10];
  Timer tmr;
  tmr.start();
  //uint32_t startMillis = millis();

  do{
    while (stream.readable() > 0) {
      stream.read(&_msg[i], 1);
      i++;
      if (i < 10) continue;
    }
  } while (tmr.read_ms() < UART_TIMEOUT);

  // Clear Version message from UART
  clearUART();

  /*while( i > 0 ) {
    DBG(_msg[i-1]);
    i--;
  }*/

  if( _msg[3] == 0 )
  {
    // Message Not-Acknowledged
    DBG("Message Not-Acknowledged");
    return false;
  }

  return crc( _msg , sizeof(_msg) );

}

bool GNSS::getCoodinates( float &lon, float &lat, fixType_t &fix, float &acc, float acc_min )
{
  DBG("UBX-NAV-PVT -> Pooling coordinates");

  if( !gnss_init )
  {
    lat = 0;
    lon = 0;
    acc = 0;
    fix = (fixType_t)0;
    return false;
  }

  int i = 0;
  uint8_t res[100];
  Timer tmr;

  // Reset accuracy for while loop;
  acc = 10000;

  // Poll navigation data byte -> UBX-NAV-PVT
  uint8_t nav_pvt[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
  uint8_t my_byte = 0xFF;
  stream.write(&my_byte, 1);          // Send something to wake GNSS
  ThisThread::sleep_for(600);

  // Clear NMEA messages if any
  clearUART();
  
do{  
  stream.write( nav_pvt, sizeof(nav_pvt) );

  i = 0;
  tmr.stop();
  tmr.reset();
  tmr.start();

  do{
    while (stream.readable() > 0) {
      stream.read(&res[i], 1);
      i++;
      if (i < 100) continue;
    }
  } while (tmr.read_ms() < UART_TIMEOUT);
  
  /*while( i >= 0 ) {
    DBG(res[i]);
    i--;
  }*/

  lon = (float)((res[33] << 24) + (res[32] << 16) + (res[31] << 8) + res[30]) * 0.0000001f;
  lat = (float)((res[37] << 24) + (res[36] << 16) + (res[35] << 8) + res[34]) * 0.0000001f;
  fix = (fixType_t)res[26];
  acc = (float)((res[49] << 24) + (res[48] << 16) + (res[47] << 8) + res[46] ) / 1000.0f;

  DBG("\nlon: ", lon, "lat: ", lat, " - ", fix, " ~", acc, "m");
  if( acc < acc_min ) goto finish;
  else {
    ThisThread::sleep_for(2000);
    clearUART();
  }

}while( tmr.read() < FIX_TIMEOUT );

  finish:
  return crc( res, sizeof(res) );
}

bool GNSS::crc( uint8_t *msg, uint32_t size)
{
  uint32_t s = size;
  uint8_t CK_A = 0, CK_B = 0;

  for( uint8_t i = 2; i < (s-2) ; i++)
  {
    CK_A = CK_A + msg[i];
    CK_B = CK_B + CK_A;
  }
  bool checksum = ( CK_A == msg[s-2] && CK_B == msg[s-1]);
  DBG("CRC = ", checksum);

  return checksum;

}

void GNSS::off( void )
{
  // first send dumb data to make sure its on
  uint8_t my_byte = 0xFF;
  stream.write(&my_byte, 1);

  uint8_t message[] = {0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                    0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x61, 0x6B};

  stream.write(message, sizeof(message));
}

void GNSS::factoryRST( void )
{
  // first send dumb data to make sure its on
  uint8_t my_byte = 0xFF;
  stream.write(&my_byte, 1);

  uint8_t message[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07, 0x1F, 0x9E};

  stream.write(message, sizeof(message));
}

bool GNSS::saveCFG()
{
  return sendUBX( _saveCFG, sizeof(_saveCFG));
}
