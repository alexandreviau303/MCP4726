#pragma once

/**************************************************************************/
/*! 
    @file     MCP4726.cpp
    @author   S. Rodgers
	@license  BSD (see license.txt)
		Based on original work from Adafruit Industries.
        Modified for MCP4706 by Pio Baettig
        Modified for MCP4726 by S. Rodgers

	I2C Driver for Microchip's MCP4726 I2C DAC

	This is a library for the MCP4726 12-bit DAC modified from:
   
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define MCP4726_DEFAULT_ADDR 0x60

#define VREF_BITS_VDD_UNBUFFERED    0b00
#define VREF_BITS_VREF_UNBUFFERED   0b10
#define VREF_BITS_VREF_BUFFERD      0b11

#define POWER_DOWN_BITS_NOT_POWERED_DOWN  0b00
#define POWER_DOWN_BITS_POWERD_DOWN_1K    0b01
#define POWER_DOWN_BITS_POWERD_DOWN_100K  0b10
#define POWER_DOWN_BITS_POWERD_DOWN_500K  0b11

#define GAIN_BITS_1X 0b0
#define GAIN_BITS_2X 0b1

#include <Wire.h>

class MCP4726{
 public:
  MCP4726(uint8_t a_vrefBits = VREF_BITS_VDD_UNBUFFERED, uint8_t a_powerDownBits = POWER_DOWN_BITS_NOT_POWERED_DOWN, uint8_t a_gainBits = GAIN_BITS_1X);
  void begin(uint8_t a);  
  void setVoltage( uint16_t output);

 private:
  uint8_t _i2caddr;

  uint8_t m_vrefBits;
  uint8_t m_powerDownBits;
  uint8_t m_gainBits;

  void writeVolatileDACRegister(uint8_t a_powerDown, uint16_t a_value);
  void writeVolatileMemory(uint8_t a_vref, uint8_t a_powerDown, uint8_t a_gain, uint16_t a_value);
  void write(uint8_t* a_data, uint8_t a_size);
};
