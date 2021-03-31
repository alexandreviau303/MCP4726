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

#include <Wire.h>

#include "MCP4726.h"



/**************************************************************************/
/*! 
    @brief  Instantiates a new MCP4726 class
*/
/**************************************************************************/
MCP4726::MCP4726(uint8_t a_vrefBits, uint8_t a_powerDownBits, uint8_t a_gainBits) 
   : m_vrefBits(a_vrefBits)
   , m_powerDownBits(a_powerDownBits)
   , m_gainBits(a_gainBits)
{
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
void MCP4726::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();

  writeVolatileMemory(m_vrefBits, m_powerDownBits, m_gainBits, 0);

}
 
/**************************************************************************/
/*! 
    @brief  Sets the output voltage to a fraction of source vref.  (Value
            can be 0..4096)

    @param[in]  output
                The 12-bit value representing the relationship between
                the DAC's input voltage and its output voltage.
*/
/**************************************************************************/
void MCP4726::setVoltage( uint16_t output)
{
  writeVolatileDACRegister(m_powerDownBits, output);
}

void MCP4726::writeVolatileDACRegister(uint8_t a_powerDown, uint16_t a_value)
{
   uint8_t toTransmit[2];

   toTransmit[0] = ((a_powerDown << 4) & 0x30) | ((a_value >> 8) & 0x0f);
   toTransmit[1] = a_value & 0x00ff;

   write(toTransmit, 2);
}

void MCP4726::writeVolatileMemory(uint8_t a_vref, uint8_t a_powerDown, uint8_t a_gain, uint16_t a_value)
{
   uint8_t toTransmit[3];

   toTransmit[0] = ((0b010 << 5) & 0xe0) | ((a_vref << 3) & 0x18) | ((a_powerDown << 1) & 0x6) | (a_gain & 0x01);
   toTransmit[1] = (a_value >> 8) & 0x00ff;
   toTransmit[2] = a_value & 0x00ff;

   write(toTransmit, 3);
}

void MCP4726::write(uint8_t* a_data, uint8_t a_size)
{
   Wire.beginTransmission(_i2caddr);
   for(uint8_t i = 0; i < a_size; i++)
   {
      Wire.write(a_data[i]);
   }
   Wire.endTransmission();
}
