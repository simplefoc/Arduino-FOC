#include "samd_mcu.h"

#include <SPI.h>

SercomSpiClockMode from_SPI_MODE(int spi_mode)
{
    switch (spi_mode)
    {
      case SPI_MODE0:
        return SERCOM_SPI_MODE_0;
      case SPI_MODE1:
        return SERCOM_SPI_MODE_1;
      case SPI_MODE2:
        return SERCOM_SPI_MODE_2;
      case SPI_MODE3:
        return SERCOM_SPI_MODE_3;
      default:
        return SERCOM_SPI_MODE_0;
    }
}

uint32_t computeDSTADDR(uint8_t * startAddress, uint32_t STEPSEL, uint32_t STEPSIZE, uint32_t BEATSIZE, uint32_t BTCNT)
{
    /*
    p.283 When destination address incrementation is configured (BTCTRL.DSTINC is one), SRCADDR must be set to the 
    destination address of the last beat transfer in the block transfer. The destination address should be calculated as 
    follows: 
    DSTADDR = DSTADDRSTART + BTCNT ⋅ ( BEATSIZE + 1 ) ⋅ 2 STEPSIZE , where BTCTRL.STEPSEL is zero 
    DSTADDR = DSTADDRSTART + BTCNT ⋅ ( BEATSIZE + 1 ) , where BTCTRL.STEPSEL is one 
    -  DSTADDRSTART is the destination address of the first beat transfer in the block transfer 
    -  BTCNT is the initial number of beats remaining in the block transfer 
    -  BEATSIZE is the configured number of bytes in a beat 
    -  STEPSIZE is the configured number of beats for each incrementation 
    */
    uint32_t factor  = STEPSEL == 0 ? (1 << STEPSIZE) /*2^STEPSIZE*/: 1;

    return (uint32_t)(startAddress + BTCNT * (BEATSIZE + 1) * factor);   // end address
}


const SamdPinDefinition * getSamdPinDefinition(int arduinoPin)
{
    if(arduinoPin < 0)
    {
        debugPrint(F("getSamdPinDefinition() : pin < 0"));
        return nullptr;
    }
    if((uint32_t)arduinoPin > (PINS_COUNT-1))
    {
        debugPrintf("getSamdPinDefinition() : arduino pin %d above %d", arduinoPin, PINS_COUNT-1);
        return nullptr;
    }

    EPortType port = g_APinDescription[arduinoPin].ulPort;
    uint8_t port_pin = g_APinDescription[arduinoPin].ulPin;

    if(port_pin > 31)
    {
        debugPrint(F("getSamdPinDefinition() : port_pin > 31"));
        return nullptr;
    }

    if(port_pin > 31)
    {
        debugPrint(F("getSamdPinDefinition() : port_pin > 31"));
        return nullptr;
    }

    int8_t index = g_SamdMapPortPin[port][port_pin];

    if(index < 0)
    {
        debugPrint(F("getSamdPinDefinition() : index < 0, the pin doesn't exist!"));
        return nullptr;
    }
 
    const SamdPinDefinition * rv = &g_SamdPinDefinitions[index];

    debugPrintf("getSamdPinDefinition(): Arduino pin: %d, PORT%c%d, SERCOM%d:PAD[%d], SERCOM_ALT%d:PAD[%d] \n\r", arduinoPin, 'A' + port, port_pin, rv->sercom, rv->sercom_pad, rv->sercom_alt, rv->sercom_pad_alt);

    return rv;
}

