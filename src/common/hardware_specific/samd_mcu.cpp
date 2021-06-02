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

const SamdPinDefinition * getSamdPinDefinition(int arduinoPin)
{
    if(arduinoPin < 0)
    {
        debugPrint(F("getSamdPinDefinition() : pin < 0"));
        return nullptr;
    }
    if(arduinoPin > PINS_COUNT-1)
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