#pragma once

#include <Arduino.h>

#ifdef _SAMD21_

#include "samd_mcu.h"

class SAMDAdvancedSPI
{
    public:

        SAMDAdvancedSPI(SercomChannel sercomChannel, uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinSCK, uint8_t pinSS, bool hardwareSS);

        int init(SercomSpiClockMode spi_mode, uint32_t clock_speed);
    
        byte spiCalcEvenParity(word value);

        uint8_t transfer(uint8_t data);

        uint16_t transfer16(uint16_t data);

        void close();

    // private:

        
        SercomChannel sercomChannel;
        const uint8_t pinMOSI;
        const uint8_t pinMISO;
        const uint8_t pinSCK;
        const uint8_t pinSS;
        const bool hardwareSS;
        bool isSoftwareSSLow;
        
        const SamdPinDefinition * defMOSI;
        const SamdPinDefinition * defMISO;
        const SamdPinDefinition * defSCK;
        const SamdPinDefinition * defSS;
        SercomConfig sercomCfg;

        friend class SSGuard;
};

#endif