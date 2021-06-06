#include "samd21_AdvancedSPI.h"
#include <wiring_private.h>


class SSGuard
{
    public:
    SSGuard(SAMDAdvancedSPI & spi) : spi(spi), isHoldingGuard(false)
    {
        if(!spi.hardwareSS && !spi.isSoftwareSSLow)
        {
            digitalWrite(spi.pinSS, LOW);
            spi.isSoftwareSSLow = true;
            isHoldingGuard = true;
        }
    }
    ~SSGuard()
    {
        if(isHoldingGuard)
        {
            digitalWrite(spi.pinSS, HIGH);
            spi.isSoftwareSSLow = false;
        }
    }

    SAMDAdvancedSPI & spi;
    bool isHoldingGuard;

};


SAMDAdvancedSPI::SAMDAdvancedSPI(SercomChannel sercomChannel, uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinSCK, uint8_t pinSS, bool hardwareSS) : 
    sercomChannel(sercomChannel), pinMOSI(pinMOSI), pinMISO(pinMISO), pinSCK(pinSCK), pinSS(pinSS), hardwareSS(hardwareSS)
{

}

int SAMDAdvancedSPI::init(SercomSpiClockMode spi_mode, uint32_t clock_speed)
{
    debugPrintf("MOSI: ");
    defMOSI = getSamdPinDefinition(pinMOSI);
    debugPrintf("MISO: ");
    defMISO = getSamdPinDefinition(pinMISO);
    debugPrintf("SCK : ");
    defSCK  = getSamdPinDefinition(pinSCK);
    debugPrintf("SS  : ");
    defSS   = getSamdPinDefinition(pinSS);

    if(defMOSI == nullptr || defMISO == nullptr || defSCK == nullptr || (hardwareSS && defSS == nullptr))
    {
        return -1;
    }

    sercomCfg  = getSercom(sercomChannel);

    sercomCfg.sercom->SPI.CTRLA.bit.ENABLE = 0;
    while(sercomCfg.sercom->SPI.SYNCBUSY.bit.ENABLE);

    // Setting NVIC
    NVIC_EnableIRQ(sercomCfg.irq);
    NVIC_SetPriority (sercomCfg.irq, SERCOM_NVIC_PRIORITY);  /* set Priority */

    //Setting clock
    GCLK_CLKCTRL_Type clckctrl{.reg = 0};
    clckctrl.bit.ID = sercomCfg.clockId ;
    clckctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
    clckctrl.bit.CLKEN = 0b1;
    GCLK->CLKCTRL.reg = clckctrl.reg;

    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    if(!hardwareSS)
    {
        pinMode(pinSS, OUTPUT);
        digitalWrite(pinSS, HIGH);
        isSoftwareSSLow = false;
    }

    //Setting the CTRLB register
    SERCOM_SPI_CTRLB_Type ctrlb{.reg = 0};
    ctrlb.bit.CHSIZE = SPI_CHAR_SIZE_8_BITS;
    ctrlb.bit.RXEN = 0b1;
    ctrlb.bit.MSSEN = hardwareSS ? 0b1 : 0b0; 
    sercomCfg.sercom->SPI.CTRLB.reg = ctrlb.reg;
    while(sercomCfg.sercom->SPI.SYNCBUSY.bit.CTRLB);


    //Synchronous arithmetic
    sercomCfg.sercom->SPI.BAUD.reg = SERCOM_FREQ_REF / (2 * clock_speed) - 1;

    //Setting the CTRLA register
    SERCOM_SPI_CTRLA_Type ctrla{.reg = 0};
    switch (spi_mode)
    {
        case SERCOM_SPI_MODE_0:
            ctrla.bit.CPOL = 0b0;
            ctrla.bit.CPHA = 0b0;
            break;
        case SERCOM_SPI_MODE_1:
            ctrla.bit.CPOL = 0b0;
            ctrla.bit.CPHA = 0b1; 
            break;
        case SERCOM_SPI_MODE_2:
            ctrla.bit.CPOL = 0b1;
            ctrla.bit.CPHA = 0b0; 
            break;
        case SERCOM_SPI_MODE_3:
            ctrla.bit.CPOL = 0b1;
            ctrla.bit.CPHA = 0b1; 
            break;
        default:
            debugPrint("SPI sercom CLOCK configuration error!");
            return -1;
    }

    SercomPad MOSI_pad, MISO_pad, SCK_pad, SS_pad;
    EPioType peripheral;
    
    if(defMOSI->sercom == sercomChannel && defMISO->sercom == sercomChannel && defSCK->sercom  == sercomChannel && (ctrlb.bit.MSSEN == 0 || defSS->sercom == sercomChannel))
    {
        MOSI_pad = defMOSI->sercom_pad;
        MISO_pad = defMISO->sercom_pad;
        SCK_pad = defSCK->sercom_pad;
        SS_pad = defSS->sercom_pad;
        peripheral = EPioType::PIO_SERCOM;

        debugPrintf("Using sercom %d\n\r", defMOSI->sercom);
    }
    else if(defMOSI->sercom_alt == sercomChannel && defMISO->sercom_alt == sercomChannel && defSCK->sercom_alt == sercomChannel && (ctrlb.bit.MSSEN == 0 || defSS->sercom_alt == sercomChannel))
    {
        MOSI_pad = defMOSI->sercom_pad_alt;
        MISO_pad = defMISO->sercom_pad_alt;
        SCK_pad = defSCK->sercom_pad_alt;
        SS_pad = defSS->sercom_pad_alt;
        peripheral = EPioType::PIO_SERCOM_ALT;

        debugPrintf("Using sercom alt %d\n\r", defMOSI->sercom_alt);
    } 
    else
    {
        debugPrint("SPI sercom CHANNEL configuration error!");
        return -1;
    }

    pinPeripheral(pinMOSI, peripheral);
    pinPeripheral(pinMISO, peripheral);
    pinPeripheral(pinSCK, peripheral);
    pinPeripheral(pinSS, ctrlb.bit.MSSEN ? peripheral : EPioType::PIO_OUTPUT); 

    debugPrintf("Using sercom pads MOSI[%d], MISO[%d], SCK[%d], SS[%d]\n\r", MOSI_pad, MISO_pad, SCK_pad, ctrlb.bit.MSSEN ? SS_pad : -1);

    if(MOSI_pad == PAD_0 && SCK_pad == PAD_1  
    && (ctrlb.bit.MSSEN ? (SS_pad == PAD_2 && MISO_pad == PAD_3) : (MISO_pad == PAD_3 || MISO_pad == PAD_2)))
    {
        ctrla.bit.DOPO = 0x0;
        ctrla.bit.DIPO = MISO_pad;
    }
    else if(MOSI_pad == PAD_2 && SCK_pad == PAD_3 
    && (ctrlb.bit.MSSEN ? (SS_pad == PAD_1 && MISO_pad == PAD_0) : (MISO_pad == PAD_0 || MISO_pad == PAD_1)))
    {
        ctrla.bit.DOPO = 0x1;
        ctrla.bit.DIPO = MISO_pad;
    }
    else if(MOSI_pad == PAD_3 && SCK_pad == PAD_1
    && (ctrlb.bit.MSSEN ? (SS_pad == PAD_2 && MISO_pad == PAD_0) : (MISO_pad == PAD_0 || MISO_pad == PAD_2)))
    {
        ctrla.bit.DOPO = 0x2;
        ctrla.bit.DIPO = MISO_pad;
    }
    else if(MOSI_pad == PAD_0 && SCK_pad == PAD_3
    && (ctrlb.bit.MSSEN ? (SS_pad == PAD_1 && MISO_pad == PAD_2) : (MISO_pad == PAD_2 || MISO_pad == PAD_1)))
    {
        ctrla.bit.DOPO = 0x3;
        ctrla.bit.DIPO = MISO_pad;
    }
    else
    {
        debugPrint("SPI sercom PADS configuration invalid!");
        return -1;
    }

    debugPrintf("Using sercom DOPO %d and DIPO %d\n\r", ctrla.bit.DOPO, ctrla.bit.DIPO);

    ctrla.bit.MODE = SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val;
    ctrla.bit.DORD = SercomDataOrder::MSB_FIRST;

    
    ctrla.bit.ENABLE = 0b1; //***enables the SPI***

    sercomCfg.sercom->SPI.CTRLA.reg = ctrla.reg;
    while(sercomCfg.sercom->SPI.SYNCBUSY.bit.ENABLE);


    return 0;
    
}

byte SAMDAdvancedSPI::spiCalcEvenParity(word value)
{
    byte cnt = 0;
    byte i;

    for (i = 0; i < 16; i++)
    {
        if (value & 0x1) cnt++;
        value >>= 1;
    }
    return cnt & 0x1;
}


uint8_t SAMDAdvancedSPI::transfer(uint8_t data)
{
    SSGuard g(*this);
    sercomCfg.sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register
    while( sercomCfg.sercom->SPI.INTFLAG.bit.RXC == 0 );
    return sercomCfg.sercom->SPI.DATA.bit.DATA;  // Reading data
}




uint16_t SAMDAdvancedSPI::transfer16(uint16_t data) 
{
    union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;
    
    
    SSGuard g(*this);

    t.val = data;

    if (sercomCfg.sercom->SPI.CTRLA.bit.DORD == SercomDataOrder::LSB_FIRST) {
        t.lsb = transfer(t.lsb);
        t.msb = transfer(t.msb);
    } else {
        t.msb = transfer(t.msb);
        t.lsb = transfer(t.lsb);
    }


    return t.val;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void SAMDAdvancedSPI::close(){
    //Setting the Software Reset bit to 1
    sercomCfg.sercom->SPI.CTRLA.bit.SWRST = 1;
    //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
    while(sercomCfg.sercom->SPI.CTRLA.bit.SWRST || sercomCfg.sercom->SPI.SYNCBUSY.bit.SWRST);
}