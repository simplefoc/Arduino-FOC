#pragma once
#include <Arduino.h>
#include <wiring_private.h>
#include "../MagneticSensor.h"
#include "../MagneticSensorSPI.h"
#include "../../common/hardware_specific/samd21_AdvancedSPI.h"

#define SPIbufferSize 2
class SAMDMagneticSensorSPI: public MagneticSensor, public TccInterruptCallback, public DMACInterruptCallback{
 public:
    /**
     * class constructor
     * @param config   SPI config
     */
    SAMDMagneticSensorSPI(MagneticSensorSPIConfig_s config, int8_t tccN = -1, int8_t dmaTX = -1, int8_t dmaRX = -1);

    /** sensor initialise pins */
    void init(SAMDAdvancedSPI* _spi);

    uint32_t getRawCount(uint64_t & timestamp_us) override;

    void operator()(Tcc * tcc) override;

    void operator()(volatile DMAC_CHINTFLAG_Type &, volatile DMAC_CHCTRLA_Type &) override;

  private:
    void tccHandler(Tcc * tcc);
    void initDMA();
    MagneticSensorSPIConfig_s config;

    SAMDAdvancedSPI* spi;
    int8_t tccN, dmaTX, dmaRX;

    
    uint8_t SPIreceiveBuffer[SPIbufferSize];
    uint8_t SPItransmitBuffer[SPIbufferSize];

    DmacDescriptor SPIdescriptors[2] __attribute__ ((aligned (16)));
};