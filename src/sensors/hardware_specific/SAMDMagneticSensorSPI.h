#pragma once
#include <Arduino.h>
#include <wiring_private.h>
#include "../MagneticSensor.h"
#include "../MagneticSensorSPI.h"
#include "../../common/hardware_specific/samd21_AdvancedSPI.h"

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

    void operator()(uint8_t channel, volatile DMAC_CHINTFLAG_Type &, volatile DMAC_CHCTRLA_Type &) override;

  private:
    void initDMA();
    word makeSPICommand();
    word extractResult(word register_value);
    MagneticSensorSPIConfig_s config;

    SAMDAdvancedSPI* spi;
    word command;
    int8_t tccN, dmaTX, dmaRX;
    bool pinLow;
    
    static const uint8_t bufferSize = 2;
    uint8_t receiveBuffer[bufferSize];
    uint8_t transmitBuffer[bufferSize];
    uint64_t last_timestamp_us;


    DmacDescriptor SPIdescriptors[2] __attribute__ ((aligned (16)));
};