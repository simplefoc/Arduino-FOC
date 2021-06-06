#ifndef CURRENT_SENSE_SAMD21_H
#define CURRENT_SENSE_SAMD21_H


#include <stdint.h>
#include <Arduino.h>
#include "../../common/hardware_specific/samd_mcu.h"
// #define SIMPLEFOC_SAMD_DEBUG
#if !defined(SIMPLEFOC_SAMD_DEBUG_SERIAL)
#define SIMPLEFOC_SAMD_DEBUG_SERIAL Serial
#endif


class SAMDCurrentSenseADCDMA : public DMACInterruptCallback
{

public:
  static SAMDCurrentSenseADCDMA * getHardwareAPIInstance();
  SAMDCurrentSenseADCDMA();
  
  void init(int pinA, int pinB, int pinC, int EVSYS_ID_GEN_TCC_OVF = -1, int pinAREF = -1, float voltageAREF = 3.3, uint32_t adcBits = 12, uint32_t channelDMA = 3);
  
  uint32_t readResults(uint16_t & a, uint16_t & b, uint16_t & c);

  void operator()(volatile DMAC_CHINTFLAG_Type &, volatile DMAC_CHCTRLA_Type &) override;
  
  float toVolts(uint16_t counts);
  
  uint16_t adcBuffer[20];
  int adc_i = 0;
  int dma_i = 0;
  int EVSYS_ID_GEN_TCC_OVF;
private:


  void initPins();
  void initADC();
  void initDMA();
  void initDMAChannel();
  int initEVSYS();
 

  uint32_t oneBeforeFirstAIN; // hack to discard first noisy readout
  uint32_t firstAIN;
  uint32_t lastAIN; 
  uint32_t ainA;
  uint32_t ainB;
  uint32_t ainC;
  uint32_t refsel;
  uint32_t bufferSize = 0;


  int pinA;
  int pinB;
  int pinC;
  int pinAREF;
  uint32_t channelDMA;  // DMA channel
  bool freeRunning;

  float voltageAREF;
  float maxCountsADC;
  float countsToVolts;

};

#endif