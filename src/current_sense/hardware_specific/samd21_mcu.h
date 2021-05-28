#ifndef CURRENT_SENSE_SAMD21_H
#define CURRENT_SENSE_SAMD21_H


#include <stdint.h>

// #define SIMPLEFOC_SAMD_DEBUG
#if !defined(SIMPLEFOC_SAMD_DEBUG_SERIAL)
#define SIMPLEFOC_SAMD_DEBUG_SERIAL Serial
#endif


class SAMDCurrentSenseADCDMA
{

public:
  static SAMDCurrentSenseADCDMA * getHardwareAPIInstance();
  SAMDCurrentSenseADCDMA();
  void init(int pinA, int pinB, int pinC, int EVSYS_ID_GEN_TCC_OVF = -1, int pinAREF = -1, float voltageAREF = 3.3, uint32_t adcBits = 12, uint32_t channelDMA = 3);
  bool readResults(uint16_t & a, uint16_t & b, uint16_t & c);
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
  void initEVSYS();
 

  uint32_t oneBeforeFirstAIN; // hack to discard first noisy readout
  uint32_t firstAIN;
  uint32_t lastAIN; 
  uint32_t bufferSize = 0;


  uint32_t pinA;
  uint32_t pinB;
  uint32_t pinC;
  uint32_t pinAREF;
  uint32_t channelDMA;  // DMA channel
  bool freeRunning;

  float voltageAREF;
  float maxCountsADC;
  float countsToVolts;

};

#endif