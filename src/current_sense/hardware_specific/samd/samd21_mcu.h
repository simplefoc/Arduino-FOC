#ifdef _SAMD21_

#ifndef CURRENT_SENSE_SAMD21_H
#define CURRENT_SENSE_SAMD21_H

#define SIMPLEFOC_SAMD_DEBUG
#if !defined(SIMPLEFOC_SAMD_DEBUG_SERIAL)
#define SIMPLEFOC_SAMD_DEBUG_SERIAL Serial
#endif

#include <stdint.h>
  typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
  } dmacdescriptor ;


// AREF pin is 42

class SAMDCurrentSenseADCDMA
{

public:
  static SAMDCurrentSenseADCDMA * getHardwareAPIInstance();
  SAMDCurrentSenseADCDMA();
  void init(int pinA, int pinB, int pinC, int pinAREF = 42, float voltageAREF = 3.3, uint32_t adcBits = 12, uint32_t channelDMA = 3);
  void startADCScan();
  bool readResults(uint16_t & a, uint16_t & b, uint16_t & c);
  float toVolts(uint16_t counts);
private:

  void adcToDMATransfer(void *rxdata,  uint32_t hwords);

  void initPins();
  void initADC();
  void initDMA();
 
  uint32_t oneBeforeFirstAIN; // hack to discard first noisy readout
  uint32_t firstAIN;
  uint32_t lastAIN; 
  uint32_t BufferSize = 0;

  uint16_t adcBuffer[20];


  uint32_t pinA;
  uint32_t pinB;
  uint32_t pinC;
  uint32_t pinAREF;
  uint32_t channelDMA;  // DMA channel
  bool freeRunning;

  float voltageAREF;
  float maxCountsADC;
  float countsToVolts;
  
  dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
  dmacdescriptor descriptor __attribute__ ((aligned (16)));

};

#endif



#endif
