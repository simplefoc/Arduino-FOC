
#include "../hardware_api.h"
#include <stdint.h>
  typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
  } dmacdescriptor ;

class SAMDCurrentSenseADCDMA
{

public:
  
  SAMDCurrentSenseADCDMA(int pinA, int pinB, int pinC, int pinAREF = 42, float voltageAREF = 3.3, uint32_t adcBits = 12, uint32_t channelDMA = 3);

  void init();
  void startADCScan();
  bool readResults(float & a, float & b, float & c);

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

  float voltageAREF;
  float maxCountsADC;
  float countsToVolts;
  
  dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
  dmacdescriptor descriptor __attribute__ ((aligned (16)));

};
