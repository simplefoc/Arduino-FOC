
#include "../hardware_api.h"


class SAMDCurrentSensceADC
{

public:
  
  SAMDCurrentSensceADC(int pinA, int pinB, int pinC, float arefaVoltage = 3.3, uint32_t adcBits = 12);

  void init();
  // this code was pulled from Paul Gould's git https://github.com/gouldpa/FOC-Arduino-Brushless
  uint32_t ADC_OneBeforeFirstPin; // hack to discard first noisy readout
  uint32_t ADC_FirstPin; // PA04
  uint32_t ADC_LastPin;  // PA06
  uint32_t BufferSize = 0;

  uint16_t adcBuffer[20];


  uint32_t pinA = A4;
  uint32_t pinB = A5;
  uint32_t pinC = 8;

  float _ADC_VOLTAGE;
  float _ADC_RESOLUTION;
  float ADC_CONV_;

  void _start3PinsDMA();
  void _read3PinsDMA(const int pinA,const int pinB,const int pinC, float & a, float & b, float & c);
  // function reading an ADC value and returning the read voltage
  void _configure3PinsDMA();




  uint32_t ADC_DMA_chnl = 3;  // DMA channel


  /**
   * @brief  Initialize ADC
   * @retval void
   */
  void adc_init();


  /**
   * @brief  dma_init
   * @retval void
   */
  void dma_init();

  /**
   * @brief  adc_dma 
   * @retval void
   */
  void adc_dma(void *rxdata,  size_t hwords);

};
