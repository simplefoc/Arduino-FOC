
#include "samd21_mcu.h"


void adc_stop_with_DMA(void);
void adc_start_with_DMA(void);

/**
 * @brief  ADC sync wait 
 * @retval void
 */
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

//  ADC DMA sequential free running (6) with Interrupts /////////////////

typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor ;
volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));
DmacDescriptor *desc; // DMA descriptor address (so we can change contents)



SAMDCurrentSensceADC::SAMDCurrentSensceADC(int pinA, int pinB, int pinC, float arefaVoltage, uint32_t adcBits) 
: _ADC_VOLTAGE(arefaVoltage), _ADC_RESOLUTION(1 << adcBits)
{
  ADC_CONV_ = ( _ADC_VOLTAGE / _ADC_RESOLUTION );
  this->pinA = pinA;
  this->pinB = pinB;
  this->pinC = pinC;
}

void SAMDCurrentSensceADC::init()
{
  _configure3PinsDMA();
  _start3PinsDMA(); //s
}


void SAMDCurrentSensceADC::_start3PinsDMA()
{
  adc_dma(adcBuffer + ADC_OneBeforeFirstPin, BufferSize);
  adc_start_with_DMA();
}
void SAMDCurrentSensceADC::_read3PinsDMA(const int pinA,const int pinB,const int pinC, float & a, float & b, float & c)
{
  while(ADC->CTRLA.bit.ENABLE) ;
  uint32_t adcA = g_APinDescription[pinA].ulADCChannelNumber;
  uint32_t adcB = g_APinDescription[pinB].ulADCChannelNumber;
  a = adcBuffer[adcA] * ADC_CONV_;
  b = adcBuffer[adcB] * ADC_CONV_;
  if(_isset(pinC))
  {
    uint32_t adcC = g_APinDescription[pinC].ulADCChannelNumber;
    c = adcBuffer[adcC] * ADC_CONV_;
  }
}

// function reading an ADC value and returning the read voltage
void SAMDCurrentSensceADC::_configure3PinsDMA(){
  
  uint32_t adcA = g_APinDescription[pinA].ulADCChannelNumber;
  uint32_t adcB = g_APinDescription[pinB].ulADCChannelNumber;
  uint32_t adcC = g_APinDescription[pinC].ulADCChannelNumber;

  pinMode(42, INPUT);
  ADC_FirstPin = min(adcA, adcB);
  ADC_LastPin = max(adcA, adcB);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) 
  {
    pinMode(pinC, INPUT);
    ADC_FirstPin = min(ADC_FirstPin, adcC);
    ADC_LastPin = max(ADC_LastPin, adcC);
  }

  ADC_OneBeforeFirstPin = ADC_FirstPin - 1; //hack to discard noisy first readout
  BufferSize = ADC_LastPin - ADC_OneBeforeFirstPin + 1;

  // ADC and DMA
  adc_init();
  dma_init();
}





/**
 * @brief  Initialize ADC
 * @retval void
 */
void SAMDCurrentSensceADC::adc_init(){

  analogRead(pinA);  // do some pin init  pinPeripheral() 
  analogRead(pinB);  // do some pin init  pinPeripheral() 
  analogRead(pinC);  // do some pin init  pinPeripheral() 

  ADC->CTRLA.bit.ENABLE = 0x00; // Disable ADC
  ADCsync();
  //ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val; // Gain select as 1X
  // ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // default
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA;
  // ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0;
  ADCsync();  //  ref 31.6.16

  /*
  Bits 19:16 – INPUTSCAN[3:0]: Number of Input Channels Included in Scan
  This register gives the number of input sources included in the pin scan. The number of input sources included is
  INPUTSCAN + 1. The input channels included are in the range from MUXPOS + INPUTOFFSET to MUXPOS +
  INPUTOFFSET + INPUTSCAN.
  The range of the scan mode must not exceed the number of input channels available on the device.
  Bits 4:0 – MUXPOS[4:0]: Positive Mux Input Selection
  These bits define the Mux selection for the positive ADC input. Table 32-14 shows the possible input selections. If
  the internal bandgap voltage or temperature sensor input channel is selected, then the Sampling Time Length bit
  group in the SamplingControl register must be written.
  Table 32-14. Positive Mux Input Selection
  MUXPOS[4:0] Group configuration Description
  0x00 PIN0 ADC AIN0 pin
  0x01 PIN1 ADC AIN1 pin
  0x02 PIN2 ADC AIN2 pin
  0x03 PIN3 ADC AIN3 pin
  0x04 PIN4 ADC AIN4 pin
  0x05 PIN5 ADC AIN5 pin
  0x06 PIN6 ADC AIN6 pin
  0x07 PIN7 ADC AIN7 pin
  0x08 PIN8 ADC AIN8 pin
  0x09 PIN9 ADC AIN9 pin
  0x0A PIN10 ADC AIN10 pin
  0x0B PIN11 ADC AIN11 pin
  0x0C PIN12 ADC AIN12 pin
  0x0D PIN13 ADC AIN13 pin
  0x0E PIN14 ADC AIN14 pin
  0x0F PIN15 ADC AIN15 pin
  0x10 PIN16 ADC AIN16 pin
  0x11 PIN17 ADC AIN17 pin
  0x12 PIN18 ADC AIN18 pin
  0x13 PIN19 ADC AIN19 pin
  0x14-0x17 Reserved
  0x18 TEMP Temperature reference
  0x19 BANDGAP Bandgap voltage
  0x1A SCALEDCOREVCC 1/4 scaled core supply
  0x1B SCALEDIOVCC 1/4 scaled I/O supply
  0x1C DAC DAC output
  0x1D-0x1F Reserved
  */
  ADC->INPUTCTRL.bit.MUXPOS = ADC_OneBeforeFirstPin;
  ADCsync();
  ADC->INPUTCTRL.bit.INPUTSCAN = ADC_LastPin; // so the adc will scan from AIN[1] to AIN[ADC_Number+1] 
  ADCsync();
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0; //input scan cursor
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;  //no averaging
  ADC->SAMPCTRL.reg = 0x05;  ; //sample length in 1/2 CLK_ADC cycles, see GCLK_ADC and ADC_CTRLB_PRESCALER_DIV16
  // according to the specsheet: f_GCLK_ADC ADC input clock frequency 48 MHz, so same as fCPU
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_12BIT;
  ADCsync();
}


/**
 * @brief  dma_init
 * @retval void
 */
void SAMDCurrentSensceADC::dma_init() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
  NVIC_EnableIRQ( DMAC_IRQn ) ;
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}

/**
 * @brief  adc_dma 
 * @retval void
 */
void SAMDCurrentSensceADC::adc_dma(void *rxdata,  size_t hwords) {
  uint32_t temp_CHCTRLB_reg;

  DMAC->CHID.reg = DMAC_CHID_ID(ADC_DMA_chnl);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << ADC_DMA_chnl));
  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
  DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
  descriptor.descaddr = 0;
  descriptor.srcaddr = (uint32_t) &ADC->RESULT.reg;
  descriptor.btcnt =  hwords;
  descriptor.dstaddr = (uint32_t)rxdata + hwords*2;   // end address
  descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
  memcpy(&descriptor_section[ADC_DMA_chnl],&descriptor, sizeof(dmacdescriptor));

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(ADC_DMA_chnl);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}




/**
 * @brief  adc_stop_with_DMA
 * @retval void
 */
void adc_stop_with_DMA(void)
{
  ADC->CTRLA.bit.ENABLE = 0x00;
  // SerialUSB.println("DMA stopped!");
}

/**
 * @brief  adc_start_with_DMA
 * @retval void
 */
void adc_start_with_DMA(void)
{
  // SerialUSB.println("strating DMA...");
  // ADC->INPUTCTRL.bit.MUXPOS = ADC_OneBeforeFirstPin;
  // ADC->INPUTCTRL.bit.INPUTSCAN = ADC_LastPin;
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0;
  ADC->SWTRIG.bit.FLUSH = 1;
  ADC->CTRLA.bit.ENABLE = 0x01; 
}
/**
 * @brief  DMAC_Handler
 * @retval void
 */
void DMAC_Handler() {
  uint8_t active_channel;
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  adc_stop_with_DMA();
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
}