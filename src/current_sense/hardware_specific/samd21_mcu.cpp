#ifdef _SAMD21_

#include "samd21_mcu.h"
#include "../hardware_api.h"


static bool freeRunning = false;
static int _pinA, _pinB, _pinC;
static uint16_t a = 0xFFFF, b = 0xFFFF, c = 0xFFFF; // updated by adcStopWithDMA when configured in freerunning mode
static SAMDCurrentSenseADCDMA instance;
/**
 *  function reading an ADC value and returning the read voltage 
 * 
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void _configureADCLowSide(const int pinA,const int pinB,const int pinC)
{
  _pinA = pinA;
  _pinB = pinB; 
  _pinC = pinC;
  freeRunning = true;
  instance.init(pinA, pinB, pinC);

}
void _startADC3PinConversionLowSide()
{
  instance.startADCScan();
}
/**
 *  function reading an ADC value and returning the read voltage
 * 
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
float _readADCVoltageLowSide(const int pinA)
{
  instance.readResults(a, b, c);
  
  if(pinA == _pinA)
    return instance.toVolts(a);
  if(pinA == _pinB)
    return instance.toVolts(b);
  if(pinA == _pinC)
    return instance.toVolts(c);

  return NAN;
}

/**
 *  function syncing the Driver with the ADC  for the LowSide Sensing
 */
void _driverSyncLowSide()
{
  SIMPLEFOC_SAMD_DEBUG_SERIAL.println(F("TODO! _driverSyncLowSide() is not implemented"));
  instance.startADCScan();
  //TODO: hook with PWM interrupts
}










 // Credit: significant portions of this code were pulled from Paul Gould's git https://github.com/gouldpa/FOC-Arduino-Brushless

static void adcStopWithDMA(void);
static void adcStartWithDMA(void);

/**
 * @brief  ADC sync wait 
 * @retval void
 */
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

//  ADC DMA sequential free running (6) with Interrupts /////////////////

SAMDCurrentSenseADCDMA * SAMDCurrentSenseADCDMA::getHardwareAPIInstance() 
{
  
  return &instance;
}

SAMDCurrentSenseADCDMA::SAMDCurrentSenseADCDMA()
{
}

void SAMDCurrentSenseADCDMA::init(int pinA, int pinB, int pinC, int pinAREF, float voltageAREF, uint32_t adcBits, uint32_t channelDMA)
{
  this->pinA = pinA;
  this->pinB = pinB;
  this->pinC = pinC;
  this->pinAREF = pinAREF;
  this->channelDMA = channelDMA;
  this->voltageAREF = voltageAREF;
  this->maxCountsADC = 1 << adcBits;
  countsToVolts = ( voltageAREF / maxCountsADC );

  initPins();
  initADC();
  initDMA();
  startADCScan(); //so we have something to read next time we call readResults()
}


void SAMDCurrentSenseADCDMA::startADCScan(){
  adcToDMATransfer(adcBuffer + oneBeforeFirstAIN, BufferSize);
  adcStartWithDMA();
}

bool SAMDCurrentSenseADCDMA::readResults(uint16_t & a, uint16_t & b, uint16_t & c){
  if(ADC->CTRLA.bit.ENABLE)
    return false;
  uint32_t ainA = g_APinDescription[pinA].ulADCChannelNumber;
  uint32_t ainB = g_APinDescription[pinB].ulADCChannelNumber;
  a = adcBuffer[ainA];
  b = adcBuffer[ainB];
  if(_isset(pinC))
  {
    uint32_t ainC = g_APinDescription[pinC].ulADCChannelNumber;
    c = adcBuffer[ainC];
  }
  return true;
}


float SAMDCurrentSenseADCDMA::toVolts(uint16_t counts) {
  return counts * countsToVolts;
}

void SAMDCurrentSenseADCDMA::initPins(){
  
  if (pinAREF>=0)
    pinMode(pinAREF, INPUT);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  uint32_t ainA = g_APinDescription[pinA].ulADCChannelNumber;
  uint32_t ainB = g_APinDescription[pinB].ulADCChannelNumber;
  firstAIN = min(ainA, ainB);
  lastAIN = max(ainA, ainB);
  if( _isset(pinC) ) 
  {
    uint32_t ainC = g_APinDescription[pinC].ulADCChannelNumber;
    pinMode(pinC, INPUT);
    firstAIN = min(firstAIN, ainC);
    lastAIN = max(lastAIN, ainC);
  }

  oneBeforeFirstAIN = firstAIN - 1; //hack to discard noisy first readout
  BufferSize = lastAIN - oneBeforeFirstAIN + 1;

}

void SAMDCurrentSenseADCDMA::initADC(){

  analogRead(pinA);  // do some pin init  pinPeripheral() 
  analogRead(pinB);  // do some pin init  pinPeripheral() 
  analogRead(pinC);  // do some pin init  pinPeripheral() 

  ADC->CTRLA.bit.ENABLE = 0x00; // Disable ADC
  ADCsync();
  //ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val; // Gain select as 1X
  // ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // default
  if (pinAREF>=0)
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA;
  else
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0;
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
  ADC->INPUTCTRL.bit.MUXPOS = oneBeforeFirstAIN;
  ADCsync();
  ADC->INPUTCTRL.bit.INPUTSCAN = lastAIN; // so the adc will scan from oneBeforeFirstAIN to lastAIN (inclusive) 
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

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));

void SAMDCurrentSenseADCDMA::initDMA() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
  NVIC_EnableIRQ( DMAC_IRQn ) ;
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}


void SAMDCurrentSenseADCDMA::adcToDMATransfer(void *rxdata,  uint32_t hwords) {

  DMAC->CHID.reg = DMAC_CHID_ID(channelDMA);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << channelDMA));
  
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) 
  | DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) 
  | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
  descriptor.descaddr = 0;
  descriptor.srcaddr = (uint32_t) &ADC->RESULT.reg;
  descriptor.btcnt =  hwords;
  descriptor.dstaddr = (uint32_t)rxdata + hwords*2;   // end address
  descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
  memcpy(&descriptor_section[channelDMA],&descriptor, sizeof(dmacdescriptor));

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(channelDMA);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


int iii = 0;

void adcStopWithDMA(void){
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x00;
  // ADCsync();
  // if(iii++ % 1000 == 0)
  // {
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(a);
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" :: ");
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(b);
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" :: ");
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(c);
  //   SIMPLEFOC_SAMD_DEBUG_SERIAL.println("yo!");
  // }


}

void adcStartWithDMA(void){
  ADCsync();
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0;
  ADCsync();
  ADC->SWTRIG.bit.FLUSH = 1;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01; 
  ADCsync();
}

void DMAC_Handler() {
  uint8_t active_channel;
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  adcStopWithDMA();
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;

}


#endif