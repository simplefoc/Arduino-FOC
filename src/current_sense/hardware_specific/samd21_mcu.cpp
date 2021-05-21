
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
  // instance.startADCScan();
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

void SAMDCurrentSenseADCDMA::init(int pinA, int pinB, int pinC
, int pinAREF, float voltageAREF
, uint32_t adcBits, uint32_t channelDMA)
{
  this->pinA = pinA;
  this->pinB = pinB;
  this->pinC = pinC;
  this->pinAREF = pinAREF;
  this->channelDMA = channelDMA;
  this->voltageAREF = voltageAREF;
  this->maxCountsADC = 1 << adcBits;
  countsToVolts = ( voltageAREF / maxCountsADC );

  for(static int i = 0; i < 20; i++)
    adcBuffer[i] = 0;
  initPins();
  initADC();
  // initDMA();
  // startADCScan(); //so we have something to read next time we call readResults()


}


void SAMDCurrentSenseADCDMA::startADCScan(){
  // adcToDMATransfer(adcBuffer + oneBeforeFirstAIN, BufferSize);
  // adcStartWithDMA();
}

bool SAMDCurrentSenseADCDMA::readResults(uint16_t & a, uint16_t & b, uint16_t & c){
  // if(ADC->CTRLA.bit.ENABLE)
  //   return false;
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

  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC

	/* Turn on the digital interface clock */
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	/* Turn on the analog interface clock and use GCLK Generator
	 */
	GCLK_CLKCTRL_Type adc_clkctrl{.reg = 0};
	adc_clkctrl.bit.WRTLOCK = 0;
	adc_clkctrl.bit.CLKEN = 1;  /* enable clock */
	adc_clkctrl.bit.ID = ADC_GCLK_ID;
	adc_clkctrl.bit.GEN = 0;   /* GCLK_GENERATOR_0 */
	GCLK->CLKCTRL.reg = adc_clkctrl.reg;

	/* reset the ADC to its initial settings */
	ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  ADCsync();

	/* Configure reference */
	ADC_REFCTRL_Type refctrl{.reg = 0};
  // refctrl.bit.REFCOMP = 1; /* enable reference compensation */
  refctrl.bit.REFSEL = this->pinAREF == 42 ? ADC_REFCTRL_REFSEL_AREFA_Val : ADC_REFCTRL_REFSEL_INTVCC1_Val;
  
	ADC->REFCTRL.reg = refctrl.reg;

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
  [...]
  0x13 PIN19 ADC AIN19 pin
  0x14-0x17 Reserved
  0x18 TEMP Temperature reference
  0x19 BANDGAP Bandgap voltage
  0x1A SCALEDCOREVCC 1/4 scaled core supply
  0x1B SCALEDIOVCC 1/4 scaled I/O supply
  0x1C DAC DAC output
  0x1D-0x1F Reserved
  */
  ADC_INPUTCTRL_Type inputctrl{.reg = 0};
  inputctrl.bit.GAIN = refctrl.bit.REFSEL == ADC_REFCTRL_REFSEL_AREFA_Val ? ADC_INPUTCTRL_GAIN_1X_Val : ADC_INPUTCTRL_GAIN_DIV2_Val;
  inputctrl.bit.MUXPOS = firstAIN;
  inputctrl.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
  inputctrl.bit.INPUTSCAN = lastAIN - inputctrl.bit.MUXPOS + 1; // so the adc will scan from oneBeforeFirstAIN to lastAIN (inclusive) 
  inputctrl.bit.INPUTOFFSET = 0; //input scan cursor
  ADC->INPUTCTRL.reg = inputctrl.reg;
  ADCsync();

  	/* Set up the average and samples */
	ADC_AVGCTRL_Type avgctrl{.reg = 0};
	avgctrl.bit.ADJRES = 0,
	avgctrl.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1_Val,
	ADC->AVGCTRL.reg = avgctrl.reg;
	/* Configure sample length */
	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(5); //sample length in 1/2 CLK_ADC cycles, see GCLK_ADC and ADC_CTRLB_PRESCALER_DIV16
  // according to the specsheet: fGCLK_ADC ADC input clock frequency 48 MHz, so same as fCPU
  ADCsync();
  
  ADC_CTRLB_Type ctrlb{.reg = 0};
  /* ADC clock is 8MHz / 4 = 2MHz */
  ctrlb.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV16_Val;
  ctrlb.bit.RESSEL =  ADC_CTRLB_RESSEL_12BIT_Val;
  ctrlb.bit.CORREN = 0;
  ctrlb.bit.FREERUN = 0;
  ctrlb.bit.LEFTADJ = 0;
  ctrlb.bit.DIFFMODE = 0;

  ADC->CTRLB.reg = ctrlb.reg;
  ADCsync();

  ADC_EVCTRL_Type adc_evctrl{.reg = 0};
	adc_evctrl.bit.WINMONEO = 0;
	adc_evctrl.bit.RESRDYEO = 0;
	adc_evctrl.bit.SYNCEI = 1;
	adc_evctrl.bit.STARTEI = 1;
	ADC->EVCTRL.reg = adc_evctrl.reg;

  ADC->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ( ADC_IRQn );
  ADC->CTRLA.bit.ENABLE = 0x01;

}

// volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));

// void SAMDCurrentSenseADCDMA::initDMA() {
//   // probably on by default
//   PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
//   PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
//   NVIC_EnableIRQ( DMAC_IRQn ) ;
//   DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
//   DMAC->WRBADDR.reg = (uint32_t)wrb;
//   DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
// }


// void SAMDCurrentSenseADCDMA::adcToDMATransfer(void *rxdata,  uint32_t hwords) {

//   DMAC->CHID.reg = DMAC_CHID_ID(channelDMA);
//   DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
//   DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
//   DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << channelDMA));
  
//   DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) 
//   | DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) 
//   | DMAC_CHCTRLB_TRIGACT_BEAT;
//   DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
//   descriptor.descaddr = 0;
//   descriptor.srcaddr = (uint32_t) &ADC->RESULT.reg;
//   descriptor.btcnt =  hwords;
//   descriptor.dstaddr = (uint32_t)rxdata + hwords*2;   // end address
//   descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
//   memcpy(&descriptor_section[channelDMA],&descriptor, sizeof(dmacdescriptor));

//   // start channel
//   DMAC->CHID.reg = DMAC_CHID_ID(channelDMA);
//   DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
// }


// int iii = 0;

// void adcStopWithDMA(void){
//   ADCsync();
//   ADC->CTRLA.bit.ENABLE = 0x00;
//   // ADCsync();
//   // if(iii++ % 1000 == 0)
//   // {
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(a);
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" :: ");
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(b);
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" :: ");
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.print(c);
//   //   SIMPLEFOC_SAMD_DEBUG_SERIAL.println("yo!");
//   // }


// }

void ADC_Handler()
{

  instance.adcBuffer[ADC->INPUTCTRL.bit.MUXPOS + ADC->INPUTCTRL.bit.INPUTOFFSET - 1] = ADC->RESULT.reg;

  // instance.adcBuffer[i++%4] = ADC->RESULT.reg;
}

// void adcStartWithDMA(void){
//   ADCsync();
//   ADC->INPUTCTRL.bit.INPUTOFFSET = 0;
//   ADCsync();
//   ADC->SWTRIG.bit.FLUSH = 1;
//   ADCsync();
//   ADC->CTRLA.bit.ENABLE = 0x01; 
//   ADCsync();
// }

// void DMAC_Handler() {
//   uint8_t active_channel;
//   active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
//   DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
//   // adcStopWithDMA(); no need to stop it anymore it's not freerunning
//   DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
//   DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
//   DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;

// }