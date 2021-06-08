#include "../hardware_api.h"

#if defined(_SAMD21_ASYNC_)

#include "samd21_mcu.h"

#include "../../common/hardware_specific/samd_mcu.h"
#include <wiring_private.h>
static int _pinA, _pinB, _pinC;
static uint16_t a_raw = 0xFFFF, b_raw = 0xFFFF, c_raw = 0xFFFF; // updated by adcStopWithDMA when configured in freerunning mode
static SAMD21AsyncCurrentSense instance;
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
  instance.init(pinA, pinB, pinC, EVSYS_ID_GEN_TCC0_OVF);

}
/**
 *  function reading an ADC value and returning the read voltage
 * 
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
bool _readADCVoltagesLowSide(float & a, float & b, float & c)
{
  static uint64_t last_ts = _micros();
  bool isNew = false;
  if(last_ts != instance.timestamp_us)
  {
    last_ts = instance.timestamp_us;
    instance.readResults(a_raw, b_raw, c_raw);
    isNew = true;
  }
  a = instance.toVolts(a_raw);
  b = instance.toVolts(b_raw);
  if(_isset(_pinC))
    c = instance.toVolts(c_raw);

  return isNew;
}

/**
 *  function syncing the Driver with the ADC  for the LowSide Sensing
 */
void _driverSyncLowSide()
{
  debugPrintln(F("TODO! _driverSyncLowSide() is not untested, but if you use EVSYS_ID_GEN_TCC_OVF != -1 you are in sync"));
  //TODO: hook with PWM interrupts
}










 // Credit: significant portions of this code were pulled from Paul Gould's git https://github.com/gouldpa/FOC-Arduino-Brushless


/**
 * @brief  ADC sync wait 
 * @retval void
 */
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

//  ADC DMA sequential free running (6) with Interrupts /////////////////

SAMD21AsyncCurrentSense * SAMD21AsyncCurrentSense::getHardwareAPIInstance() 
{
  
  return &instance;
}

SAMD21AsyncCurrentSense::SAMD21AsyncCurrentSense()
{
}

void SAMD21AsyncCurrentSense::init(int pinA, int pinB, int pinC,
int EVSYS_ID_GEN_TCC_OVF, int pinAREF, float voltageAREF
, uint8_t adcBits, uint8_t channelDMA)
{
  this->pinA = pinA;
  this->pinB = pinB;
  this->pinC = pinC;
  this->pinAREF = pinAREF;
  this->channelDMA = channelDMA;
  this->voltageAREF = voltageAREF;
  this->maxCountsADC = 1 << adcBits;
  this->EVSYS_ID_GEN_TCC_OVF = EVSYS_ID_GEN_TCC_OVF;
  countsToVolts = ( voltageAREF / maxCountsADC );

  for(static int i = 0; i < 20; i++)
    adcBuffer[i] = 42;
  initPins();
  if(this->EVSYS_ID_GEN_TCC_OVF != -1)
  {
    debugPrintf(PSTR("Configuring EVSYS for ADC with EVSYS_ID_GEN #%d, DMA %d\n\r"), this->EVSYS_ID_GEN_TCC_OVF, channelDMA);

    
    if(initEVSYS() != 0)
      return;
    initDMA();

    initDMAChannel();
  }
  initADC();

}


uint32_t SAMD21AsyncCurrentSense::readResults(uint16_t & a, uint16_t & b, uint16_t & c){

  a = adcBuffer[ainA];
  b = adcBuffer[ainB];
  if(_isset(pinC))
    c = adcBuffer[ainC];
  return 0; // TODO: return timestamp
}


float SAMD21AsyncCurrentSense::toVolts(uint16_t counts) {
  return counts * countsToVolts;
}

void SAMD21AsyncCurrentSense::initPins(){
  
  refsel = ADC_REFCTRL_REFSEL_INTVCC1_Val;
  if(pinAREF != -1)
  {
    debugPrintf(PSTR("Using AREF pin %d"), pinAREF);
    switch(getSamdPinDefinition(pinAREF)->vref)
    {
      case VRef::VREFA:
        refsel =  ADC_REFCTRL_REFSEL_AREFA_Val;
        break;
      case VRef::VREFB:
        refsel = ADC_REFCTRL_REFSEL_AREFB_Val;
        break;
      default:
        debugPrintf(PSTR("Error: pin %d is not a valid AREF pin, falling back to 'INTVCC1'"), pinAREF);
    }
  }

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  ainA = getSamdPinDefinition(pinA)->adc_channel;
  ainB = getSamdPinDefinition(pinB)->adc_channel;
  firstAIN = min(ainA, ainB);
  lastAIN = max(ainA, ainB);
  if( _isset(pinC) ) 
  {
    ainC = getSamdPinDefinition(pinC)->adc_channel;
    pinMode(pinC, INPUT);
    firstAIN = min(firstAIN, ainC);
    lastAIN = max(lastAIN, ainC);
  }

  oneBeforeFirstAIN = firstAIN - 1; //hack to discard noisy first readout
  bufferSize = lastAIN - oneBeforeFirstAIN + 1;

}

void SAMD21AsyncCurrentSense::initADC(){

  pinPeripheral(pinA, PIO_ANALOG); 
  pinPeripheral(pinB, PIO_ANALOG); 
  pinPeripheral(pinC, PIO_ANALOG); 

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


  /* Load the factory calibration data. */

  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

	/* Configure reference */
	ADC_REFCTRL_Type refctrl{.reg = 0};
  // refctrl.bit.REFCOMP = 1; /* enable reference compensation */
  refctrl.bit.REFSEL = refsel;
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
  inputctrl.bit.GAIN = refctrl.bit.REFSEL == ADC_REFCTRL_REFSEL_INTVCC1_Val ? ADC_INPUTCTRL_GAIN_DIV2_Val : ADC_INPUTCTRL_GAIN_1X_Val;
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
	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(5); //sample length in 1/2 CLK_ADC cycles, see GCLK_ADC and ADC_CTRLB_PRESCALER_DIV4_Val
  // according to the specsheet: fGCLK_ADC ADC input clock frequency 48 MHz, so same as fCPU
  ADCsync();
  
  ADC_CTRLB_Type ctrlb{.reg = 0};
  /* ADC clock is 8MHz / 4 = 2MHz */
  ctrlb.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4_Val;
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

  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}

void SAMD21AsyncCurrentSense::initDMA() {
  ::initDMAC();
}

DmacDescriptor descriptors[20] __attribute__ ((aligned (16)));

void SAMD21AsyncCurrentSense::initDMAChannel() {

  DMAC_CHINTENSET_Type chinset{.reg = 0};

  chinset.bit.TCMPL = 0b1;

  // configure the channel
  DMAC_CHCTRLB_Type chctrlb{.reg = 0};
  chctrlb.bit.LVL = DMAC_CHCTRLB_LVL_LVL0_Val;
  chctrlb.bit.EVIE = 0b0; //input (USER) event enabled?
  chctrlb.bit.EVOE = 0b1; //output (GEN) event enabled?
  chctrlb.bit.EVACT = DMAC_CHCTRLB_EVACT_NOACT_Val; //only used if EVIE is set
  chctrlb.bit.TRIGSRC = ADC_DMAC_ID_RESRDY;
  chctrlb.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val; //block, beat or transaction
  DMAC->CHCTRLB.reg = chctrlb.reg;

  for(uint32_t i = firstAIN; i < lastAIN + 1; i++)
  {
    bool isLast = (i == lastAIN);
    
    descriptors[i].DESCADDR.reg = isLast ? 0 : (uint32_t)&descriptors[i+1];
    descriptors[i].SRCADDR.reg  = (uint32_t) &ADC->RESULT.reg;
    descriptors[i].BTCNT.reg    = 1;

    volatile DMAC_BTCTRL_Type & btctrl = descriptors[i].BTCTRL;
    btctrl.bit.VALID = 0b1;                                     /*!< bit:      0  Descriptor Valid                   */
    
    //we want no events after the last adc read
    btctrl.bit.EVOSEL = isLast ? DMAC_BTCTRL_EVOSEL_DISABLE_Val : DMAC_BTCTRL_EVOSEL_BLOCK_Val;            /*!< bit:  1.. 2  Event Output Selection             */
    
    btctrl.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val; //isLast ? DMAC_BTCTRL_BLOCKACT_NOACT_Val : DMAC_BTCTRL_BLOCKACT_INT_Val;       /*!< bit:  3.. 4  Block Action                       */
    btctrl.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_HWORD_Val;       /*!< bit:  8.. 9  Beat Size (byte, half-words, words) */
    btctrl.bit.SRCINC = 0b0;                                    /*!< bit:     10  Source Address Increment Enable    */
    btctrl.bit.DSTINC = 0b1;                                    /*!< bit:     11  Destination Address Increment Enable */
    btctrl.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;           /*!< bit:     12  Step Selection                     */
    btctrl.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;          /*!< bit: 13..15  Address Increment Step Size        */
    descriptors[i].DSTADDR.reg  = computeDSTADDR((uint8_t*)&adcBuffer[i], btctrl.bit.STEPSEL, btctrl.bit.STEPSIZE, btctrl.bit.BEATSIZE, descriptors[i].BTCNT.reg);
  }

  ::initDMAChannel(channelDMA, chinset, chctrlb, descriptors[firstAIN], this);
  
}

void SAMD21AsyncCurrentSense::operator()(uint8_t channel, volatile DMAC_CHINTFLAG_Type & flags, volatile DMAC_CHCTRLA_Type & chctrla) 
{
  chctrla.bit.ENABLE = 0b1;
  flags.bit.TCMPL = 0b1;
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0;
  timestamp_us = _micros();
}

int SAMD21AsyncCurrentSense::initEVSYS()
{
  if(::initEVSYS(0, EVSYS_ID_USER_ADC_SYNC, this->EVSYS_ID_GEN_TCC_OVF, EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val, EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val) != 0)
    return -1;
  
  uint16_t EVGEN_DMAC = 0;
  switch(channelDMA)
  {
    case 0: EVGEN_DMAC = EVSYS_ID_GEN_DMAC_CH_0; break;
    case 1: EVGEN_DMAC = EVSYS_ID_GEN_DMAC_CH_1; break;
    case 2: EVGEN_DMAC = EVSYS_ID_GEN_DMAC_CH_2; break;
    case 3: EVGEN_DMAC = EVSYS_ID_GEN_DMAC_CH_3; break;
    default:
      debugPrintf(PSTR("initEVSYS(): Bad dma channel %u. Only 0,1,2 or 3 are supported."), channelDMA);
      break;
  }
  if(::initEVSYS(1, EVSYS_ID_USER_ADC_START, EVGEN_DMAC, EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val, EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val) != 0)
    return -1;

  return 0;
}


#endif