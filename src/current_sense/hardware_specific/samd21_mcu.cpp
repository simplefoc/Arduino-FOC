
#include "samd21_mcu.h"
#include "../hardware_api.h"


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
  instance.init(pinA, pinB, pinC, EVSYS_ID_GEN_TCC0_OVF);

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
  SIMPLEFOC_SAMD_DEBUG_SERIAL.println(F("TODO! _driverSyncLowSide() is not untested, but if you use EVSYS_ID_GEN_TCC_OVF != -1 you are in sync"));
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

SAMDCurrentSenseADCDMA * SAMDCurrentSenseADCDMA::getHardwareAPIInstance() 
{
  
  return &instance;
}

SAMDCurrentSenseADCDMA::SAMDCurrentSenseADCDMA()
{
}

void SAMDCurrentSenseADCDMA::init(int pinA, int pinB, int pinC,
int EVSYS_ID_GEN_TCC_OVF, int pinAREF, float voltageAREF
, uint32_t adcBits, uint32_t channelDMA)
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
    #ifdef SIMPLEFOC_SAMD_DEBUG
		  SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Configuring EVSYS for ADC with EVSYS_ID_GEN_TCC_OVF");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println(this->EVSYS_ID_GEN_TCC_OVF);
		#endif

    initEVSYS();
    initDMA();
    initDMAChannel();
  }
  initADC();

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
  
  if(pinAREF != -1)
  {
    SIMPLEFOC_SAMD_DEBUG_SERIAL.print("wtf");
    if(g_APinDescription[pinAREF].ulPort == EPortType::PORTA && g_APinDescription[pinAREF].ulPin == 3)
      pinMode(pinAREF, INPUT);
  }

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
  bufferSize = lastAIN - oneBeforeFirstAIN + 1;

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


  /* Load the factory calibration data. */

  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

	/* Configure reference */
	ADC_REFCTRL_Type refctrl{.reg = 0};
  // refctrl.bit.REFCOMP = 1; /* enable reference compensation */
  refctrl.bit.REFSEL = this->pinAREF != -1 ? ADC_REFCTRL_REFSEL_AREFA_Val : ADC_REFCTRL_REFSEL_INTVCC1_Val;
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

  // if(this->EVSYS_ID_GEN_TCC_OVF == -1)
  // {
    //not evsys + dma driven
    ADC->INTENSET.bit.RESRDY = 1;
    NVIC_EnableIRQ( ADC_IRQn );
  // }
  ADC->CTRLA.bit.ENABLE = 0x01;

}

DmacDescriptor descriptor_section[12] __attribute__ ((aligned (16)));
volatile DmacDescriptor write_back[12] __attribute__ ((aligned (16)));

void SAMDCurrentSenseADCDMA::initDMA() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
  
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section; // Descriptor Base Memory Address
  DMAC->WRBADDR.reg = (uint32_t)write_back; //Write-Back Memory Base Address


  DMAC_PRICTRL0_Type prictrl0{.reg = 0};

  prictrl0.bit.RRLVLEN0 = 0b1; //enable round-robin
  prictrl0.bit.RRLVLEN1 = 0b1; //enable round-robin
  prictrl0.bit.RRLVLEN2 = 0b1; //enable round-robin
  prictrl0.bit.RRLVLEN3 = 0b1; //enable round-robin

  DMAC->PRICTRL0.reg = prictrl0.reg;

  DMAC_CTRL_Type ctrl{.reg = 0};
  ctrl.bit.DMAENABLE = 0b1;
  ctrl.bit.LVLEN0 = 0b1;
  ctrl.bit.LVLEN1 = 0b1;
  ctrl.bit.LVLEN2 = 0b1;
  ctrl.bit.LVLEN3 = 0b1;
  ctrl.bit.CRCENABLE = 0b0;

  DMAC->CTRL.reg = ctrl.reg;

  NVIC_EnableIRQ( DMAC_IRQn ) ;
}

DmacDescriptor descriptors[20] __attribute__ ((aligned (16)));

void SAMDCurrentSenseADCDMA::initDMAChannel() {

  //select the channel
  DMAC->CHID.bit.ID = channelDMA;

  // disable and reset the channel
  DMAC->CHCTRLA.bit.ENABLE = 0b0; //must be done **before** SWRST
  DMAC->CHCTRLA.bit.SWRST = 0b1;

  DMAC->CHINTENSET.bit.TCMPL = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts

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
    /*
    p.283 When destination address incrementation is configured (BTCTRL.DSTINC is one), SRCADDR must be set to the 
    destination address of the last beat transfer in the block transfer. The destination address should be calculated as 
    follows: 
    DSTADDR = DSTADDRSTART + BTCNT ⋅ ( BEATSIZE + 1 ) ⋅ 2 STEPSIZE , where BTCTRL.STEPSEL is zero 
    DSTADDR = DSTADDRSTART + BTCNT ⋅ ( BEATSIZE + 1 ) , where BTCTRL.STEPSEL is one 
    -  DSTADDRSTART is the destination address of the first beat transfer in the block transfer 
    -  BTCNT is the initial number of beats remaining in the block transfer 
    -  BEATSIZE is the configured number of bytes in a beat 
    -  STEPSIZE is the configured number of beats for each incrementation 
    */
    uint32_t factor  = btctrl.bit.STEPSEL == 0 ? (1 << btctrl.bit.STEPSIZE) /*2^STEPSIZE*/: 1;

    descriptors[i].DSTADDR.reg  = (uint32_t)(((uint8_t*)&adcBuffer[i]) + descriptors[i].BTCNT.reg * (btctrl.bit.BEATSIZE + 1) * factor);   // end address

  }
  
  memcpy(&descriptor_section[channelDMA], &descriptors[firstAIN], sizeof(DmacDescriptor));
  
  // start channel
  DMAC->CHCTRLA.bit.ENABLE = 0b1;
}

void SAMDCurrentSenseADCDMA::initEVSYS()
{
  	/* Turn on the digital interface clock */
	PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

	/* Turn on the peripheral interface clock and select GCLK */
	GCLK_CLKCTRL_Type clkctrl0;
  clkctrl0.bit.WRTLOCK = 0;
  clkctrl0.bit.CLKEN = 1;
  clkctrl0.bit.ID = EVSYS_GCLK_ID_0; //enable clock for channel 0
  clkctrl0.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val; /* GCLK_GENERATOR_0 */
	GCLK->CLKCTRL.reg = clkctrl0.reg;

  GCLK_CLKCTRL_Type clkctrl1;
  clkctrl1.bit.WRTLOCK = 0;
  clkctrl1.bit.CLKEN = 1;
  clkctrl1.bit.ID = EVSYS_GCLK_ID_1; //enable clock for channel 1
  clkctrl1.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val; /* GCLK_GENERATOR_0 */
	GCLK->CLKCTRL.reg = clkctrl1.reg;

  // event user (destination)
	EVSYS_USER_Type user_0;
		user_0.bit.CHANNEL = 0 + 1; /* use channel 0 p421: "Note that to select channel n, the value (n+1) must be written to the USER.CHANNEL bit group." */
		user_0.bit.USER = EVSYS_ID_USER_ADC_SYNC; /* ADC Sync*/
	EVSYS->USER.reg = user_0.reg;

  // event user (destination)
  EVSYS_USER_Type user_1;
  user_1.bit.CHANNEL   = 1 + 1; /* p421: "Note that to select channel n, the value (n+1) must be written to the USER.CHANNEL bit group." */
  // user_1.bit.USER = EVSYS_ID_USER_DMAC_CH_0; /* ADC Start*/
  user_1.bit.USER = EVSYS_ID_USER_ADC_START; /* ADC Start*/
  EVSYS->USER.reg = user_1.reg;

  // event generator (source)
	EVSYS_CHANNEL_Type channel_0;
	channel_0.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val;
	channel_0.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val;
	channel_0.bit.EVGEN = this->EVSYS_ID_GEN_TCC_OVF; /* TCCO Timer OVF */
	channel_0.bit.SWEVT = 0b0;   /* no software trigger */
	channel_0.bit.CHANNEL = user_0.bit.CHANNEL - 1; /* use channel 0 */
	EVSYS->CHANNEL.reg = channel_0.reg;

  // event generator (source)
  EVSYS_CHANNEL_Type channel_1{.reg = 0};
  channel_1.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val;
  channel_1.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val;
  channel_1.bit.EVGEN = EVSYS_ID_GEN_DMAC_CH_0;
  channel_1.bit.SWEVT = 0b0;   
  channel_1.bit.CHANNEL = user_1.bit.CHANNEL - 1; 
  EVSYS->CHANNEL.reg = channel_1.reg;


  // EVSYS does not emit interrups on asynchronous path
  // EVSYS->INTENSET.reg = EVSYS_INTENSET_MASK;
  // NVIC_EnableIRQ( EVSYS_IRQn ) ;
}

void ADC_Handler() __attribute__((weak));
void ADC_Handler()
{
  instance.adc_i++;
  // only used if event system is not used
  if(instance.EVSYS_ID_GEN_TCC_OVF == -1)
    instance.adcBuffer[ADC->INPUTCTRL.bit.MUXPOS + ADC->INPUTCTRL.bit.INPUTOFFSET - 1] = ADC->RESULT.reg;
  ADC->INTFLAG.bit.RESRDY = 0b1;
  ADC->INTFLAG.bit.SYNCRDY = 0b1;
}

void DMAC_Handler() __attribute__((weak));
void DMAC_Handler() {

  instance.dma_i++;
  DMAC->CHID.bit.ID = DMAC->INTPEND.bit.ID;
  DMAC->CHINTFLAG.bit.TCMPL = 0b1; // clear
  DMAC->CHCTRLA.bit.ENABLE = 0b1;
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0;

}