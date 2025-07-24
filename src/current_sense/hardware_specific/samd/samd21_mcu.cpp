
#include "samd21_mcu.h"

#if defined(_SAMD21_)

#include "../../hardware_api.h"
#include "../../../drivers/hardware_specific/samd/samd_mcu.h"

static int _pinA = NOT_SET, _pinB = NOT_SET, _pinC = NOT_SET;
static uint16_t adc_raw[3] = {0, 0, 0};
static uint8_t current_phase = 0;  // which phase we're sampling
static bool is_high_side = true;  // low-side current sense

#define _SAMD21_ADC_VOLTAGE 3.3f
#define _SAMD21_ADC_RESOLUTION 4095.0f

static void setupADCEventTriggerFromDriver(const SAMDHardwareDriverParams *par, const GenericCurrentSenseParams *cs_params) {
  
  // --- Configure ADC module ---
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;  
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_RESSEL_12BIT;
  ADC->CTRLB.bit.FREERUN = 0; 
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[_pinA].ulADCChannelNumber;
  // there must be a way to trigger more than one ADC conversion at a time.
  // SO far I was not able to do it.
  // ADC->INPUTCTRL.bit.INPUTSCAN = 1;   // N = number_of_channels - 1  
  // ADC->INPUTCTRL.bit.INPUTOFFSET = 0;   
  while (ADC->STATUS.bit.SYNCBUSY);

  // Enable event start
  ADC->EVCTRL.bit.STARTEI = 1;
  ADC->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ(ADC_IRQn);

  // --- Configure Event System ---
  uint8_t tcc_num = par->tccPinConfigurations[1]->tcc.tccn; 

  // --- Enable event output on the PWM timer (important!) ---
  Tcc* tcc = nullptr;
  switch (tcc_num) {
    case 0: tcc = TCC0; break;
    case 1: tcc = TCC1; break;
    case 2: tcc = TCC2; break;
    default: tcc = TCC0; break;
  }

  // We are enabling the overflow/underflow event output 
  // This is not ideal as it triggers twice per PWM cycle 
  // So we need to keep track if we are in high-side or low-side current sense
  if (tcc) {
    tcc->CTRLA.bit.ENABLE = 0;
    while (tcc->SYNCBUSY.bit.ENABLE);
    tcc->EVCTRL.reg |= TCC_EVCTRL_OVFEO; 
    tcc->CTRLA.bit.ENABLE = 1;
    while (tcc->SYNCBUSY.bit.ENABLE);
  }

  // Configure the event channel to trigger on the TCC overflow
  // and connect it to the ADC start event
  uint8_t evgen = 0;
  switch (tcc_num) {
    case 0: evgen = EVSYS_ID_GEN_TCC0_OVF; break;
    case 1: evgen = EVSYS_ID_GEN_TCC1_OVF; break;
    case 2: evgen = EVSYS_ID_GEN_TCC2_OVF; break;
    default: evgen = EVSYS_ID_GEN_TCC0_OVF; break;
  }

  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0)
                     | EVSYS_CHANNEL_EVGEN(evgen)
                     | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
  EVSYS->USER.reg = EVSYS_USER_USER(EVSYS_ID_USER_ADC_START)
                  | EVSYS_USER_CHANNEL(1);

  // Enable ADC
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
}


// ADC interrupt (switch between A, B, C)
void ADC_Handler() {
  //digitalWrite(13,HIGH);
  // check if we are in high-side or low-side current sense
  is_high_side = !is_high_side;
  if(is_high_side){
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    //digitalWrite(13,LOW);
    return;
  }

  // read the result and switch to the next channel
  if (ADC->INTFLAG.bit.RESRDY) {
    uint16_t result = ADC->RESULT.reg;
  
    adc_raw[current_phase] = result;
    if (current_phase == 0) {
      ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[_pinB].ulADCChannelNumber;
      current_phase = 1;
    } else if (current_phase == 1) {
      if (_pinC >= 0) {
        ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[_pinC].ulADCChannelNumber;
        current_phase = 2;
      } else {
        ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[_pinA].ulADCChannelNumber;
        current_phase = 0;
      }
    } else {
      ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[_pinA].ulADCChannelNumber;
      current_phase = 0;
    }
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  }
  //digitalWrite(13,LOW);
}


// ------- API functions -------

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC) {

  if(_isset(_pinA) || _isset(_pinB) || _isset(_pinC)) {
    SIMPLEFOC_DEBUG("SAMD-CUR: ERR: Pins already in use for current sensing!");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;;
  }

  // --- Configure ADC pins ---
  if (_isset(pinA)) pinMode(pinA, INPUT);
  if (_isset(pinB)) pinMode(pinB, INPUT);
  if (_isset(pinC)) pinMode(pinC, INPUT);
  
  // save the pins for later use 
  // only one motor possible for now
  _pinA = pinA;
  _pinB = pinB;
  _pinC = pinC;

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_SAMD21_ADC_VOLTAGE) / (_SAMD21_ADC_RESOLUTION),

  };
  return params;
}


float _readADCVoltageLowSide(const int pin, const void* cs_params) {

  // extract the ADC raw value for the given pin
  float countsToVolts = ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;

  // read the value form the buffer
  int i = 0;
  for(auto p: ((GenericCurrentSenseParams*)cs_params)->pins) {
    if (p == pin) return adc_raw[i] * countsToVolts;
    i++;
  }

  return 0.0; // pin not available
}

void* _driverSyncLowSide(void* driver_params, void* cs_params) {
  
  setupADCEventTriggerFromDriver((const SAMDHardwareDriverParams*)driver_params, (const GenericCurrentSenseParams*)cs_params);
  return cs_params;
}

#endif
