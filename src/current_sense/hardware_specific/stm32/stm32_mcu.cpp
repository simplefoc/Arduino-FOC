
#include "../../hardware_api.h"

#if defined(_STM32_DEF_) and !defined(ARDUINO_B_G431B_ESC1) 

#include "stm32_mcu.h"

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 1024.0f

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  Stm32CurrentSenseParams* params = new Stm32CurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
}

#endif