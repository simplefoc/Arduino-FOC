#include "../hardware_api.h"
#include "../../communication/SimpleFOCDebug.h"

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (5.0f)/(1024.0f)
  };

  return params;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageLowSide(const int pinA, const void* cs_params){
  SIMPLEFOC_DEBUG("ERR: Low-side cs not supported!");
  return 0.0;
}

// Configure low side for generic mcu
// cannot do much but 
__attribute__((weak))  void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  SIMPLEFOC_DEBUG("ERR: Low-side cs not supported!");
  return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}

// sync driver and the adc
__attribute__((weak)) void* _driverSyncLowSide(void* driver_params, void* cs_params){
  _UNUSED(driver_params);
  return cs_params;
}

// function starting the ADC conversion for the high side current sensing
// only necessary for certain types of MCUs 
__attribute__((weak)) void _startADC3PinConversionLowSide(){ }
