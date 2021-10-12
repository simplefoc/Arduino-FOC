#include "../hardware_api.h"

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * 5.0f/1024.0f;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  void _configureADCInline(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageLowSide(const int pinA){
  return _readADCVoltageInline(pinA);
}

// Configure low side for generic mcu
// cannot do much but 
__attribute__((weak))  void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// sync driver and the adc
__attribute__((weak)) void _driverSyncLowSide(){ }
__attribute__((weak)) void _startADC3PinConversionLowSide(){ }
