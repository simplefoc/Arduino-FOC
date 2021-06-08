
#include "../hardware_api.h"

#if defined(_STM32_DEF_)

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 1024.0f

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )
int _pinA, _pinB, _pinC = NOT_SET;
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}
// function reading an ADC value and returning the read voltage
void _configureADCInline(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}


bool _readADCVoltagesLowSide( float & a, float & b, float & c)
{
  a = _readADCVoltageInline(_pinA);
  b = _readADCVoltageInline(_pinB);
  if( _isset(_pinC) )
    c = _readADCVoltageInline(_pinC);
  return true;
}
// Configure low side for generic mcu
// cannot do much but 
void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  _pinA = pinA;
  _pinB = pinB;
  if( _isset(pinC) ) _pinC = pinC;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

}


#endif
