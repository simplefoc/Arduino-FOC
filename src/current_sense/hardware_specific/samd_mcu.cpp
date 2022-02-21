#include "../hardware_api.h" 

#if defined(_SAMD21_)|| defined(_SAMD51_) || defined(_SAME51_)

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 1024.0f

// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  digitalWrite(7,HIGH);
  uint32_t raw_adc = analogRead(pinA);
  digitalWrite(7,LOW);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);
  pinMode(7,OUTPUT);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}
#endif