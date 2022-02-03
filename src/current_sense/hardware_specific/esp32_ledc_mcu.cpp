#include "../hardware_api.h"
#include "../../drivers/hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && !defined(SOC_MCPWM_SUPPORTED) 

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )


/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  // uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}

// function reading an ADC value and returning the read voltage
void _configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}
#endif
