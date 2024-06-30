#include "esp32_mcu.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)

/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = adcRead(pinA);
  return raw_adc * ((ESP32CurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA, const int pinB, const int pinC){

  ESP32CurrentSenseParams* params = new ESP32CurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  // initialize the ADC pins
  // fail if the pin is not an ADC pin
  for (int i = 0; i < 3; i++){
    if(_isset(params->pins[i])){
      pinMode(params->pins[i], ANALOG);
      if(!adcInit(params->pins[i])) {
       SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Failed to initialise ADC pin: "+String(params->pins[i]) + String(", maybe not an ADC pin?"));
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
      }
    }
  }

  return params;
}


#endif
