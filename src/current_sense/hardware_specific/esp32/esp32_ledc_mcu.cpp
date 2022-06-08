#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && !defined(SOC_MCPWM_SUPPORTED) 

#include "esp32_adc_driver.h"

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

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
