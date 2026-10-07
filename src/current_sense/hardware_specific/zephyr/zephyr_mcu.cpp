
#include "../../hardware_api.h"

#if defined(ARDUINO_ARCH_ZEPHYR)

#include <Arduino.h>
#include "../../../communication/SimpleFOCDebug.h"

// Inline current sensing on the Arduino Zephyr core (ArduinoCore-zephyr) uses the standard
// Arduino analogRead()/analogReadResolution() API, which reads the ADC channels declared in
// the `io-channels` property of the board's `zephyr,user` devicetree node.
//
// Low-side current sensing needs the ADC conversions to be triggered in sync with the PWM
// timer. That synchronization is not available through the Arduino wiring layer, so only
// inline current sensing is supported here.

// ADC read resolution used for current sensing. Overridable from the build environment.
#ifndef SIMPLEFOC_ZEPHYR_ADC_RESOLUTION
#define SIMPLEFOC_ZEPHYR_ADC_RESOLUTION 12
#endif

// ADC reference voltage in volts. Overridable from the build environment to match the
// board's analog reference.
#ifndef SIMPLEFOC_ZEPHYR_ADC_VOLTAGE
#define SIMPLEFOC_ZEPHYR_ADC_VOLTAGE 3.3f
#endif


// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function configuring the ADC for inline current sensing
void* _configureADCInline(const void* driver_params, const int pinA, const int pinB, const int pinC){
  _UNUSED(driver_params);

  analogReadResolution(SIMPLEFOC_ZEPHYR_ADC_RESOLUTION);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  const float adc_range = (float)(1UL << SIMPLEFOC_ZEPHYR_ADC_RESOLUTION);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (SIMPLEFOC_ZEPHYR_ADC_VOLTAGE) / adc_range
  };

  return params;
}

// low-side current sensing is not supported through the Arduino Zephyr wiring API
float _readADCVoltageLowSide(const int pinA, const void* cs_params){
  _UNUSED(pinA);
  _UNUSED(cs_params);
  SIMPLEFOC_DEBUG("ERR: Low-side cs not supported on Zephyr core!");
  return 0.0f;
}

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC){
  _UNUSED(driver_params);
  _UNUSED(pinA);
  _UNUSED(pinB);
  _UNUSED(pinC);
  SIMPLEFOC_DEBUG("ERR: Low-side cs not supported on Zephyr core!");
  return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}

void* _driverSyncLowSide(void* driver_params, void* cs_params){
  _UNUSED(driver_params);
  return cs_params;
}

void _startADC3PinConversionLowSide(){ }

#endif
