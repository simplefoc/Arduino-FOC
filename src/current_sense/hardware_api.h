#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// flag returned if current sense init fails
#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void*)-1)

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct GenericCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
} GenericCurrentSenseParams;


/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 * @param cs_params -current sense parameter structure - hardware specific
 */
float _readADCVoltageInline(const int pinA, const void* cs_params);

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param driver_params - driver parameter structure - hardware specific
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void* _configureADCInline(const void *driver_params, const int pinA,const int pinB,const int pinC = NOT_SET);

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param driver_params - driver parameter structure - hardware specific
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void* _configureADCLowSide(const void *driver_params, const int pinA,const int pinB,const int pinC = NOT_SET);

void _startADC3PinConversionLowSide();

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 * @param cs_params -current sense parameter structure - hardware specific
 */
float _readADCVoltageLowSide(const int pinA, const void* cs_params);

/**
 *  function syncing the Driver with the ADC  for the LowSide Sensing
 * @param driver_params - driver parameter structure - hardware specific
 * @param cs_params - current sense parameter structure - hardware specific
 */
void _driverSyncLowSide(void* driver_params, void* cs_params);

#endif
