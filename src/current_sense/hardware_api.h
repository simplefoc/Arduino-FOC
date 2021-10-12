#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
float _readADCVoltageInline(const int pinA);

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void _configureADCInline(const int pinA,const int pinB,const int pinC = NOT_SET);

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void _configureADCLowSide(const int pinA,const int pinB,const int pinC = NOT_SET);

void _startADC3PinConversionLowSide();

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
float _readADCVoltageLowSide(const int pinA);

/**
 *  function syncing the Driver with the ADC  for the LowSide Sensing
 */
void _driverSyncLowSide();

#endif
