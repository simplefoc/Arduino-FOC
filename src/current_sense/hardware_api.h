#ifndef HARDWARE_UTILS_H
#define HARDWARE_UTILS_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/**
 *  function reading an ADC value and returning the read voltage
 * 
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
float _readADCVoltage(const int pinA);

/**
 *  function reading an ADC value and returning the read voltage
 * 
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void _configureADC(const int pinA,const int pinB,const int pinC = NOT_SET);

#endif