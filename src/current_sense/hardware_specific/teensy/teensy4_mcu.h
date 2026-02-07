
#ifndef TEENSY4_CURRENTSENSE_MCU_DEF
#define TEENSY4_CURRENTSENSE_MCU_DEF

#include "../../hardware_api.h"
#include "../../../common/foc_utils.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4026.0f

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct Teensy4CurrentSenseParams {
  int pins[3] = {(int)NOT_SET};
  float adc_voltage_conv;
} Teensy4CurrentSenseParams;



const uint8_t pin_to_channel[] = { // pg 482
  7,  // 0/A0  AD_B1_02
  8,  // 1/A1  AD_B1_03
  12, // 2/A2  AD_B1_07
  11, // 3/A3  AD_B1_06
  6,  // 4/A4  AD_B1_01
  5,  // 5/A5  AD_B1_00
  15, // 6/A6  AD_B1_10
  0,  // 7/A7  AD_B1_11
  13, // 8/A8  AD_B1_08
  14, // 9/A9  AD_B1_09
  1,  // 24/A10 AD_B0_12 
  2,  // 25/A11 AD_B0_13
  128+3,  // 26/A12 AD_B1_14 - only on ADC2, 3
  128+4,  // 27/A13 AD_B1_15 - only on ADC2, 4
  7,  // 14/A0  AD_B1_02
  8,  // 15/A1  AD_B1_03
  12, // 16/A2  AD_B1_07
  11, // 17/A3  AD_B1_06
  6,  // 18/A4  AD_B1_01
  5,  // 19/A5  AD_B1_00
  15, // 20/A6  AD_B1_10
  0,  // 21/A7  AD_B1_11
  13, // 22/A8  AD_B1_08
  14, // 23/A9  AD_B1_09
  1,  // 24/A10 AD_B0_12
  2,  // 25/A11 AD_B0_13
  128+3,  // 26/A12 AD_B1_14 - only on ADC2, 3
  128+4,  // 27/A13 AD_B1_15 - only on ADC2, 4
#ifdef ARDUINO_TEENSY41
  255,  // 28
  255,  // 29
  255,  // 30
  255,  // 31
  255,  // 32
  255,  // 33
  255,  // 34
  255,  // 35
  255,  // 36
  255,  // 37
  128+1,  // 38/A14 AD_B1_12 - only on ADC2, 1
  128+2,  // 39/A15 AD_B1_13 - only on ADC2, 2
  9,  // 40/A16 AD_B1_04
  10, // 41/A17 AD_B1_05
#endif
};


#endif

#endif