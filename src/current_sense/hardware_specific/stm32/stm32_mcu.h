
#ifndef STM32_CURRENTSENSE_MCU_DEF
#define STM32_CURRENTSENSE_MCU_DEF
#include "../../hardware_api.h"
#include "../../../common/foc_utils.h"

#if defined(_STM32_DEF_) and !defined(ARDUINO_B_G431B_ESC1) 

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 1024.0f

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct Stm32CurrentSenseParams {
  int pins[3] = {(int)NOT_SET};
  float adc_voltage_conv;
  ADC_HandleTypeDef* adc_handle = NULL;
  HardwareTimer* timer_handle = NULL;
} Stm32CurrentSenseParams;

#endif
#endif