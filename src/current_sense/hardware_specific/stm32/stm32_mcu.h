
#ifndef STM32_CURRENTSENSE_MCU_DEF
#define STM32_CURRENTSENSE_MCU_DEF
#include "../../hardware_api.h"
#include "../../../common/foc_utils.h"
#include "../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "../../../drivers/hardware_specific/stm32/stm32_timerutils.h"

#if defined(_STM32_DEF_) 

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct Stm32CurrentSenseParams {
  int pins[3] = {(int)NOT_SET};
  float adc_voltage_conv;
  ADC_HandleTypeDef* adc_handle = NP;
  TIM_HandleTypeDef* timer_handle = NP;
} Stm32CurrentSenseParams;


/**
 * Read a regular ADC channel while injected channels are running for current sensing.
 * Injected conversions have hardware priority and will pre-empt regular conversions.
 * 
 * This funciton performs a one-shot regular conversion on either on the same ADC as the one 
 * used for injected current sensing, or on another ADC if the pin belongs to a different ADC.
 * The funciton will initialize the ADC for regular conversion if not already initialized. 
 * 
 * NOTE:
 * The low-side current sensing code already initializes the ADC for injected conversions and regullar conversions
 * so if the same ADC is used for regular reading, no re-configuration is needed.
 * 
 * NOTE:
 * This function will be relatively slow >10us in comparision to the injected reading. 
 * But it is much better than analogRead though.
 * 
 * @param pin - the Arduino pin to be read (must be an ADC pin on the same ADC)
 * @return float - the voltage read from the pin, or -1.0f on error
 */
float _readRegularADCVoltage(const int pin);

#endif
#endif