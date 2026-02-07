#ifndef STM32_ADC_INCLUDE_HAL

#include "Arduino.h"

#if defined(_STM32_DEF_) 

#include "stm32_mcu.h"

// for searching the best ADCs, we need to know the number of ADCs 
// it might be better to use some HAL variable for example ADC_COUNT
// here I've assumed the maximum number of ADCs is 5
#define ADC_COUNT 5

// pointer to the ADC handles used in the project
ADC_HandleTypeDef* _get_adc_handles();

int _adc_init_regular(ADC_TypeDef* adc_instance);
int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params);
int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC);

#endif

#endif