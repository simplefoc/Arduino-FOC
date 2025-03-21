#ifndef STM32F4_LOWSIDE_HAL
#define STM32F4_LOWSIDE_HAL

#include "Arduino.h"

#if defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#include "../stm32_mcu.h"
#include "../stm32_adc_utils.h"

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params);
int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC);

#endif

#endif