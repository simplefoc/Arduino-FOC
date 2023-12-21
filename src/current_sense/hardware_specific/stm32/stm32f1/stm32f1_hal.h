#ifndef STM32F1_LOWSIDE_HAL
#define STM32F1_LOWSIDE_HAL

#include "Arduino.h"

#if defined(STM32F1xx) 
#include "stm32f1xx_hal.h"
#include "../../../../common/foc_utils.h"
#include "../../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "../stm32_mcu.h"

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params);
void _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC);

#endif

#endif