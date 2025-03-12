#pragma once

#include "Arduino.h"

#if defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#include "../stm32_mcu.h"
#include "stm32f7_utils.h"

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params);
int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC);

#endif
