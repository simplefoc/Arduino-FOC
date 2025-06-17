#ifndef STM32G4_LOWSIDE_HAL
#define STM32G4_LOWSIDE_HAL

#include "Arduino.h"

#if defined(STM32G4xx) && !defined(ARDUINO_B_G431B_ESC1)

#include "stm32g4xx_hal.h"
#include "../stm32_mcu.h"
#include "../stm32_adc_utils.h"

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params);
int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC);

#endif

#endif