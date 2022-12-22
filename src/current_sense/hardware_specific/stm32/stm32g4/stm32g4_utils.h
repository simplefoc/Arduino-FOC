
#ifndef STM32G4_UTILS_HAL
#define STM32G4_UTILS_HAL

#include "Arduino.h"

#if defined(STM32G4xx) && !defined(ARDUINO_B_G431B_ESC1)

#define _TRGO_NOT_AVAILABLE 12345


/* Exported Functions */
/**
  * @brief  Return ADC HAL channel linked to a PinName
  * @param  pin: PinName
  * @retval Valid HAL channel
  */
uint32_t _getADCChannel(PinName pin);

// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/6588dee03382e73ed42c4a5e473900ab3b79d6e4/system/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_adc_ex.h#L217
uint32_t _timerToInjectedTRGO(HardwareTimer* timer);

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/6588dee03382e73ed42c4a5e473900ab3b79d6e4/system/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_adc.h#L519
uint32_t _timerToRegularTRGO(HardwareTimer* timer);

// function returning index of the ADC instance
int _adcToIndex(ADC_HandleTypeDef *AdcHandle);
int _adcToIndex(ADC_TypeDef *AdcHandle);

#endif

#endif