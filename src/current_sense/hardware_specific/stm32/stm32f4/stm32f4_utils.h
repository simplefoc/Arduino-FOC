
#ifndef STM32F4_UTILS_HAL
#define STM32F4_UTILS_HAL

#include "Arduino.h"

#if defined(STM32F4xx)

#define _TRGO_NOT_AVAILABLE 12345


/* Exported Functions */
/**
  * @brief  Return ADC HAL channel linked to a PinName
  * @param  pin: PinName
  * @retval Valid HAL channel
  */
uint32_t _getADCChannel(PinName pin);

// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h#L179
uint32_t _timerToInjectedTRGO(HardwareTimer* timer);

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h#L331
uint32_t _timerToRegularTRGO(HardwareTimer* timer);

// function returning index of the ADC instance
int _adcToIndex(ADC_HandleTypeDef *AdcHandle);
int _adcToIndex(ADC_TypeDef *AdcHandle);

#endif

#endif