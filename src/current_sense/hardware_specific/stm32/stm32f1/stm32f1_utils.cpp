#include "../stm32_adc_utils.h"

#if defined(STM32F1xx)


// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_adc_ex.h#L215
uint32_t _timerToInjectedTRGO(TIM_HandleTypeDef* timer){
  if(timer->Instance == TIM1)  
    return ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJECCONV_T2_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->Instance == TIM4) 
    return ADC_EXTERNALTRIGINJECCONV_T4_TRGO;
#endif
#ifdef TIM5 // if defined timer 5
  else if(timer->Instance == TIM5) 
    return ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_adc_ex.h#L215
uint32_t _timerToRegularTRGO(TIM_HandleTypeDef* timer){
  if(timer->Instance == TIM3) 
    return ADC_EXTERNALTRIGCONV_T3_TRGO;
#ifdef TIM8 // if defined timer 8
  else if(timer->Instance == TIM8) 
    return ADC_EXTERNALTRIGCONV_T8_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

#endif