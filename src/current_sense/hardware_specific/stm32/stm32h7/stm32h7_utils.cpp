#include "../stm32_adc_utils.h"

#if defined(STM32H7xx)

/* Exported Functions */


// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_adc_ex.h#L235
uint32_t _timerToInjectedTRGO(TIM_HandleTypeDef* timer){

  if(timer->Instance == TIM1)  
    return ADC_EXTERNALTRIGINJEC_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJEC_T2_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->Instance == TIM4) 
    return ADC_EXTERNALTRIGINJEC_T4_TRGO;
#endif
#ifdef TIM3 // if defined timer 3
  else if(timer->Instance == TIM3) 
    return ADC_EXTERNALTRIGINJEC_T3_TRGO;
#endif
#ifdef TIM6 // if defined timer 6
  else if(timer->Instance == TIM6) 
    return ADC_EXTERNALTRIGINJEC_T6_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->Instance == TIM8) 
    return ADC_EXTERNALTRIGINJEC_T8_TRGO;
#endif
#ifdef TIM15 // if defined timer 15
  else if(timer->Instance == TIM15) 
    return ADC_EXTERNALTRIGINJEC_T15_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_adc.h#L554
uint32_t _timerToRegularTRGO(TIM_HandleTypeDef* timer){
  if(timer->Instance == TIM1)  
    return ADC_EXTERNALTRIG_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->Instance == TIM2) 
    return ADC_EXTERNALTRIG_T2_TRGO;
#endif
#ifdef TIM3 // if defined timer 3
  else if(timer->Instance == TIM3) 
    return ADC_EXTERNALTRIG_T3_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->Instance == TIM4) 
    return ADC_EXTERNALTRIG_T4_TRGO;
#endif
#ifdef TIM6 // if defined timer 6
  else if(timer->Instance == TIM6) 
    return ADC_EXTERNALTRIG_T6_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->Instance == TIM8) 
    return ADC_EXTERNALTRIG_T8_TRGO;
#endif
#ifdef TIM15 // if defined timer 15
  else if(timer->Instance == TIM15) 
    return ADC_EXTERNALTRIG_T15_TRGO;
#endif
#ifdef TIM23 // if defined timer 23
  else if(timer->Instance == TIM23) 
    return ADC_EXTERNALTRIG_T23_TRGO;
#endif
#ifdef TIM24 // if defined timer 24
  else if(timer->Instance == TIM24) 
    return ADC_EXTERNALTRIG_T24_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

#endif