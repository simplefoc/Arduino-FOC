#include "../stm32_adc_utils.h"

#if defined(STM32L4xx)


// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h#L210
uint32_t _timerToInjectedTRGO(TIM_HandleTypeDef* timer){
  if(timer->Instance == TIM1)  
    return ADC_EXTERNALTRIGINJEC_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJEC_T2_TRGO;
#endif
#ifdef TIM3 // if defined timer 3
  else if(timer->Instance == TIM3) 
    return ADC_EXTERNALTRIGINJEC_T3_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->Instance == TIM4) 
    return ADC_EXTERNALTRIGINJEC_T4_TRGO;
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
// https://github.com/stm32duino/Arduino_Core_STM32/blob/6588dee03382e73ed42c4a5e473900ab3b79d6e4/system/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_adc.h#L519
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
  else
    return _TRGO_NOT_AVAILABLE;
}

#endif