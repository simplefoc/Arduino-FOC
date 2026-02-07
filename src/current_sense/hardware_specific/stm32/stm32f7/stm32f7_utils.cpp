#include "../stm32_adc_utils.h"

#if defined(STM32F7xx)

/*
TIM1
TIM2  
TIM3  
TIM4  
TIM5  
TIM6  
TIM7  
TIM12 
TIM13 
TIM14 

ADC_EXTERNALTRIGINJECCONV_T1_TRGO     
ADC_EXTERNALTRIGINJECCONV_T2_TRGO     
ADC_EXTERNALTRIGINJECCONV_T4_TRGO     

ADC_EXTERNALTRIGINJECCONV_T1_TRGO2    
ADC_EXTERNALTRIGINJECCONV_T8_TRGO2    
ADC_EXTERNALTRIGINJECCONV_T5_TRGO     
ADC_EXTERNALTRIGINJECCONV_T6_TRGO     
*/
// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h#L179
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
#ifdef TIM6 // if defined timer 6
  else if(timer->Instance == TIM6) 
    return ADC_EXTERNALTRIGINJECCONV_T6_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->Instance == TIM8) 
    return ADC_EXTERNALTRIGINJECCONV_T8_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}
/*
  
ADC_EXTERNALTRIGCONV_T5_TRGO   
ADC_EXTERNALTRIGCONV_T8_TRGO  
ADC_EXTERNALTRIGCONV_T8_TRGO2 
ADC_EXTERNALTRIGCONV_T1_TRGO  
ADC_EXTERNALTRIGCONV_T1_TRGO2 
ADC_EXTERNALTRIGCONV_T2_TRGO  
ADC_EXTERNALTRIGCONV_T4_TRGO  
ADC_EXTERNALTRIGCONV_T6_TRGO  
*/

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h#L331
uint32_t _timerToRegularTRGO(TIM_HandleTypeDef* timer){
  if(timer->Instance == TIM1)  
    return ADC_EXTERNALTRIGCONV_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->Instance == TIM2) 
    return ADC_EXTERNALTRIGCONV_T2_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->Instance == TIM4) 
    return ADC_EXTERNALTRIGCONV_T4_TRGO;
#endif
#ifdef TIM5 // if defined timer 5
  else if(timer->Instance == TIM5) 
    return ADC_EXTERNALTRIGCONV_T5_TRGO;
#endif
#ifdef TIM6 // if defined timer 6
  else if(timer->Instance == TIM6) 
    return ADC_EXTERNALTRIGCONV_T6_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->Instance == TIM8) 
    return ADC_EXTERNALTRIGCONV_T8_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

#endif