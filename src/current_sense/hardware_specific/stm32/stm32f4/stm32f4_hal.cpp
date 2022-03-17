#include "stm32f4_hal.h"

#if defined(STM32F4xx)


// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h#L179
uint32_t _timerToInjectedTRGO(HardwareTimer* timer){
  if(timer->getHandle()->Instance == TIM1)  
    return ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  else if(timer->getHandle()->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJECCONV_T2_TRGO;
  else if(timer->getHandle()->Instance == TIM4) 
    return ADC_EXTERNALTRIGINJECCONV_T4_TRGO;
  else if(timer->getHandle()->Instance == TIM5) 
    return ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
  else
    return 0;
}

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h#L331
uint32_t _timerToRegularTRGO(HardwareTimer* timer){
  if(timer->getHandle()->Instance == TIM2)  
    return ADC_EXTERNALTRIGCONV_T2_TRGO;
  else if(timer->getHandle()->Instance == TIM3) 
    return ADC_EXTERNALTRIGCONV_T3_TRGO;
  else if(timer->getHandle()->Instance == TIM8) 
    return ADC_EXTERNALTRIGCONV_T8_TRGO;
  else
    return 0;
}

ADC_HandleTypeDef hadc;

void _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;
    
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(cs_params->pins[0]), PinMap_ADC);

  if(hadc.Instance == ADC1) __HAL_RCC_ADC1_CLK_ENABLE();
  else if(hadc.Instance == ADC2) __HAL_RCC_ADC2_CLK_ENABLE();
  else if(hadc.Instance == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();

  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ENABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc);
  /**Configure for the selected ADC regular channel to be converted. 
  */
  
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = _isset(cs_params->pins[2]) ? 3 : 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;

  // automating TRGO flag finding - hardware specific
  // sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  uint8_t tim_num = 0;
  while(driver_params->timers[tim_num] && tim_num < 6){
    uint32_t trigger_flag = _timerToInjectedTRGO(driver_params->timers[tim_num++]);
    if(!trigger_flag) continue; // timer does not have valid trgo for injected channels

    // timer does have trgo flag for injuected channels  
    sConfigInjected.ExternalTrigInjecConv = trigger_flag;
    
    // this will be the timer with which the ADC will sync
    cs_params->timer_handle = driver_params->timers[tim_num-1];
  }
  if(!cs_params->timer_handle){
    // not possible to use these timers for low-side current sense
    return;
  };
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;

  // first channel
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedChannel = STM_PIN_CHANNEL(pinmap_function(analogInputToPinName(cs_params->pins[0]), PinMap_ADC));
  HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected);
  // second channel
  sConfigInjected.InjectedRank = 2;
  sConfigInjected.InjectedChannel = STM_PIN_CHANNEL(pinmap_function(analogInputToPinName(cs_params->pins[1]), PinMap_ADC));
  HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected);

  // third channel - if exists
  if(_isset(cs_params->pins[2])){
    sConfigInjected.InjectedRank = 3;
    sConfigInjected.InjectedChannel = STM_PIN_CHANNEL(pinmap_function(analogInputToPinName(cs_params->pins[2]), PinMap_ADC));
    HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected);
  }
  
  // enable interrupt
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  
  cs_params->adc_handle = &hadc;
}

void _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC)
{
  uint8_t cnt = 0;
  if(_isset(pinA)){
    pinmap_pinout(analogInputToPinName(pinA), PinMap_ADC);
    cs_params->pins[cnt++] = pinA;
  }
  if(_isset(pinB)){
    pinmap_pinout(analogInputToPinName(pinB), PinMap_ADC);
    cs_params->pins[cnt++] = pinB;
  }
  if(_isset(pinC)){ 
    pinmap_pinout(analogInputToPinName(pinC), PinMap_ADC);
    cs_params->pins[cnt] = pinC;
  }
}

extern "C" {
  void ADC_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
}

#endif