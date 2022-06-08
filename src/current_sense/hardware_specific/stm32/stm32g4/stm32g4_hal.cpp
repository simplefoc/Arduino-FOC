#include "stm32g4_hal.h"

#if defined(STM32G4xx) && !defined(ARDUINO_B_G431B_ESC1)

#include "../../../../communication/SimpleFOCDebug.h"
#define _TRGO_NOT_AVAILABLE 12345


// timer to injected TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/6588dee03382e73ed42c4a5e473900ab3b79d6e4/system/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_adc_ex.h#L217
uint32_t _timerToInjectedTRGO(HardwareTimer* timer){
  if(timer->getHandle()->Instance == TIM1)  
    return ADC_EXTERNALTRIGINJEC_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->getHandle()->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJEC_T2_TRGO;
#endif
#ifdef TIM3 // if defined timer 3
  else if(timer->getHandle()->Instance == TIM2) 
    return ADC_EXTERNALTRIGINJEC_T3_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->getHandle()->Instance == TIM4) 
    return ADC_EXTERNALTRIGINJEC_T4_TRGO;
#endif
#ifdef TIM6 // if defined timer 6
  else if(timer->getHandle()->Instance == TIM6) 
    return ADC_EXTERNALTRIGINJEC_T6_TRGO;
#endif
#ifdef TIM7 // if defined timer 7
  else if(timer->getHandle()->Instance == TIM7) 
    return ADC_EXTERNALTRIGINJEC_T7_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->getHandle()->Instance == TIM8) 
    return ADC_EXTERNALTRIGINJEC_T8_TRGO;
#endif
#ifdef TIM15 // if defined timer 15
  else if(timer->getHandle()->Instance == TIM15) 
    return ADC_EXTERNALTRIGINJEC_T15_TRGO;
#endif
#ifdef TIM20 // if defined timer 15
  else if(timer->getHandle()->Instance == TIM20) 
    return ADC_EXTERNALTRIGINJEC_T20_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

// timer to regular TRGO
// https://github.com/stm32duino/Arduino_Core_STM32/blob/6588dee03382e73ed42c4a5e473900ab3b79d6e4/system/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_adc.h#L519
uint32_t _timerToRegularTRGO(HardwareTimer* timer){
  if(timer->getHandle()->Instance == TIM1)  
    return ADC_EXTERNALTRIG_T1_TRGO;
#ifdef TIM2 // if defined timer 2
  else if(timer->getHandle()->Instance == TIM2) 
    return ADC_EXTERNALTRIG_T2_TRGO;
#endif
#ifdef TIM3 // if defined timer 3
  else if(timer->getHandle()->Instance == TIM2) 
    return ADC_EXTERNALTRIG_T3_TRGO;
#endif
#ifdef TIM4 // if defined timer 4
  else if(timer->getHandle()->Instance == TIM4) 
    return ADC_EXTERNALTRIG_T4_TRGO;
#endif
#ifdef TIM6 // if defined timer 6
  else if(timer->getHandle()->Instance == TIM6) 
    return ADC_EXTERNALTRIG_T6_TRGO;
#endif
#ifdef TIM7 // if defined timer 7
  else if(timer->getHandle()->Instance == TIM7) 
    return ADC_EXTERNALTRIG_T7_TRGO;
#endif
#ifdef TIM8 // if defined timer 8
  else if(timer->getHandle()->Instance == TIM8) 
    return ADC_EXTERNALTRIG_T7_TRGO;
#endif
#ifdef TIM15 // if defined timer 15
  else if(timer->getHandle()->Instance == TIM15) 
    return ADC_EXTERNALTRIG_T15_TRGO;
#endif
#ifdef TIM20 // if defined timer 15
  else if(timer->getHandle()->Instance == TIM20) 
    return ADC_EXTERNALTRIG_T20_TRGO;
#endif
  else
    return _TRGO_NOT_AVAILABLE;
}

ADC_HandleTypeDef hadc;

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params)
{
  ADC_InjectionConfTypeDef sConfigInjected;
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(cs_params->pins[0]), PinMap_ADC);

  if(hadc.Instance == ADC1) {
#ifdef __HAL_RCC_ADC1_CLK_ENABLE
    __HAL_RCC_ADC1_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#ifdef ADC2
  else if (hadc.Instance == ADC2) {
#ifdef __HAL_RCC_ADC2_CLK_ENABLE
    __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC3
  else if (hadc.Instance == ADC3) {
#ifdef __HAL_RCC_ADC3_CLK_ENABLE
    __HAL_RCC_ADC3_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC34_CLK_ENABLE
    __HAL_RCC_ADC34_CLK_ENABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  } 
#endif
  else{
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Pin does not belong to any ADC!");
#endif
    return -1; // error not a valid ADC instance
  }
  
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ENABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
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
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;  
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;

  // automating TRGO flag finding - hardware specific
  uint8_t tim_num = 0;
  while(driver_params->timers[tim_num] != NP && tim_num < 6){
    uint32_t trigger_flag = _timerToInjectedTRGO(driver_params->timers[tim_num++]);
    if(trigger_flag == _TRGO_NOT_AVAILABLE) continue; // timer does not have valid trgo for injected channels

    // if the code comes here, it has found the timer available
    // timer does have trgo flag for injected channels  
    sConfigInjected.ExternalTrigInjecConv = trigger_flag;
    
    // this will be the timer with which the ADC will sync
    cs_params->timer_handle = driver_params->timers[tim_num-1];
    // done
    break;
  }
  if( cs_params->timer_handle == NP ){
    // not possible to use these timers for low-side current sense
  #ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot sync any timer to injected channels!");
  #endif
    return -1;
  }
  // display which timer is being used
  #ifdef SIMPLEFOC_STM32_DEBUG
    // it would be better to use the getTimerNumber from driver
    SIMPLEFOC_DEBUG("STM32-CS: injected trigger for timer index: ", get_timer_index(cs_params->timer_handle->getHandle()->Instance) + 1);
  #endif

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
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  
  cs_params->adc_handle = &hadc;
  return 0;
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