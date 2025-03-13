#include "stm32f1_hal.h"

#if defined(STM32F1xx)  

#include "../../../../communication/SimpleFOCDebug.h"
#define _TRGO_NOT_AVAILABLE 12345

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

ADC_HandleTypeDef hadc;

/**
 * Function initializing the ADC and the injected channels for the low-side current sensing
 * 
 * @param cs_params - current sense parameters
 * @param driver_params - driver parameters
 * 
 * @return int - 0 if success 
 */
int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params)
{
  ADC_InjectionConfTypeDef sConfigInjected;
    
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(cs_params->pins[0]), PinMap_ADC);
  if(hadc.Instance == ADC1) __HAL_RCC_ADC1_CLK_ENABLE();
#ifdef ADC2  // if defined ADC2
  else if(hadc.Instance == ADC2) __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef ADC3  // if defined ADC3
  else if(hadc.Instance == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();
#endif
  else{
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Pin does not belong to any ADC!");
#endif
    return -1; // error not a valid ADC instance
  }

  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc);
  /**Configure for the selected ADC regular channel to be converted. 
  */
  
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = _isset(cs_params->pins[2]) ? 3 : 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;

  // automating TRGO flag finding - hardware specific
  uint8_t tim_num = 0;
  while(driver_params->timers_handle[tim_num] != NP && tim_num < 6){
    uint32_t trigger_flag = _timerToInjectedTRGO(driver_params->timers_handle[tim_num++]);
    if(trigger_flag == _TRGO_NOT_AVAILABLE) continue; // timer does not have valid trgo for injected channels

    // check if TRGO used already - if yes use the next timer
    if((driver_params->timers_handle[tim_num-1]->Instance->CR2 & LL_TIM_TRGO_ENABLE) || // if used for timer sync
       (driver_params->timers_handle[tim_num-1]->Instance->CR2 & LL_TIM_TRGO_UPDATE)) // if used for ADC sync
      {
      continue;
    }
    
    // if the code comes here, it has found the timer available
    // timer does have trgo flag for injected channels  
    sConfigInjected.ExternalTrigInjecConv = trigger_flag;
    
    // this will be the timer with which the ADC will sync
    cs_params->timer_handle = driver_params->timers_handle[tim_num-1];
    // done
    break;
  }
  if( cs_params->timer_handle == NP ){
    // not possible to use these timers for low-side current sense
  #ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot sync any timer to injected channels!");
  #endif
    return -1;
  }else{
  #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-CS: Using timer: ", stm32_getTimerNumber(cs_params->timer_handle->Instance));
  #endif
  }


  uint8_t rank_no = 0;
  for(int i=0; i<3; i++){
    // skip if not set
    if (!_isset(cs_params->pins[i])) continue;
    
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1 + rank_no++;
    sConfigInjected.InjectedChannel = STM_PIN_CHANNEL(pinmap_function(analogInputToPinName(cs_params->pins[i]), PinMap_ADC));
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK){
  #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)STM_PIN_CHANNEL(pinmap_function(analogInputToPinName(cs_params->pins[i]), PinMap_ADC)));
  #endif
      return -1;
    }
  }
  cs_params->adc_handle = &hadc;
  return 0;
}

/**
 * Function to initialize the ADC GPIO pins
 * 
 * @param cs_params current sense parameters
 * @param pinA pin number for phase A
 * @param pinB pin number for phase B
 * @param pinC pin number for phase C
 * @return int 0 if success, -1 if error
 */
int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC)
{
  int pins[3] = {pinA, pinB, pinC};
  const char* port_names[3] = {"A", "B", "C"};
  for(int i=0; i<3; i++){
    if(_isset(pins[i])){
      // check if pin is an analog pin
      if(pinmap_peripheral(analogInputToPinName(pins[i]), PinMap_ADC) == NP){
#ifdef SIMPLEFOC_STM32_DEBUG
        SimpleFOCDebug::print("STM32-CS: ERR: Pin ");
        SimpleFOCDebug::print(port_names[i]);
        SimpleFOCDebug::println(" does not belong to any ADC!");
#endif
        return -1;
      }
      pinmap_pinout(analogInputToPinName(pins[i]), PinMap_ADC);
      cs_params->pins[i] = pins[i];
    }
  }
  return 0;
}


extern "C" {
  void ADC1_2_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
}

#endif