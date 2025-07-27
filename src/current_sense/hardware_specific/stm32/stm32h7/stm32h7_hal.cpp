#include "stm32h7_hal.h"

#if defined(STM32H7xx)

//#define SIMPLEFOC_STM32_DEBUG

#include "../../../../communication/SimpleFOCDebug.h"

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
  hadc.Instance = _findBestADCForPins(3, cs_params->pins);

  if(hadc.Instance == ADC1) __HAL_RCC_ADC12_CLK_ENABLE();
#ifdef ADC2  // if defined ADC2
  else if(hadc.Instance == ADC2) __HAL_RCC_ADC12_CLK_ENABLE();
#endif
#ifdef ADC3  // if defined ADC3
  else if(hadc.Instance == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();
#endif
  else{
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Cannot find a common ADC for the pins!");
#endif
    return -1; // error not a valid ADC instance
  }
  
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: Using ADC: ", _adcToIndex(&hadc)+1);
#endif

  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
#if defined(ADC_VER_V5_V90)
  // only for ADC3 
  if(hadc.Instance == ADC3){
    hadc.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  }
  // more info here
  // https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_adc.h#L170C13-L170C27
  hadc.Init.DMAContinuousRequests = DISABLE;
  // not sure about this one!!! maybe use: ADC_SAMPLING_MODE_NORMAL
  hadc.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
#endif
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.NbrOfDiscConversion = 0;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  hadc.Init.LowPowerAutoWait = DISABLE;


  if ( HAL_ADC_Init(&hadc) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init ADC!");
#endif
    return -1;
  }
    
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = 0;
  for(int pin_no=0; pin_no<3; pin_no++){
    if(_isset(cs_params->pins[pin_no])){
      sConfigInjected.InjectedNbrOfConversion++;
    }
  }
  // if ADC1 or ADC2
  if(hadc.Instance == ADC1 || hadc.Instance == ADC2){
    // more info here: https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_adc.h#L658
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  }else {
    // adc3
    // https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/system/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_adc.h#L673
    sConfigInjected.InjectedSamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  }
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISINGFALLING;  
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedSingleDiff            = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber          = ADC_OFFSET_NONE;
  sConfigInjected.QueueInjectedContext          = DISABLE;
  sConfigInjected.InjecOversamplingMode         = DISABLE;  

  // automating TRGO flag finding - hardware specific
  uint8_t tim_num = 0;
  for (size_t i=0; i<6; i++) {
    TIM_HandleTypeDef *timer_to_check = driver_params->timers_handle[tim_num++];
    TIM_TypeDef *instance_to_check = timer_to_check->Instance;

    uint32_t trigger_flag = _timerToInjectedTRGO(timer_to_check);
    if(trigger_flag == _TRGO_NOT_AVAILABLE) continue; // timer does not have valid trgo for injected channels

    // check if TRGO used already - if yes use the next timer
    if((timer_to_check->Instance->CR2 & LL_TIM_TRGO_ENABLE) || // if used for timer sync
       (timer_to_check->Instance->CR2 & LL_TIM_TRGO_UPDATE)) // if used for ADC sync
      {
      continue;
    }

    // if the code comes here, it has found the timer available
    // timer does have trgo flag for injected channels  
    sConfigInjected.ExternalTrigInjecConv = trigger_flag;
    
    // this will be the timer with which the ADC will sync
    cs_params->timer_handle = timer_to_check;
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

  uint8_t channel_no = 0;
  for(int i=0; i<3; i++){
    // skip if not set
    if (!_isset(cs_params->pins[i])) continue;
    
    sConfigInjected.InjectedRank = _getADCInjectedRank(channel_no++);
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[i]), hadc.Instance);
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK){
  #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[i]) , hadc.Instance));
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
  void ADC_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
}

#ifdef ADC3
extern "C" {
  void ADC3_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
}
#endif

#endif