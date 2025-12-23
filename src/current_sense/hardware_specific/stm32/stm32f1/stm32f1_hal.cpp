#include "stm32f1_hal.h"

#if defined(STM32F1xx)  

#include "../../../../communication/SimpleFOCDebug.h"
#include "../stm32_adc_utils.h"

// pointer to the ADC handles used in the project
ADC_HandleTypeDef hadc[ADC_COUNT] = {0};

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
  auto adc_instance = _findBestADCForPins(3, cs_params->pins, hadc);

  if(adc_instance == ADC1) __HAL_RCC_ADC1_CLK_ENABLE();
#ifdef ADC2  // if defined ADC2
  else if(adc_instance == ADC2) __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef ADC3  // if defined ADC3
  else if(adc_instance == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();
#endif
  else{
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Pin does not belong to any ADC!");
#endif
    return -1; // error not a valid ADC instance
  }

  auto adc_num = _adcToIndex(adc_instance);
  hadc[adc_num].Instance = adc_instance;
  hadc[adc_num].Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc[adc_num].Init.ContinuousConvMode = ENABLE;
  hadc[adc_num].Init.DiscontinuousConvMode = DISABLE;
  hadc[adc_num].Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc[adc_num].Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc[adc_num].Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc[adc_num]);
  /**Configure for the selected ADC regular channel to be converted. 
  */
  
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = 0;
  for(int pin_no=0; pin_no<3; pin_no++){
    if(_isset(cs_params->pins[pin_no])){
      sConfigInjected.InjectedNbrOfConversion++;
    }
  }
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


  uint8_t channel_no = 0;
  for(int i=0; i<3; i++){
    // skip if not set
    if (!_isset(cs_params->pins[i])) continue;
    
    sConfigInjected.InjectedRank = _getADCInjectedRank(channel_no++);
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[i]), hadc[adc_num].Instance);
    if (HAL_ADCEx_InjectedConfigChannel(&hadc[adc_num], &sConfigInjected) != HAL_OK){
  #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[i]) , hadc[adc_num].Instance));
  #endif
      return -1;
    }
  }
  cs_params->adc_handle = &hadc[adc_num];
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
#if defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG)
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
    if (hadc[0].Instance != NP)
      HAL_ADC_IRQHandler(&hadc[0]);
    if (hadc[1].Instance != NP)
      HAL_ADC_IRQHandler(&hadc[1]);
  }
#ifdef ADC3
  void ADC3_IRQHandler(void)
  {
      if (hadc[2].Instance != NP)
        HAL_ADC_IRQHandler(&hadc[2]);
  }
#endif
}

#endif