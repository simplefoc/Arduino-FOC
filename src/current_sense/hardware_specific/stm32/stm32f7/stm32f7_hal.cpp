#include "stm32f7_hal.h"

#if defined(STM32F7xx)

//#define SIMPLEFOC_STM32_DEBUG

#include "../../../../communication/SimpleFOCDebug.h"

// pointer to the ADC handles used in the project
ADC_HandleTypeDef hadc[ADC_COUNT] = {0};

ADC_HandleTypeDef* _get_adc_handles(){
  return hadc;
}



/**
 * Function initializing the ADC for the regular channels for the low-side current sensing
 * 
 * @param adc_instance - ADC instance to initialize
 * 
 * @return int - 0 if success 
 */
int _adc_init_regular(ADC_TypeDef* adc_instance)
{

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

  int adc_num = _adcToIndex(adc_instance);
  
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: Using ADC: ", adc_num+1);
#endif

  hadc[adc_num].Instance = adc_instance;
  hadc[adc_num].Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc[adc_num].Init.Resolution = ADC_RESOLUTION_12B;
  hadc[adc_num].Init.ScanConvMode = ENABLE;
  hadc[adc_num].Init.ContinuousConvMode = DISABLE;
  hadc[adc_num].Init.DiscontinuousConvMode = DISABLE;
  hadc[adc_num].Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc[adc_num].Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
  hadc[adc_num].Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc[adc_num].Init.NbrOfConversion = 1;
  hadc[adc_num].Init.DMAContinuousRequests = DISABLE;
  hadc[adc_num].Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if ( HAL_ADC_Init(&hadc[adc_num]) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init ADC!");
#endif
    return -1;
  }
  return 0;
}

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

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  auto adc_instance = _findBestADCForInjectedPins(3, cs_params->pins, hadc);
  if(adc_instance == NP){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Pin does not belong to any ADC!");
#endif
    return -1; // error not a valid ADC instance
  }
  if( _adc_init_regular(adc_instance) != 0){
    return -1;
  }
  int adc_num = _adcToIndex(adc_instance);
  
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: Using ADC: ", adc_num+1);
#endif


  ADC_InjectionConfTypeDef sConfigInjected;
    
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = 0;
  for(int pin_no=0; pin_no<3; pin_no++){
    if(_isset(cs_params->pins[pin_no])){
      sConfigInjected.InjectedNbrOfConversion++;
    }
  }
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;  
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;

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
    if (!IS_TIM_REPETITION_COUNTER_INSTANCE(instance_to_check)) {
      // workaround for errata 2.2.1 in ES0290 Rev 7
      // https://www.st.com/resource/en/errata_sheet/es0290-stm32f74xxx-and-stm32f75xxx-device-limitations-stmicroelectronics.pdf
      __HAL_RCC_DAC_CLK_ENABLE();
    } 
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
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ADC channel: ", (int)STM_PIN_CHANNEL(pinmap_function(PinMap_ADC[i].pin, PinMap_ADC)));
#endif
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
  void ADC_IRQHandler(void)
  {
    for(int adc_num=0; adc_num<ADC_COUNT; adc_num++){
      if(hadc[adc_num].Instance == NP) continue; 
      HAL_ADC_IRQHandler(&hadc[adc_num]);
    }
  }
}

#endif