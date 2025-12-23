#include "stm32l4_hal.h"

#if defined(STM32L4xx)

#include "../../../../communication/SimpleFOCDebug.h"

#define SIMPLEFOC_STM32_DEBUG

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
  int adc_num = _adcToIndex(adc_instance);
  if (hadc[adc_num].Instance != NP) {
    SIMPLEFOC_DEBUG("STM32-CS: ERR: ADC already in use: ADC", adc_num + 1);
    return -1;
  }
  
  if(adc_instance == ADC1) {
#ifdef __HAL_RCC_ADC1_CLK_ENABLE
    __HAL_RCC_ADC1_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#ifdef ADC2
  else if (adc_instance == ADC2) {
#ifdef __HAL_RCC_ADC2_CLK_ENABLE
    __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC3
  else if (adc_instance == ADC3) {
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
#ifdef ADC4
  else if (adc_instance == ADC4) {
#ifdef __HAL_RCC_ADC4_CLK_ENABLE
    __HAL_RCC_ADC4_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC34_CLK_ENABLE
    __HAL_RCC_ADC34_CLK_ENABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC5
  else if (adc_instance == ADC5) {
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  }
#endif
  else{
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Cannot find a common ADC for the pins!");
#endif
    return -1; // error not a valid ADC instance
  }

#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: Using ADC: ", adc_num+1);
#endif

  hadc[adc_num].Instance = adc_instance;
  hadc[adc_num].Init.ClockPrescaler =  ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc[adc_num].Init.Resolution = ADC_RESOLUTION_12B;
  hadc[adc_num].Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc[adc_num].Init.ContinuousConvMode = DISABLE;
  hadc[adc_num].Init.LowPowerAutoWait = DISABLE;
  hadc[adc_num].Init.DiscontinuousConvMode = DISABLE;
  hadc[adc_num].Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc[adc_num].Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
  hadc[adc_num].Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc[adc_num].Init.NbrOfConversion = 1;
  hadc[adc_num].Init.DMAContinuousRequests = DISABLE;
  hadc[adc_num].Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc[adc_num].Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if ( HAL_ADC_Init(&hadc[adc_num]) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init ADC!");
#endif
    return -1;
  }
    
  /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedNbrOfConversion = _isset(cs_params->pins[2]) ? 3 : 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;  
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;

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
      SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[i]), hadc[adc_num].Instance));
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

#ifdef ADC4
  void ADC4_IRQHandler(void)
  {
    if (hadc[3].Instance != NP)
      HAL_ADC_IRQHandler(&hadc[3]);
  }
#endif

#ifdef ADC5
  void ADC5_IRQHandler(void)
  {
    if (hadc[4].Instance != NP)
      HAL_ADC_IRQHandler(&hadc[4]);
  }
#endif
}

#endif