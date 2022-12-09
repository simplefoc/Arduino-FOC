#include "stm32g4_hal.h"

#if defined(STM32G4xx) && !defined(ARDUINO_B_G431B_ESC1)

#include "../../../../communication/SimpleFOCDebug.h"

#define SIMPLEFOC_STM32_DEBUG

ADC_HandleTypeDef hadc;

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params)
{
  ADC_InjectionConfTypeDef sConfigInjected;
 
  // check if all pins belong to the same ADC
  ADC_TypeDef* adc_pin1 = (ADC_TypeDef*)pinmap_peripheral(analogInputToPinName(cs_params->pins[0]), PinMap_ADC);
  ADC_TypeDef* adc_pin2 = (ADC_TypeDef*)pinmap_peripheral(analogInputToPinName(cs_params->pins[1]), PinMap_ADC);
  ADC_TypeDef* adc_pin3 = _isset(cs_params->pins[2]) ? (ADC_TypeDef*)pinmap_peripheral(analogInputToPinName(cs_params->pins[2]), PinMap_ADC) : nullptr;
 if ( (adc_pin1 != adc_pin2) || ( (adc_pin3) && (adc_pin1 != adc_pin3) )){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: Analog pins dont belong to the same ADC!");
#endif
  return -1;
 }


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
#ifdef ADC4
  else if (hadc.Instance == ADC4) {
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
  else if (hadc.Instance == ADC5) {
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

#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: Using ADC: ", _adcToIndex(&hadc)+1);
#endif

  hadc.Init.ClockPrescaler =  ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.GainCompensation = 0;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 2;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if ( HAL_ADC_Init(&hadc) != HAL_OK){
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


  // first channel
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[0]));
  if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[0])) );
#endif
    return -1;
  }

  // second channel
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[1]));
  if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[1]))) ;
#endif
    return -1;
  }

  // third channel - if exists
  if(_isset(cs_params->pins[2])){
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[2]));
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK){
#ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-CS: ERR: cannot init injected channel: ", (int)_getADCChannel(analogInputToPinName(cs_params->pins[2]))) ;
#endif
      return -1;
    }
  }
  

 
  if(hadc.Instance == ADC1) {
    // enable interrupt
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
#ifdef ADC2
  else if (hadc.Instance == ADC2) {
    // enable interrupt
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
#endif
#ifdef ADC3
  else if (hadc.Instance == ADC3) {
    // enable interrupt
    HAL_NVIC_SetPriority(ADC3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
  } 
#endif
#ifdef ADC4
  else if (hadc.Instance == ADC4) {
    // enable interrupt
    HAL_NVIC_SetPriority(ADC4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC4_IRQn);
  } 
#endif
#ifdef ADC5
  else if (hadc.Instance == ADC5) {
    // enable interrupt
    HAL_NVIC_SetPriority(ADC5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC5_IRQn);
  } 
#endif
  
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
  void ADC1_2_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
#ifdef ADC3
  void ADC3_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
#endif

#ifdef ADC4
  void ADC4_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
#endif

#ifdef ADC5
  void ADC5_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
#endif
}

#endif