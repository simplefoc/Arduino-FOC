#include "stm32_adc_utils.h"
#include "stm32_mcu.h"

#if defined(_STM32_DEF_) 



int _adcToIndex(ADC_TypeDef *AdcHandle){
  if(AdcHandle == ADC1) return 0;
#ifdef ADC2 // if ADC2 exists
  else if(AdcHandle == ADC2) return 1;
#endif
#ifdef ADC3 // if ADC3 exists
  else if(AdcHandle == ADC3) return 2;
#endif
#ifdef ADC4 // if ADC4 exists
  else if(AdcHandle == ADC4) return 3;
#endif
#ifdef ADC5 // if ADC5 exists
  else if(AdcHandle == ADC5) return 4;
#endif
  return 0;
}
int _adcToIndex(ADC_HandleTypeDef *AdcHandle){
  return _adcToIndex(AdcHandle->Instance);
}


ADC_TypeDef* _indexToADC(uint8_t index){
  switch (index) {
    case 0:
      return ADC1;
      break;
#ifdef ADC2 // if ADC2 exists
    case 1:
      return ADC2;
      break;
#endif
#ifdef ADC3 // if ADC3 exists
    case 2:
      return ADC3;
      break;
#endif
#ifdef ADC4 // if ADC4 exists
    case 3:
      return ADC4;
      break;
#endif
#ifdef ADC5 // if ADC5 exists
    case 4:
      return ADC5;
      break;
#endif
  }
  return nullptr;
}

int _findIndexOfEntry(PinName pin) {
  // remove the ALT if it is there
  PinName pinName = (PinName)(pinName & ~ALTX_MASK);
  int i = 0;
  SIMPLEFOC_DEBUG("STM32-CS: Looking for pin ");
  while (PinMap_ADC[i].pin !=NC) {
    if (pinName == PinMap_ADC[i].pin )
      return i;
    i++;
  SIMPLEFOC_DEBUG("STM32-CS: Looking for pin ", i);
  }
  return -1;
}

int _findIndexOfLastEntry(PinName pin) {
  // remove the ALT if it is there
  PinName pinName = (PinName)(pin & ~ALTX_MASK);

  int i = 0;
  while (PinMap_ADC[i].pin!=NC) {
    if (   pinName == (PinMap_ADC[i].pin & ~ALTX_MASK) 
        && pinName != (PinMap_ADC[i+1].pin & ~ALTX_MASK))
      return i;
    i++;
  }
  return -1;
}
int _findIndexOfFirstEntry(PinName pin) {
  // remove the ALT if it is there
  PinName pinName = (PinName)(pin & ~ALTX_MASK);
  int i = 0;
  while (PinMap_ADC[i].pin !=NC) {
    if (pinName == PinMap_ADC[i].pin )
      return i;
    i++;
  }
  return -1;
}

// functions finding the index of the first pin entry in the PinMap_ADC
// returns -1 if not found
int _findIndexOfFirstPinMapADCEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  // remove the ALT if it is there
  return _findIndexOfFirstEntry(pinName);
}

// functions finding the index of the last pin entry in the PinMap_ADC
// returns -1 if not found
int _findIndexOfLastPinMapADCEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  // remove the ALT if it is there
  return _findIndexOfLastEntry(pinName);
}

// find the best ADC combination for the given pins
// returns the index of the best ADC 
// each pin can be connected to multiple ADCs
// the function will try to find a single ADC that can be used for all pins
// if not possible it will return nullptr
ADC_TypeDef* _findBestADCForPins(int numPins, int pins[]) {

  // assuning that there is less than 8 ADCs
  uint8_t pins_at_adc[ADC_COUNT] = {0};

  // check how many pins are there and are not set
  int no_pins = 0;
  for (int i = 0; i < numPins; i++) {
    if(_isset(pins[i])) no_pins++;
  }

  // loop over all elements and count the pins connected to each ADC
  for (int i = 0; i < numPins; i++) {
    int pin = pins[i];
    if(!_isset(pin)) continue;

    int index = _findIndexOfFirstPinMapADCEntry(pin);
    int last_index = _findIndexOfLastPinMapADCEntry(pin);
    if (index == -1) {
      return nullptr;
    }
    for (int j = index; j <= last_index; j++) {
      if (PinMap_ADC[j].pin == NC) {
        break;
      }
      int adcIndex = _adcToIndex((ADC_TypeDef*)PinMap_ADC[j].peripheral);
      pins_at_adc[adcIndex]++;
    }
  }

  for (int i = 0; i < ADC_COUNT; i++) {
    if(!pins_at_adc[i]) continue;
    SimpleFOCDebug::print("STM32-CS: ADC");
    SimpleFOCDebug::print(i+1);
    SimpleFOCDebug::print(" pins: ");
    SimpleFOCDebug::println(pins_at_adc[i]);
  }

  // now take the first ADC that has all pins connected
  for (int i = 0; i < ADC_COUNT; i++) {
    if (pins_at_adc[i] == no_pins) {
      return _indexToADC(i);
    }
  }
  return nullptr;
}



/**
  * @brief  Return ADC HAL channel linked to a PinName
  * @param  pin: PinName
  * @retval Valid HAL channel
  */
uint32_t _getADCChannelFromPinMap(PinName pin)
{
  uint32_t function = pinmap_function(pin, PinMap_ADC);
  uint32_t channel = 0;
  switch (STM_PIN_CHANNEL(function)) {
#ifdef ADC_CHANNEL_0
    case 0:
      channel = ADC_CHANNEL_0;
      break;
#endif
    case 1:
      channel = ADC_CHANNEL_1;
      break;
    case 2:
      channel = ADC_CHANNEL_2;
      break;
    case 3:
      channel = ADC_CHANNEL_3;
      break;
    case 4:
      channel = ADC_CHANNEL_4;
      break;
    case 5:
      channel = ADC_CHANNEL_5;
      break;
    case 6:
      channel = ADC_CHANNEL_6;
      break;
    case 7:
      channel = ADC_CHANNEL_7;
      break;
    case 8:
      channel = ADC_CHANNEL_8;
      break;
    case 9:
      channel = ADC_CHANNEL_9;
      break;
    case 10:
      channel = ADC_CHANNEL_10;
      break;
    case 11:
      channel = ADC_CHANNEL_11;
      break;
    case 12:
      channel = ADC_CHANNEL_12;
      break;
    case 13:
      channel = ADC_CHANNEL_13;
      break;
    case 14:
      channel = ADC_CHANNEL_14;
      break;
    case 15:
      channel = ADC_CHANNEL_15;
      break;
#ifdef ADC_CHANNEL_16
    case 16:
      channel = ADC_CHANNEL_16;
      break;
#endif
    case 17:
      channel = ADC_CHANNEL_17;
      break;
#ifdef ADC_CHANNEL_18
    case 18:
      channel = ADC_CHANNEL_18;
      break;
#endif
#ifdef ADC_CHANNEL_19
    case 19:
      channel = ADC_CHANNEL_19;
      break;
#endif
#ifdef ADC_CHANNEL_20
    case 20:
      channel = ADC_CHANNEL_20;
      break;
    case 21:
      channel = ADC_CHANNEL_21;
      break;
    case 22:
      channel = ADC_CHANNEL_22;
      break;
    case 23:
      channel = ADC_CHANNEL_23;
      break;
#ifdef ADC_CHANNEL_24
    case 24:
      channel = ADC_CHANNEL_24;
      break;
    case 25:
      channel = ADC_CHANNEL_25;
      break;
    case 26:
      channel = ADC_CHANNEL_26;
      break;
#ifdef ADC_CHANNEL_27
    case 27:
      channel = ADC_CHANNEL_27;
      break;
    case 28:
      channel = ADC_CHANNEL_28;
      break;
    case 29:
      channel = ADC_CHANNEL_29;
      break;
    case 30:
      channel = ADC_CHANNEL_30;
      break;
    case 31:
      channel = ADC_CHANNEL_31;
      break;
#endif
#endif
#endif
    default:
      _Error_Handler("ADC: Unknown adc channel", (int)(STM_PIN_CHANNEL(function)));
      break;
  }
  return channel;
}

/**
  * @brief  Return ADC HAL channel linked to a PinName and the ADC handle
  * @param  pin: PinName
  * @param AdcHandle: ADC_HandleTypeDef a pointer to the ADC handle
  * @retval Valid HAL channel
  */
uint32_t _getADCChannel(PinName pin, ADC_TypeDef *AdcHandle )
{  
  if (AdcHandle == NP) {
    return _getADCChannelFromPinMap(pin);
  }
  // find the PinName that corresponds to the ADC 
  int first_ind = _findIndexOfFirstEntry(pin);
  int last_ind = _findIndexOfLastEntry(pin);
  if (first_ind == -1 || last_ind == -1) {
    _Error_Handler("ADC: Pin not found in PinMap_ADC", (int)pin);
  }
  // find the channel
  uint32_t channel = 0;
  for (int i = first_ind; i <= last_ind; i++) {
    if (PinMap_ADC[i].peripheral == AdcHandle) {
      channel =_getADCChannelFromPinMap(PinMap_ADC[i].pin);
      SIMPLEFOC_DEBUG("STM32-CS: ADC channel: ", (int)STM_PIN_CHANNEL(pinmap_function(PinMap_ADC[i].pin, PinMap_ADC)));
      break;
    }
  }
  return channel;
}

uint32_t _getADCInjectedRank(uint8_t ind){
  switch (ind) {
    case 0:
      return ADC_INJECTED_RANK_1;
      break;
    case 1:
      return ADC_INJECTED_RANK_2;
      break;
    case 2:
      return ADC_INJECTED_RANK_3;
      break;
    case 3:
      return ADC_INJECTED_RANK_4;
      break;
    default:
      return 0;
      break;
  }
}

// returns 0 if no interrupt is needed, 1 if interrupt is needed
uint32_t _initTimerInterruptDownsampling(Stm32CurrentSenseParams* cs_params, STM32DriverParams* driver_params, Stm32AdcInterruptConfig& adc_interrupt_config){

  // If DIR is 0 (upcounting), the next event is high-side active (PWM rising edge)
  // If DIR is 1 (downcounting), the next event is low-side active (PWM falling edge)
  bool next_event_high_side = (cs_params->timer_handle->Instance->CR1 & TIM_CR1_DIR) == 0;
  
  // if timer has repetition counter - it will downsample using it
  // and it does not need the software downsample
  if( IS_TIM_REPETITION_COUNTER_INSTANCE(cs_params->timer_handle->Instance) ){
    // adjust the initial timer state such that the trigger 
    //   - only necessary for the timers that have repetition counters
    //   - basically make sure that the next trigger event is the one that is expected (high-side first then low-side)

    // set the direction and the 
    for(int i=0; i< 6; i++){
      if(driver_params->timers_handle[i] == NP) continue; // skip if not set
      if(next_event_high_side){
        // Set DIR bit to 0 (downcounting)
        driver_params->timers_handle[i]->Instance->CR1 |= TIM_CR1_DIR;
        // Set CNT to ARR so it starts upcounting from the top
        driver_params->timers_handle[i]->Instance->CNT =  driver_params->timers_handle[i]->Instance->ARR;
      }else{
        // Set DIR bit to 0 (upcounting)
        driver_params->timers_handle[i]->Instance->CR1 &= ~TIM_CR1_DIR;
        // Set CNT to ARR so it starts upcounting from zero
        driver_params->timers_handle[i]->Instance->CNT = 0;// driver_params->timers_handle[i]->Instance->ARR;
      }
    }
    return 0; // no interrupt is needed, the timer will handle the downsampling
  }else{
    if(!adc_interrupt_config.use_adc_interrupt){
      // If the timer has no repetition counter, it needs to use the interrupt to downsample for low side sensing
      adc_interrupt_config.use_adc_interrupt = 1;
      // remember that this timer does not have the repetition counter - need to downasmple
      adc_interrupt_config.needs_downsample = 1;

      if(next_event_high_side) // Next event is high-side active
        adc_interrupt_config.tim_downsample = 0; // skip the next interrupt (and every second one)
      else // Next event is low-side active
        adc_interrupt_config.tim_downsample = 1; // read the next one (and every second one after)
      
      return 1; // interrupt is needed
    }
  }
  return 1; // interrupt is needed
}

// returns 0 if no downsampling is needed, 1 if downsampling is needed, 2 if error
uint8_t _handleInjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle, Stm32AdcInterruptConfig& adc_interrupt_config, uint32_t adc_val[4]) {

  // if the timer han't repetition counter - downsample two times
  if( adc_interrupt_config.needs_downsample && adc_interrupt_config.tim_downsample++ > 0) {
    adc_interrupt_config.tim_downsample = 0;
    return 1;
  }
  
  adc_val[0]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_1);
  adc_val[1]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_2);
  adc_val[2]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_3);  
  adc_val[3]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_4);  
  
  return 0; // no downsampling needed
}

// reads the ADC injected voltage for the given pin
// returns the voltage 
// if the pin is not found in the current sense parameters, returns 0
float _readADCInjectedChannelVoltage(int pin, void* cs_params, Stm32AdcInterruptConfig& adc_interrupt_config, uint32_t adc_val[4]) {
  Stm32CurrentSenseParams* cs_p = (Stm32CurrentSenseParams*)cs_params;
  uint8_t channel_no = 0;
  uint8_t adc_index = (uint8_t)_adcToIndex(cs_p->adc_handle);
  for(int i=0; i < 3; i++){
    if( pin == cs_p->pins[i]){ // found in the buffer
      if (adc_interrupt_config.use_adc_interrupt){
        return adc_val[channel_no] * cs_p->adc_voltage_conv;
      }else{
        // an optimized way to go from i to the channel i=0 -> channel 1, i=1 -> channel 2, i=2 -> channel 3
        uint32_t channel = _getADCInjectedRank(channel_no);
        return HAL_ADCEx_InjectedGetValue(cs_p->adc_handle, channel) * cs_p->adc_voltage_conv;
      }
    }
    if(_isset(cs_p->pins[i])) channel_no++;
  } 
  return 0; // pin not found
}

#endif