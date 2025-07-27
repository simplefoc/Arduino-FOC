
#include "./stm32_timerutils.h"
#include <Arduino.h>

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7) // if stm32duino or portenta


void stm32_pauseTimer(TIM_HandleTypeDef* handle){
  /* Disable timer unconditionally. Required to guarantee timer is stopped,
   * even if some channels are still running */
  LL_TIM_DisableCounter(handle->Instance);
//  handle->State = HAL_TIM_STATE_READY;
  // on advanced control timers there is also the option of using the brake or the MOE?
  // TIM1->EGR |= TIM_EGR_BG; // break generation
}


void stm32_resumeTimer(TIM_HandleTypeDef* handle){
  LL_TIM_EnableCounter(handle->Instance);
}


void stm32_refreshTimer(TIM_HandleTypeDef* handle){
  handle->Instance->EGR = TIM_EVENTSOURCE_UPDATE;
}


void stm32_pauseChannel(TIM_HandleTypeDef* handle, uint32_t llchannels){
    LL_TIM_CC_DisableChannel(handle->Instance, llchannels);
}


void stm32_resumeChannel(TIM_HandleTypeDef* handle, uint32_t llchannels){
    LL_TIM_CC_EnableChannel(handle->Instance, llchannels);
}








uint32_t stm32_setClockAndARR(TIM_HandleTypeDef* handle, uint32_t PWM_freq) {
  // set the clock
  uint32_t arr_value = 0;
  uint32_t cycles = stm32_getTimerClockFreq(handle) / PWM_freq / 2;
  uint32_t prescaler = (cycles / 0x10000) + 1; // TODO consider 32 bit timers
  LL_TIM_SetPrescaler(handle->Instance, prescaler - 1);
  uint32_t ticks = cycles / prescaler;
  if (ticks > 0) {
    arr_value = ticks - 1;
  }
  __HAL_TIM_SET_AUTORELOAD(handle, arr_value);
  stm32_refreshTimer(handle);
  // #ifdef SIMPLEFOC_STM32_DEBUG
  //   SIMPLEFOC_DEBUG("STM32-DRV: Timer clock: ", (int)stm32_getTimerClockFreq(handle));
  //   SIMPLEFOC_DEBUG("STM32-DRV: Timer prescaler: ", (int)prescaler);
  //   SIMPLEFOC_DEBUG("STM32-DRV: Timer ARR: ", (int)arr_value);
  // #endif
  return arr_value;
}





// setting pwm to hardware pin - instead analogWrite()
void stm32_setPwm(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value) {
  // value assumed to be in correct range
  switch (channel) {
    case 1: timer->Instance->CCR1 = value; break;
    case 2: timer->Instance->CCR2 = value; break;
    case 3: timer->Instance->CCR3 = value; break;
    case 4: timer->Instance->CCR4 = value; break;
    #ifdef LL_TIM_CHANNEL_CH5
    case 5: timer->Instance->CCR5 = value; break;
    #endif
    #ifdef LL_TIM_CHANNEL_CH6
    case 6: timer->Instance->CCR6 = value; break;
    #endif
    default: break;
  }
}


uint32_t stm32_getHALChannel(uint32_t channel) {
  uint32_t return_value;
  switch (channel) {
    case 1:
      return_value = TIM_CHANNEL_1;
      break;
    case 2:
      return_value = TIM_CHANNEL_2;
      break;
    case 3:
      return_value = TIM_CHANNEL_3;
      break;
    case 4:
      return_value = TIM_CHANNEL_4;
      break;
    #ifdef TIM_CHANNEL_5
    case 5:
      return_value = TIM_CHANNEL_5;
      break;
    #endif
    #ifdef TIM_CHANNEL_6
    case 6:
      return_value = TIM_CHANNEL_6;
      break;
    #endif
    default:
      return_value = -1;
  }
  return return_value;
}



uint32_t stm32_getLLChannel(PinMap* timer) {
#if defined(TIM_CCER_CC1NE)
  if (STM_PIN_INVERTED(timer->function)) {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1N;
      case 2: return LL_TIM_CHANNEL_CH2N;
      case 3: return LL_TIM_CHANNEL_CH3N;
#if defined(LL_TIM_CHANNEL_CH4N)
      case 4: return LL_TIM_CHANNEL_CH4N;
#endif
      default: return -1;
    }
  } else
#endif
  {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1;
      case 2: return LL_TIM_CHANNEL_CH2;
      case 3: return LL_TIM_CHANNEL_CH3;
      case 4: return LL_TIM_CHANNEL_CH4;
  #ifdef LL_TIM_CHANNEL_CH5
      case 5: return LL_TIM_CHANNEL_CH5;
  #endif
  #ifdef LL_TIM_CHANNEL_CH6
      case 6: return LL_TIM_CHANNEL_CH6;
  #endif
      default: return -1;
    }
  }
  return -1;
}



uint8_t stm32_countTimers(TIM_HandleTypeDef *timers[], uint8_t num_timers) {
  uint8_t count = 1;
  for (int i=1; i<num_timers; i++) {
    if (timers[i] != NULL) break;
    bool found = false;
    for (int j=0; j<i; j++) {
      if (timers[i] == timers[j]) { 
        found = true;
        break;
      }
    }
    if (found==false)
      count++;
  }
  return count;
}

uint8_t stm32_distinctTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers, TIM_HandleTypeDef* timers_out[]) {
  uint8_t count = 0;
  for (int i=0; i<num_timers; i++) {
    if (timers_in[i] == NULL) continue;
    bool found = false;
    for (int j=0; j<count; j++) {
      if (timers_in[i] == timers_out[j]) {
        found = true;
        break;
      }
    }
    if (found==false)
      timers_out[count++] = timers_in[i];
  }
  return count;
}



#if defined(STM32G4xx)
// function finds the appropriate timer source trigger for the master/slave timer combination
// returns -1 if no trigger source is found
// currently supports the master timers to be from TIM1 to TIM4 and TIM8
int stm32_getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave) { // put master and slave in temp variables to avoid arrows
  TIM_TypeDef *TIM_master = master->Instance;
  #if defined(TIM1) && defined(LL_TIM_TS_ITR0)
    if (TIM_master == TIM1) return LL_TIM_TS_ITR0;// return TIM_TS_ITR0;
  #endif
  #if defined(TIM2) &&  defined(LL_TIM_TS_ITR1)
    else if (TIM_master == TIM2) return LL_TIM_TS_ITR1;//return TIM_TS_ITR1;
  #endif
  #if defined(TIM3) &&  defined(LL_TIM_TS_ITR2)
    else if (TIM_master == TIM3) return LL_TIM_TS_ITR2;//return TIM_TS_ITR2;
  #endif  
  #if defined(TIM4) &&  defined(LL_TIM_TS_ITR3)
    else if (TIM_master == TIM4) return LL_TIM_TS_ITR3;//return TIM_TS_ITR3;
  #endif 
  #if defined(TIM5) &&  defined(LL_TIM_TS_ITR4)
    else if (TIM_master == TIM5) return LL_TIM_TS_ITR4;//return TIM_TS_ITR4;
  #endif
  #if defined(TIM8) &&  defined(LL_TIM_TS_ITR5)
    else if (TIM_master == TIM8) return LL_TIM_TS_ITR5;//return TIM_TS_ITR5;
  #endif
  return -1;
}
#elif defined(STM32F4xx) || defined(STM32F1xx) || defined(STM32L4xx) || defined(STM32F7xx) || defined(STM32H7xx) || defined(TARGET_STM32H7)

// function finds the appropriate timer source trigger for the master/slave timer combination
// returns -1 if no trigger source is found
// currently supports the master timers to be from
// 
//   fammilies     | timers
//   --------------| --------------------------------
//   f1,f4,f7      | TIM1 to TIM4 and TIM8
//   l4            | TIM1 to TIM4, TIM8 and TIM15
//   h7            | TIM1 to TIM5, TIM8, TIM15, TIM23 and TIM24
int stm32_getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave) {
  // put master and slave in temp variables to avoid arrows
  TIM_TypeDef *TIM_master = master->Instance;
  TIM_TypeDef *TIM_slave = slave->Instance;
  #if defined(TIM1) && defined(LL_TIM_TS_ITR0)
    if (TIM_master == TIM1){
      #if defined(TIM2)
      if(TIM_slave == TIM2) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR0;
      #endif
      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR0;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR0;
        #endif
      #endif
    }
  #endif
  #if defined(TIM2) &&  defined(LL_TIM_TS_ITR1)
    else if (TIM_master == TIM2){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR1;
      #endif
      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR1;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR1;
        #endif
      #else
        #if defined(TIM5) 
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR0;
        #endif
      #endif
    }
  #endif
  #if defined(TIM3) &&  defined(LL_TIM_TS_ITR2)
    else if (TIM_master == TIM3){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR2;
      #endif
      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR2;
        #endif
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR2;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR2;
        #endif
      #else
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
        #endif
      #endif
    }
  #endif  
  #if defined(TIM4) &&  defined(LL_TIM_TS_ITR3)
    else if (TIM_master == TIM4){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR2;
      #endif

      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR3;
        #endif
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR3;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR3;
        #endif
      #else
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR2;
        #endif
      #endif
    }
  #endif 
  #if defined(TIM5) 
    else if (TIM_master == TIM5){
      #if defined(STM32F4xx) || defined(STM32F1xx) || defined(STM32F7xx) // f1, f4 adn f7 have tim5 sycned with tim1 and tim3 while others (l4, h7) have tim15
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR2;
      #endif
      #endif
      #if defined(TIM8)
      if(TIM_slave == TIM8) return LL_TIM_TS_ITR3;
      #endif

      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR4;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR4;
        #endif
      #endif
    }
  #endif
  #if defined(TIM8)
    else if (TIM_master == TIM8){
      #if defined(TIM2)
      if(TIM_slave==TIM2) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR3;
      #endif

      #if defined(STM32H7xx) || defined(TARGET_STM32H7)
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
        #endif
        #if defined(TIM23)
        else if(TIM_slave == TIM23) return LL_TIM_TS_ITR5;
        #endif
        #if defined(TIM24)
        else if(TIM_slave == TIM24) return LL_TIM_TS_ITR5;
        #endif
      #else
        #if defined(TIM5)
        else if(TIM_slave == TIM5) return LL_TIM_TS_ITR3;
        #endif
      #endif
    }
  #endif
  #if defined(TIM15) && (defined(STM32L4xx) || defined(STM32H7xx) || defined(TARGET_STM32H7) )
    else if (TIM_master == TIM15){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      if(TIM_slave == TIM3) return LL_TIM_TS_ITR2;
      #endif
    }
  #endif
  #if defined(TIM23) && (defined(STM32H7xx) || defined(TARGET_STM32H7))
    else if (TIM_master == TIM23){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM2)
      if(TIM_slave == TIM2) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM3)
      if(TIM_slave == TIM3) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM4)
      if(TIM_slave == TIM4) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM5)
      if(TIM_slave == TIM5) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM8)
      if(TIM_slave == TIM8) return LL_TIM_TS_ITR12;
      #endif
      #if defined(TIM24)
      if(TIM_slave == TIM24) return LL_TIM_TS_ITR12;
      #endif
    }
  #endif
  #if defined(TIM24) && (defined(STM32H7xx) || defined(TARGET_STM32H7))
    else if (TIM_master == TIM24){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM2)
      if(TIM_slave == TIM2) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM3)
      if(TIM_slave == TIM3) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM4)
      if(TIM_slave == TIM4) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM5)
      if(TIM_slave == TIM5) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM8)
      if(TIM_slave == TIM8) return LL_TIM_TS_ITR13;
      #endif
      #if defined(TIM23)
      if(TIM_slave == TIM23) return LL_TIM_TS_ITR13;
      #endif
    }
  #endif
  return -1; // combination not supported
}
#else
// Alignment not supported for this architecture
int stm32_getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave) {
  return -1;
}
#endif





// call with the timers used for a motor. The function will align the timers and
// start them at the same time (if possible). Results of this function can be:
//  - success, no changes needed
//  - success, different timers aligned and started concurrently
//  - failure, cannot align timers
// in each case, the timers are started.
//
// TODO: this topic is quite complex if we allow multiple motors and multiple timers per motor.
//       that's because the motors could potentially share the same timer. In this case aligning
//       the timers is problematic.
//       Even more problematic is stopping and resetting the timers. Even with a single motor,
//       this would cause problems and mis-align the PWM signals.
// We can handle three cases:
//   - only one timer needed, no need to align
//   - master timer can be found, and timers used by only this motor: alignment possible
//   - TODO all timers for all motors can be aligned to one master: alignment possible
//   - otherwise, alignment not possible
// frequency alignment is based on checking their pwm frequencies match.
// pwm alignment is based on linking timers to start them at the same time, and making sure they are reset in sync.
// On newer chips supporting it (STM32G4) we use gated + reset mode to start/stop only the master timer. Slaves 
// are started/stopped automatically with the master. TODO probably just using gated mode is better
// On older chips (STM32F1) we used gated mode to start/stop the slave timers with the master, but have to take
// care of the reset manually. TODO is it really needed to reset the timers?
TIM_HandleTypeDef* stm32_alignTimers(TIM_HandleTypeDef *timers_in[], uint8_t num_timers_in) {
  // find the timers used
  TIM_HandleTypeDef *timers[6];
  int numTimers = stm32_distinctTimers(timers_in, num_timers_in, timers);

  // compare with the existing timers used for other motors...
  for (int i=0; i<numTimers; i++) {
    for (int j=0; j<stm32_getNumMotorsUsed(); j++) {
      STM32DriverParams* params = stm32_getMotorUsed(j);
      for (int k=0; k<6; k++) {
        if (params->timers_handle[k] == NULL) break;
        if (params->timers_handle[k] == timers[i]) {
          #ifdef SIMPLEFOC_STM32_DEBUG
            SIMPLEFOC_DEBUG("STM32-DRV: ERR: Timer already used by another motor: TIM", stm32_getTimerNumber(timers[i]->Instance));
          #endif
          // TODO handle this case, and make it a warning
          // two sub-cases we could handle at the moment:
          //  - the other motor does not have a master timer, so we can initialize this motor without a master also
        }
      }
    }
  }


  #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::print("STM32-DRV: Synchronising ");
    SimpleFOCDebug::print(numTimers);
    SimpleFOCDebug::println(" timers");
  #endif

  // see if there is more then 1 timers used for the pwm
  // if yes, try to align timers
  if(numTimers > 1){
    // find the master timer
    int16_t master_index = -1;
    int triggerEvent = -1;
    for (int i=0; i<numTimers; i++) {
      // check if timer can be master
      if(IS_TIM_MASTER_INSTANCE(timers[i]->Instance)) {
        // check if timer already configured in TRGO update mode (used for ADC triggering)
        // in that case we should not change its TRGO configuration
        if(timers[i]->Instance->CR2 & LL_TIM_TRGO_UPDATE) continue;
        // check if the timer has the supported internal trigger for other timers
        for (int slave_i=0; slave_i<numTimers; slave_i++) {
          if (i==slave_i) continue; // skip self
          // check if it has the supported internal trigger
          triggerEvent = stm32_getInternalSourceTrigger(timers[i],timers[slave_i]); 
          if(triggerEvent == -1) break; // not supported keep searching
        }
        if(triggerEvent == -1) continue; // cannot be master, keep searching
        // otherwise the master has been found, remember the index
        master_index = i; // found the master timer
        break;
      }
    }
    

    // if no master timer found do not perform alignment
    if (master_index == -1) {
      #ifdef SIMPLEFOC_STM32_DEBUG
        SIMPLEFOC_DEBUG("STM32-DRV: WARN: No master timer found, cannot align timers!");
      #endif
    }else{
      #ifdef SIMPLEFOC_STM32_DEBUG
        SIMPLEFOC_DEBUG("STM32-DRV: master timer: TIM",  stm32_getTimerNumber(timers[master_index]->Instance));
      #endif
      // make the master timer generate ITRGx event
      // if it was already configured in slave mode, disable the slave mode on the master
      LL_TIM_SetSlaveMode(timers[master_index]->Instance, LL_TIM_SLAVEMODE_DISABLED );
      // Configure the master  timer to send a trigger signal on enable 
      LL_TIM_SetTriggerOutput(timers[master_index]->Instance, LL_TIM_TRGO_ENABLE);
      //LL_TIM_EnableMasterSlaveMode(timers[master_index]->Instance);
      
      // configure other timers to get the input trigger from the master timer
      for (int slave_index=0; slave_index < numTimers; slave_index++) {
        if (slave_index == master_index)
          continue;
        #ifdef SIMPLEFOC_STM32_DEBUG
          SIMPLEFOC_DEBUG("STM32-DRV: slave timer: TIM",  stm32_getTimerNumber(timers[slave_index]->Instance));
        #endif
        // Configure the slave timer to be triggered by the master enable signal
        uint32_t trigger = stm32_getInternalSourceTrigger(timers[master_index], timers[slave_index]);
        // #ifdef SIMPLEFOC_STM32_DEBUG
        //   SIMPLEFOC_DEBUG("STM32-DRV: slave trigger ITR ", (int)trigger);
        // #endif
        LL_TIM_SetTriggerInput(timers[slave_index]->Instance, trigger);
        // #if defined(STM32G4xx)
        // LL_TIM_SetSlaveMode(timers[slave_index]->Instance, LL_TIM_SLAVEMODE_COMBINED_GATEDRESET);
        // #else
        LL_TIM_SetSlaveMode(timers[slave_index]->Instance, LL_TIM_SLAVEMODE_GATED);
        // #endif
      }
      for (int i=0; i<numTimers; i++) { // resume the timers TODO at the moment the first PWM cycle is not well-aligned
        stm32_refreshTimer(timers[i]);
        if (i != master_index)
          stm32_resumeTimer(timers[i]);
        SIMPLEFOC_DEBUG("STM32-DRV: slave counter: ", (int)timers[i]->Instance->CNT);
      }
      stm32_resumeTimer(timers[master_index]);
      return timers[master_index];
    }
  }

  // if we had only one timer, or there was no master timer found
  for (int i=0; i<numTimers; i++) // resume the timers individually
    stm32_resumeTimer(timers[i]);
  return NULL;

}





uint32_t stm32_getTimerClockFreq(TIM_HandleTypeDef *handle) {
#if defined(STM32MP1xx)
  uint8_t timerClkSrc = getTimerClkSrc(handle->Instance);
  uint64_t clkSelection = timerClkSrc == 1 ? RCC_PERIPHCLK_TIMG1 : RCC_PERIPHCLK_TIMG2;
  return HAL_RCCEx_GetPeriphCLKFreq(clkSelection);
#else
  RCC_ClkInitTypeDef    clkconfig = {};
  uint32_t              pFLatency = 0U;
  uint32_t              uwTimclock = 0U, uwAPBxPrescaler = 0U;

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  switch (getTimerClkSrc(handle->Instance)) {
    case 1:
      uwAPBxPrescaler = clkconfig.APB1CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK1Freq();
      break;
#if !defined(STM32C0xx) && !defined(STM32F0xx) && !defined(STM32G0xx)
    case 2:
      uwAPBxPrescaler = clkconfig.APB2CLKDivider;
      uwTimclock = HAL_RCC_GetPCLK2Freq();
      break;
#endif
    default:
    case 0: // Unknown timer clock source
      return 0;
  }

#if defined(STM32H7xx) || defined(TARGET_STM32H7) 
  /* When TIMPRE bit of the RCC_CFGR register is reset,
   *   if APBx prescaler is 1 or 2 then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 2x PCLKx.
   * When TIMPRE bit in the RCC_CFGR register is set,
   *   if APBx prescaler is 1,2 or 4, then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 4x PCLKx
   */
  RCC_PeriphCLKInitTypeDef PeriphClkConfig = {};
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkConfig);

  if (PeriphClkConfig.TIMPresSelection == RCC_TIMPRES_ACTIVATED) {
    switch (uwAPBxPrescaler) {
      default:
      case RCC_APB1_DIV1:
      case RCC_APB1_DIV2:
      case RCC_APB1_DIV4:
      /* case RCC_APB2_DIV1: */
      case RCC_APB2_DIV2:
      case RCC_APB2_DIV4:
        /* Note: in such cases, HCLK = (APBCLK * DIVx) */
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_APB1_DIV8:
      case RCC_APB1_DIV16:
      case RCC_APB2_DIV8:
      case RCC_APB2_DIV16:
        uwTimclock *= 4;
        break;
    }
  } else {
    switch (uwAPBxPrescaler) {
      default:
      case RCC_APB1_DIV1:
      case RCC_APB1_DIV2:
      /* case RCC_APB2_DIV1: */
      case RCC_APB2_DIV2:
        /* Note: in such cases, HCLK = (APBCLK * DIVx) */
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_APB1_DIV4:
      case RCC_APB1_DIV8:
      case RCC_APB1_DIV16:
      case RCC_APB2_DIV4:
      case RCC_APB2_DIV8:
      case RCC_APB2_DIV16:
        uwTimclock *= 2;
        break;
    }
  }
#else
  /* When TIMPRE bit of the RCC_DCKCFGR register is reset,
   *   if APBx prescaler is 1, then TIMxCLK = PCLKx,
   *   otherwise TIMxCLK = 2x PCLKx.
   * When TIMPRE bit in the RCC_DCKCFGR register is set,
   *   if APBx prescaler is 1,2 or 4, then TIMxCLK = HCLK,
   *   otherwise TIMxCLK = 4x PCLKx
   */
#if defined(STM32F4xx) || defined(STM32F7xx)
#if !defined(STM32F405xx) && !defined(STM32F415xx) &&\
    !defined(STM32F407xx) && !defined(STM32F417xx)
  RCC_PeriphCLKInitTypeDef PeriphClkConfig = {};
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkConfig);

  if (PeriphClkConfig.TIMPresSelection == RCC_TIMPRES_ACTIVATED)
    switch (uwAPBxPrescaler) {
      default:
      case RCC_HCLK_DIV1:
      case RCC_HCLK_DIV2:
      case RCC_HCLK_DIV4:
        /* Note: in such cases, HCLK = (APBCLK * DIVx) */
        uwTimclock = HAL_RCC_GetHCLKFreq();
        break;
      case RCC_HCLK_DIV8:
      case RCC_HCLK_DIV16:
        uwTimclock *= 4;
        break;
    } else
#endif
#endif
    switch (uwAPBxPrescaler) {
      default:
      case RCC_HCLK_DIV1:
        // uwTimclock*=1;
        break;
      case RCC_HCLK_DIV2:
      case RCC_HCLK_DIV4:
      case RCC_HCLK_DIV8:
      case RCC_HCLK_DIV16:
        uwTimclock *= 2;
        break;
    }
#endif /* STM32H7xx */
  return uwTimclock;
#endif /* STM32MP1xx */
}







#if defined(__MBED__)

void enableTimerClock(TIM_HandleTypeDef *htim) {
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
#endif
#if defined(TIM2_BASE)
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
#endif
#if defined(TIM3_BASE)
  if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
#endif
#if defined(TIM4_BASE)
  if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
#endif
#if defined(TIM5_BASE)
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
  }
#endif
#if defined(TIM6_BASE)
  if (htim->Instance == TIM6) {
    __HAL_RCC_TIM6_CLK_ENABLE();
  }
#endif
#if defined(TIM7_BASE)
  if (htim->Instance == TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }
#endif
#if defined(TIM8_BASE)
  if (htim->Instance == TIM8) {
    __HAL_RCC_TIM8_CLK_ENABLE();
  }
#endif
#if defined(TIM9_BASE)
  if (htim->Instance == TIM9) {
    __HAL_RCC_TIM9_CLK_ENABLE();
  }
#endif
#if defined(TIM10_BASE)
  if (htim->Instance == TIM10) {
    __HAL_RCC_TIM10_CLK_ENABLE();
  }
#endif
#if defined(TIM11_BASE)
  if (htim->Instance == TIM11) {
    __HAL_RCC_TIM11_CLK_ENABLE();
  }
#endif
#if defined(TIM12_BASE)
  if (htim->Instance == TIM12) {
    __HAL_RCC_TIM12_CLK_ENABLE();
  }
#endif
#if defined(TIM13_BASE)
  if (htim->Instance == TIM13) {
    __HAL_RCC_TIM13_CLK_ENABLE();
  }
#endif
#if defined(TIM14_BASE)
  if (htim->Instance == TIM14) {
    __HAL_RCC_TIM14_CLK_ENABLE();
  }
#endif
#if defined(TIM15_BASE)
  if (htim->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_ENABLE();
  }
#endif
#if defined(TIM16_BASE)
  if (htim->Instance == TIM16) {
    __HAL_RCC_TIM16_CLK_ENABLE();
  }
#endif
#if defined(TIM17_BASE)
  if (htim->Instance == TIM17) {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
#endif
#if defined(TIM18_BASE)
  if (htim->Instance == TIM18) {
    __HAL_RCC_TIM18_CLK_ENABLE();
  }
#endif
#if defined(TIM19_BASE)
  if (htim->Instance == TIM19) {
    __HAL_RCC_TIM19_CLK_ENABLE();
  }
#endif
#if defined(TIM20_BASE)
  if (htim->Instance == TIM20) {
    __HAL_RCC_TIM20_CLK_ENABLE();
  }
#endif
#if defined(TIM21_BASE)
  if (htim->Instance == TIM21) {
    __HAL_RCC_TIM21_CLK_ENABLE();
  }
#endif
#if defined(TIM22_BASE)
  if (htim->Instance == TIM22) {
    __HAL_RCC_TIM22_CLK_ENABLE();
  }
#endif
}


uint8_t getTimerClkSrc(TIM_TypeDef *tim) {
  uint8_t clkSrc = 0;

  if (tim != (TIM_TypeDef *)NC)
#if defined(STM32C0xx) || defined(STM32F0xx) || defined(STM32G0xx)
    /* TIMx source CLK is PCKL1 */
    clkSrc = 1;
#else
  {
    if (
      #if defined(TIM2_BASE)
            tim == TIM2 ||
      #endif
      #if defined(TIM3_BASE)
            tim == TIM3 ||
      #endif
      #if defined(TIM4_BASE)
            tim == TIM4 ||
      #endif
      #if defined(TIM5_BASE)
            tim == TIM5 ||
      #endif
      #if defined(TIM6_BASE)
            tim == TIM6 ||
      #endif
      #if defined(TIM7_BASE)
            tim == TIM7 ||
      #endif
      #if defined(TIM12_BASE)
            tim == TIM12 ||
      #endif
      #if defined(TIM13_BASE)
            tim == TIM13 ||
      #endif
      #if defined(TIM14_BASE)
            tim == TIM14 ||
      #endif
      #if defined(TIM18_BASE)
            tim == TIM18 ||
      #endif
      false)
        clkSrc = 1;
  else
    if (
      #if defined(TIM1_BASE)
            tim == TIM1 ||
      #endif
      #if defined(TIM8_BASE)
            tim == TIM8 ||
      #endif
      #if defined(TIM9_BASE)
            tim == TIM9 ||
      #endif
      #if defined(TIM10_BASE)
            tim == TIM10 ||
      #endif
      #if defined(TIM11_BASE)
            tim == TIM11 ||
      #endif
      #if defined(TIM15_BASE)
            tim == TIM15 ||
      #endif
      #if defined(TIM16_BASE)
            tim == TIM16 ||
      #endif
      #if defined(TIM17_BASE)
            tim == TIM17 ||
      #endif
      #if defined(TIM19_BASE)
            tim == TIM19 ||
      #endif
      #if defined(TIM20_BASE)
            tim == TIM20 ||
      #endif
      #if defined(TIM21_BASE)
            tim == TIM21 ||
      #endif
      #if defined(TIM22_BASE)
            tim == TIM22 ||
      #endif
      false )
        clkSrc = 2;
    else
        return 0;
  }
#endif
  return clkSrc;
}

#endif








#ifdef SIMPLEFOC_STM32_DEBUG

/*
    some utility functions to print out the timer combinations
*/

int stm32_getTimerNumber(TIM_TypeDef *instance) {
  #if defined(TIM1_BASE)
    if (instance==TIM1) return 1;
  #endif
  #if defined(TIM2_BASE)
    if (instance==TIM2) return 2;
  #endif
  #if defined(TIM3_BASE)
    if (instance==TIM3) return 3;
  #endif
  #if defined(TIM4_BASE)
    if (instance==TIM4) return 4;
  #endif
  #if defined(TIM5_BASE)
    if (instance==TIM5) return 5;
  #endif
  #if defined(TIM6_BASE)
    if (instance==TIM6) return 6;
  #endif
  #if defined(TIM7_BASE)
    if (instance==TIM7) return 7;
  #endif
  #if defined(TIM8_BASE)
    if (instance==TIM8) return 8;
  #endif
  #if defined(TIM9_BASE)
    if (instance==TIM9) return 9;
  #endif
  #if defined(TIM10_BASE)
    if (instance==TIM10) return 10;
  #endif
  #if defined(TIM11_BASE)
    if (instance==TIM11) return 11;
  #endif
  #if defined(TIM12_BASE)
    if (instance==TIM12) return 12;
  #endif
  #if defined(TIM13_BASE)
    if (instance==TIM13) return 13;
  #endif
  #if defined(TIM14_BASE)
    if (instance==TIM14) return 14;
  #endif
  #if defined(TIM15_BASE)
    if (instance==TIM15) return 15;
  #endif
  #if defined(TIM16_BASE)
    if (instance==TIM16) return 16;
  #endif
  #if defined(TIM17_BASE)
    if (instance==TIM17) return 17;
  #endif
  #if defined(TIM18_BASE)
    if (instance==TIM18) return 18;
  #endif
  #if defined(TIM19_BASE)
    if (instance==TIM19) return 19;
  #endif
  #if defined(TIM20_BASE)
    if (instance==TIM20) return 20;
  #endif
  #if defined(TIM21_BASE)
    if (instance==TIM21) return 21;
  #endif
  #if defined(TIM22_BASE)
    if (instance==TIM22) return 22;
  #endif
  return -1;
}


void stm32_printTimerCombination(int numPins, PinMap* timers[], int score) {
  for (int i=0; i<numPins; i++) {
    if (timers[i] == NULL)
      SimpleFOCDebug::print("NP");
    else {
      SimpleFOCDebug::print("TIM");
      SimpleFOCDebug::print(stm32_getTimerNumber((TIM_TypeDef*)timers[i]->peripheral));
      SimpleFOCDebug::print("-CH");
      SimpleFOCDebug::print(STM_PIN_CHANNEL(timers[i]->function));
      if (STM_PIN_INVERTED(timers[i]->function))
        SimpleFOCDebug::print("N");
    }
    SimpleFOCDebug::print(" ");
  }
  SimpleFOCDebug::println("score: ", score);
}

#endif


#endif
