
#include "../../hardware_api.h"
#include "stm32_mcu.h"

#if defined(_STM32_DEF_)


//#define SIMPLEFOC_STM32_DEBUG

#ifdef SIMPLEFOC_STM32_DEBUG
void printTimerCombination(int numPins, PinMap* timers[], int score);
int getTimerNumber(int timerIndex);
#endif




#ifndef SIMPLEFOC_STM32_MAX_PINTIMERSUSED
#define SIMPLEFOC_STM32_MAX_PINTIMERSUSED 12
#endif
int numTimerPinsUsed;
PinMap* timerPinsUsed[SIMPLEFOC_STM32_MAX_PINTIMERSUSED];






// setting pwm to hardware pin - instead analogWrite()
void _setPwm(HardwareTimer *HT, uint32_t channel, uint32_t value, int resolution)
{
  // TODO - remove commented code
  // PinName pin = digitalPinToPinName(ulPin);
  // TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  // uint32_t index = get_timer_index(Instance);
  // HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  
  HT->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}




int getLLChannel(PinMap* timer) {
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
      default: return -1;
    }
  }
  return -1;
}





// init pin pwm
HardwareTimer* _initPinPWM(uint32_t PWM_freq, PinMap* timer) {
  // sanity check
  if (timer==NP)
    return NP;
  uint32_t index = get_timer_index((TIM_TypeDef*)timer->peripheral);
  bool init = false;
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)timer->peripheral);
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    init = true;
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(timer->function);
  HT->pause();
  if (init)
    HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, timer->pin);
  #if SIMPLEFOC_PWM_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
#ifdef SIMPLEFOC_STM32_DEBUG
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring high timer ", (int)getTimerNumber(get_timer_index(HardwareTimer_Handle[index]->handle.Instance)));
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring high channel ", (int)channel);
#endif
  return HT;
}







// init high side pin
HardwareTimer* _initPinPWMHigh(uint32_t PWM_freq, PinMap* timer) {
  HardwareTimer* HT = _initPinPWM(PWM_freq, timer);
  #if SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH==false && SIMPLEFOC_PWM_ACTIVE_HIGH==true
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
  return HT;
}

// init low side pin
HardwareTimer* _initPinPWMLow(uint32_t PWM_freq, PinMap* timer)
{
  uint32_t index = get_timer_index((TIM_TypeDef*)timer->peripheral);

  bool init = false;
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)timer->peripheral);
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    init = true;
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(timer->function);

#ifdef SIMPLEFOC_STM32_DEBUG
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring low timer ", (int)getTimerNumber(get_timer_index(HardwareTimer_Handle[index]->handle.Instance)));
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring low channel ", (int)channel);
#endif

  HT->pause();
  if (init)
    HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  // sets internal fields of HT, but we can't set polarity here
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, timer->pin);
  #if SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
  return HT;
}



// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1, HardwareTimer *HT2, HardwareTimer *HT3)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
}




// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1, HardwareTimer *HT2, HardwareTimer *HT3, HardwareTimer *HT4)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT4->pause();
  HT4->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
  HT4->resume();
}




// align the timers to end the init
void _stopTimers(HardwareTimer **timers_to_stop, int timer_num)
{
  // TODO - stop each timer only once
  // stop timers
  for (int i=0; i < timer_num; i++) {
    if(timers_to_stop[i] == NP) return;
    timers_to_stop[i]->pause();
    timers_to_stop[i]->refresh();
    #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Stopping timer ", getTimerNumber(get_timer_index(timers_to_stop[i]->getHandle()->Instance)));
    #endif
  }
}

// align the timers to end the init
void _startTimers(HardwareTimer **timers_to_start, int timer_num)
{
  // TODO - sart each timer only once
  // sart timers
  for (int i=0; i < timer_num; i++) {
    if(timers_to_start[i] == NP) return;
    timers_to_start[i]->resume();
    #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Starting timer ", getTimerNumber(get_timer_index(timers_to_start[i]->getHandle()->Instance)));
    #endif
  }
}

void _alignTimersNew() {
  int numTimers = 0;
  HardwareTimer *timers[numTimerPinsUsed];

  // reset timer counters
  for (int i=0; i<numTimerPinsUsed; i++) {
    uint32_t index = get_timer_index((TIM_TypeDef*)timerPinsUsed[i]->peripheral);
    HardwareTimer *timer = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
    bool found = false;
    for (int j=0; j<numTimers; j++) {
      if (timers[j] == timer) {
        found = true;
        break;
      }
    }
    if (!found)
      timers[numTimers++] = timer;
  }

  // enable timer clock
  for (int i=0; i<numTimers; i++) {
    timers[i]->pause();
    timers[i]->refresh();
    #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Restarting timer ", getTimerNumber(get_timer_index(timers[i]->getHandle()->Instance)));
    #endif
  }

  for (int i=0; i<numTimers; i++) {
    timers[i]->resume();
  }

}




// configure hardware 6pwm for a complementary pair of channels
STM32DriverParams* _initHardware6PWMPair(long PWM_freq, float dead_zone, PinMap* pinH, PinMap* pinL, STM32DriverParams* params, int paramsPos) {
  // sanity check
  if (pinH==NP || pinL==NP)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  
#if defined(STM32L0xx) // L0 boards dont have hardware 6pwm interface 
  return SIMPLEFOC_DRIVER_INIT_FAILED; // return nothing
#endif

  uint32_t channel1 = STM_PIN_CHANNEL(pinH->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinL->function);

  // more sanity check
  if (channel1!=channel2 || pinH->peripheral!=pinL->peripheral)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  uint32_t index = get_timer_index((TIM_TypeDef*)pinH->peripheral);

  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)pinH->peripheral);
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    // HardwareTimer_Handle[index]->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow((uint32_t)PWM_freq, HERTZ_FORMAT);
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  HT->setMode(channel1, TIMER_OUTPUT_COMPARE_PWM1, pinH->pin);
  HT->setMode(channel2, TIMER_OUTPUT_COMPARE_PWM1, pinL->pin);

  // dead time is set in nanoseconds
  uint32_t dead_time_ns = (float)(1e9f/PWM_freq)*dead_zone;
  uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(HT->getHandle()->Instance), dead_time_ns);
  LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, dead_time); // deadtime is non linear!
  #if SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(pinH), LL_TIM_OCPOLARITY_LOW);
  #endif
  #if SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(pinL), LL_TIM_OCPOLARITY_LOW);
  #endif
  LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, getLLChannel(pinH) | getLLChannel(pinL));
  HT->pause();

  // make sure timer output goes to LOW when timer channels are temporarily disabled
  LL_TIM_SetOffStates(HT->getHandle()->Instance, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);

  params->timers[paramsPos] = HT;
  params->timers[paramsPos+1] = HT;
  params->channels[paramsPos] = channel1;
  params->channels[paramsPos+1] = channel2;
  return params;
}







STM32DriverParams* _initHardware6PWMInterface(long PWM_freq, float dead_zone, PinMap* pinA_h, PinMap* pinA_l, PinMap* pinB_h, PinMap* pinB_l, PinMap* pinC_h, PinMap* pinC_l) {
  STM32DriverParams* params = new STM32DriverParams {
    .timers = { NP, NP, NP, NP, NP, NP },
    .channels = { 0, 0, 0, 0, 0, 0 },
    .pwm_frequency = PWM_freq,
    .dead_zone = dead_zone,
    .interface_type = _HARDWARE_6PWM
  };

  if (_initHardware6PWMPair(PWM_freq, dead_zone, pinA_h, pinA_l, params, 0) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if(_initHardware6PWMPair(PWM_freq, dead_zone, pinB_h, pinB_l, params, 2) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if (_initHardware6PWMPair(PWM_freq, dead_zone, pinC_h, pinC_l, params, 4) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  return params;
}


STM32DriverParams* _initHardware8PWMInterface(long PWM_freq, float dead_zone, PinMap* pin1A, PinMap* pin1B, PinMap* pin2A, PinMap* pin2B, PinMap* pin3A, PinMap* pin3B, PinMap* pin4A, PinMap* pin4B) {
  STM32DriverParams* params = new STM32DriverParams {
    .timers = { NP, NP, NP, NP, NP, NP, NP, NP },
    .channels = { 0, 0, 0, 0, 0, 0, 0, 0 },
    .pwm_frequency = PWM_freq,
    .dead_zone = dead_zone,
    .interface_type = _HARDWARE_8PWM
  };

  if (_initHardware6PWMPair(PWM_freq, dead_zone, pin1A, pin1B, params, 0) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if (_initHardware6PWMPair(PWM_freq, dead_zone, pin2A, pin2B, params, 2) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if (_initHardware6PWMPair(PWM_freq, dead_zone, pin3A, pin3B, params, 4) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if (_initHardware6PWMPair(PWM_freq, dead_zone, pin4A, pin4B, params, 6) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  return params;
}




/*
  timer combination scoring function
  assigns a score, and also checks the combination is valid
  returns <0 if combination is invalid, >=0 if combination is valid. lower (but positive) score is better
  for 6 pwm, hardware 6-pwm is preferred over software 6-pwm
  hardware 6-pwm is possible if each low channel is the inverted counterpart of its high channel
  inverted channels are not allowed except when using hardware 6-pwm (in theory they could be but lets not complicate things)
*/
int scoreCombination(int numPins, PinMap* pinTimers[]) {
  // check already used - TODO move this to outer loop also...
  for (int i=0; i<numTimerPinsUsed; i++) {
    if (pinTimers[i]->peripheral == timerPinsUsed[i]->peripheral
        && STM_PIN_CHANNEL(pinTimers[i]->function) == STM_PIN_CHANNEL(timerPinsUsed[i]->function))
      return -2; // bad combination - timer channel already used
  }
  
  // TODO LPTIM and HRTIM should be ignored for now
  
  // check for inverted channels
  if (numPins < 6) {
    for (int i=0; i<numPins; i++) {
      if (STM_PIN_INVERTED(pinTimers[i]->function))
        return -3; // bad combination - inverted channel used in non-hardware 6pwm
    }
  }
  // check for duplicated channels
  for (int i=0; i<numPins-1; i++) {
    for (int j=i+1; j<numPins; j++) {
      if (pinTimers[i]->peripheral == pinTimers[j]->peripheral
          && STM_PIN_CHANNEL(pinTimers[i]->function) == STM_PIN_CHANNEL(pinTimers[j]->function)
          && STM_PIN_INVERTED(pinTimers[i]->function) == STM_PIN_INVERTED(pinTimers[j]->function))
        return -4; // bad combination - duplicated channel
    }
  }
  int score = 0;
  for (int i=0; i<numPins; i++) {
    // count distinct timers
    bool found = false;
    for (int j=i+1; j<numPins; j++) {
      if (pinTimers[i]->peripheral == pinTimers[j]->peripheral)
        found = true;
    }
    if (!found) score++;
  }
  if (numPins==6) {
    // check for inverted channels - best: 1 timer, 3 channels, 3 matching inverted channels
    //                                     >1 timer, 3 channels, 3 matching inverted channels
    //                                     1 timer, 6 channels (no inverted channels)
    //                                     >1 timer, 6 channels (no inverted channels)
    // check for inverted high-side channels - TODO is this a configuration we should allow? what if all 3 high side channels are inverted and the low-side non-inverted?
    if (STM_PIN_INVERTED(pinTimers[0]->function) || STM_PIN_INVERTED(pinTimers[2]->function) || STM_PIN_INVERTED(pinTimers[4]->function))
      return -5; // bad combination - inverted channel used on high-side channel
    if (pinTimers[0]->peripheral == pinTimers[1]->peripheral
        && pinTimers[2]->peripheral == pinTimers[3]->peripheral
        && pinTimers[4]->peripheral == pinTimers[5]->peripheral
        && STM_PIN_CHANNEL(pinTimers[0]->function) == STM_PIN_CHANNEL(pinTimers[1]->function)
        && STM_PIN_CHANNEL(pinTimers[2]->function) == STM_PIN_CHANNEL(pinTimers[3]->function)
        && STM_PIN_CHANNEL(pinTimers[4]->function) == STM_PIN_CHANNEL(pinTimers[5]->function)
        && STM_PIN_INVERTED(pinTimers[1]->function) && STM_PIN_INVERTED(pinTimers[3]->function) && STM_PIN_INVERTED(pinTimers[5]->function)) {
          // hardware 6pwm, score <10

          // TODO F37xxx doesn't support dead-time insertion, it has no TIM1/TIM8
          // F301, F302 --> 6 channels, but only 1-3 have dead-time insertion
          // TIM2/TIM3/TIM4/TIM5 don't do dead-time insertion
          // TIM15/TIM16/TIM17 do dead-time insertion only on channel 1

          // TODO check these defines
          #if defined(STM32F4xx_HAL_TIM_H) || defined(STM32F3xx_HAL_TIM_H) || defined(STM32F2xx_HAL_TIM_H) || defined(STM32F1xx_HAL_TIM_H) || defined(STM32F100_HAL_TIM_H) || defined(STM32FG0x1_HAL_TIM_H)  || defined(STM32G0x0_HAL_TIM_H) 
          if (STM_PIN_CHANNEL(pinTimers[0]->function)>3 || STM_PIN_CHANNEL(pinTimers[2]->function)>3 || STM_PIN_CHANNEL(pinTimers[4]->function)>3 )
            return -8; // channel 4 does not have dead-time insertion
          #endif
          #ifdef STM32G4xx_HAL_TIM_H
          if (STM_PIN_CHANNEL(pinTimers[0]->function)>4 || STM_PIN_CHANNEL(pinTimers[2]->function)>4 || STM_PIN_CHANNEL(pinTimers[4]->function)>4 )
            return -8; // channels 5 & 6 do not have dead-time insertion
          #endif
        }
    else {
      // check for inverted low-side channels
      if (STM_PIN_INVERTED(pinTimers[1]->function) || STM_PIN_INVERTED(pinTimers[3]->function) || STM_PIN_INVERTED(pinTimers[5]->function))
        return -6; // bad combination - inverted channel used on low-side channel in software 6-pwm
      if (pinTimers[0]->peripheral != pinTimers[1]->peripheral
        || pinTimers[2]->peripheral != pinTimers[3]->peripheral
        || pinTimers[4]->peripheral != pinTimers[5]->peripheral)
        return -7; // bad combination - non-matching timers for H/L side in software 6-pwm
      score += 10; // software 6pwm, score >10
    }
  }
  return score;
}




int findIndexOfFirstPinMapEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  int i = 0;
  while (PinMap_TIM[i].pin!=NC) {
    if (pinName == PinMap_TIM[i].pin)
      return i;
    i++;
  }
  return -1;
}


int findIndexOfLastPinMapEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  int i = 0;
  while (PinMap_TIM[i].pin!=NC) {
    if (   pinName == (PinMap_TIM[i].pin & ~ALTX_MASK) 
        && pinName != (PinMap_TIM[i+1].pin & ~ALTX_MASK))
      return i;
    i++;
  }
  return -1;
}






#define NOT_FOUND 1000

int findBestTimerCombination(int numPins, int index, int pins[], PinMap* pinTimers[]) {
  PinMap* searchArray[numPins];
  for (int j=0;j<numPins;j++)
        searchArray[j] = pinTimers[j];
  int bestScore = NOT_FOUND;
  int startIndex = findIndexOfFirstPinMapEntry(pins[index]);
  int endIndex = findIndexOfLastPinMapEntry(pins[index]);
  if (startIndex == -1 || endIndex == -1) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: no timer on pin ", pins[index]);
    return -1; // pin is not connected to any timer
  }
  for (int i=startIndex;i<=endIndex;i++) {
    searchArray[index] = (PinMap*)&PinMap_TIM[i];
    int score = NOT_FOUND;
    if (index<numPins-1)
      score = findBestTimerCombination(numPins, index+1, pins, searchArray);
    else {
      score = scoreCombination(numPins, searchArray);
      #ifdef SIMPLEFOC_STM32_DEBUG
      printTimerCombination(numPins, searchArray, score);
      #endif
    }
    if (score==-1)
      return -1; // pin not connected to any timer, propagate driectly
    if (score>=0 && score<bestScore) {
      bestScore = score;
      for (int j=index;j<numPins;j++)
        pinTimers[j] = searchArray[j];
    }
  }
  return bestScore;
}





int findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]) {
  int bestScore = findBestTimerCombination(numPins, 0, pins, pinTimers);
  if (bestScore == NOT_FOUND) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::println("STM32-DRV: no workable combination found on these pins");
    #endif
    return -10; // no workable combination found
  }
  else if (bestScore >= 0) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::print("STM32-DRV: best: ");
    printTimerCombination(numPins, pinTimers, bestScore);
    #endif
  }
  return bestScore;
};



void* _configure1PWM(long pwm_frequency, const int pinA) {
  if (numTimerPinsUsed+1 > SIMPLEFOC_STM32_MAX_PINTIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many pins used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  int pins[1] = { pinA };
  PinMap* pinTimers[1] = { NP };
  if (findBestTimerCombination(1, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinTimers[0]);\
  // allign the timers
  _alignTimersNew();
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1 },
    .channels = { channel1 },
    .pwm_frequency = pwm_frequency
  };
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[0];
  return params;
}





// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if (numTimerPinsUsed+2 > SIMPLEFOC_STM32_MAX_PINTIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many pins used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  int pins[2] = { pinA, pinB };
  PinMap* pinTimers[2] = { NP, NP };
  if (findBestTimerCombination(2, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinTimers[0]);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinTimers[1]);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT2);
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2 },
    .channels = { channel1, channel2 },
    .pwm_frequency = pwm_frequency
  };
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[0];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[1];
  return params;
}





// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if (numTimerPinsUsed+3 > SIMPLEFOC_STM32_MAX_PINTIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many pins used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  int pins[3] = { pinA, pinB, pinC };
  PinMap* pinTimers[3] = { NP, NP, NP };
  if (findBestTimerCombination(3, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinTimers[0]);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinTimers[1]);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinTimers[2]);
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
  uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2, HT3 },
    .channels = { channel1, channel2, channel3 },
    .pwm_frequency = pwm_frequency
  };


  timerPinsUsed[numTimerPinsUsed++] = pinTimers[0];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[1];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[2];

  _alignTimersNew();

  return params;
}


#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"




// Declare timer handles for TIM1 and TIM8
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

// Function to configure PWM output on TIM1 channels 1-6 and TIM8 channel 1
void* _configure8PWM(long pwm_frequency, float dead_zone)
{ 
    

    // GPIO pin initialization struct
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clock for TIM1 and TIM8
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
          // Configure PE8, PE9, PE10, PE11, PE12, PE13 as TIM1 channels
      GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

            // Configure PA14 and PB0 as TIM8 channels
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      GPIO_InitStruct.Pin = GPIO_PIN_14;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF5_TIM8;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_0;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 

    

    //Set initial values
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim1.Init.Period = 4359;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;
    if (HAL_TIM_PWM_Init(&htim1)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("TIM1 INIT OK!");}

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim8.Init.Period = 4359;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 1;
    if (HAL_TIM_PWM_Init(&htim8)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("TIM8 INIT OK!");}

    /*
    ***************************************************
    Yes, those values should produce a 38kHz PWM signal.
    what should the value be for 50Khz PWM signal?
    The value for a 50kHz PWM signal would be: htim8.Init.Period = 3279;
    And 50Khz for this: TIM1->ARR = (168000000 / (38000 * 2)) - 1; // Set auto-reload value for 38kHz frequency is egual
    No, the equation should be changed to calculate the auto-reload value for a 50kHz frequency. 
    The equation would be: TIM1->ARR = (168000000 / (50000 * 2)) - 1; // Set auto-reload value for 50kHz frequency
    *******************************************************************************************************************
    */

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("Timer TIM1 Synced OK!");}

    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("Timer TIM8 Synced OK!");}




    // Set TIM1 dead time values to 50 ns
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 50;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("TIM1 DeadTime Set");}
    
    // Set TIM8 dead time values to 50 ns
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfigTIM8 = {0};
    sBreakDeadTimeConfigTIM8.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfigTIM8.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfigTIM8.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfigTIM8.DeadTime = 50;
    sBreakDeadTimeConfigTIM8.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfigTIM8.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfigTIM8.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfigTIM8)!= HAL_OK)
    {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
    }else{Serial.println("TIM8 DeadTime Set");}

          // Configure TIM1 channels 1-6 for PWM output
      TIM_OC_InitTypeDef sConfigOC1 = {0};
      sConfigOC1.OCMode = TIM_OCMODE_PWM1;
      sConfigOC1.Pulse = 0;
      sConfigOC1.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC1.OCFastMode = TIM_OCFAST_DISABLE;

      TIM_OC_InitTypeDef sConfigOC2 = {0};
      sConfigOC2.OCMode = TIM_OCMODE_PWM2;
      sConfigOC2.Pulse = 0;
      sConfigOC2.OCPolarity = TIM_OCPOLARITY_LOW;
      sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;

      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC1, LL_TIM_CHANNEL_CH1)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH1 Config OK");}
      
      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC2, LL_TIM_CHANNEL_CH1N)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH1N Config OK");}

      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC1, LL_TIM_CHANNEL_CH2)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH2 Config OK");}

      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC2, LL_TIM_CHANNEL_CH2N)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH2N Config OK");}

      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC1, LL_TIM_CHANNEL_CH3)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH3 Config OK");}

      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC2, LL_TIM_CHANNEL_CH3N)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("CH3N Config OK");}


      // Configure PWM output on TIM8 channel 1 and additional channel
      if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC1, LL_TIM_CHANNEL_CH2)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("TIM8_CH2 Config OK");}

      if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC2, LL_TIM_CHANNEL_CH2N)!= HAL_OK)
      {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{Serial.println("TIM8_CH2N Config OK");}


/*

For TIM1, you have CH1 and CH1N as complementary outputs, CH2 and CH2N as complementary outputs, 
and CH3 and CH3N as complementary outputs.

For TIM8, you have CH2 and CH2N as complementary outputs.

Complementary outputs are a pair of signals where one is the inverse of the other, 
and they are used for driving push-pull outputs such as MOSFETs.

*/


        // Enable PWM outputs
        if( HAL_TIM_PWM_Start(&htim1, LL_TIM_CHANNEL_CH1)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH1 Start OK");}

        if (HAL_TIMEx_PWMN_Start(&htim1, LL_TIM_CHANNEL_CH1N)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH1N Start OK");}

        if (HAL_TIM_PWM_Start(&htim1, LL_TIM_CHANNEL_CH2)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH2 Start OK");}

        if (HAL_TIMEx_PWMN_Start(&htim1, LL_TIM_CHANNEL_CH2N)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH2N Start OK");}

        if(HAL_TIM_PWM_Start(&htim1, LL_TIM_CHANNEL_CH3)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH3 Start OK");}

        if( HAL_TIMEx_PWMN_Start(&htim1, LL_TIM_CHANNEL_CH3N)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("CH3N Start OK");}


        if (HAL_TIM_PWM_Start(&htim8, LL_TIM_CHANNEL_CH2)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("TIM8_CH2 Start OK");}

        if (HAL_TIMEx_PWMN_Start(&htim8, LL_TIM_CHANNEL_CH2N)!= HAL_OK)
        {return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
        }else{Serial.println("TIM8_CH2N Start OK");}

    

    // Configure TIM1 for PWM output
    TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM1->CR1 &= ~TIM_CR1_DIR; // Up counting mode
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 on OC1
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM mode 1 on OC3 and OC4
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable output on OC1, OC3, and OC4
    TIM1->PSC = 0; // Set prescaler to 0
    TIM1->ARR = (168000000 / (38000 * 2)) - 1; // Set auto-reload value for 38kHz frequency
    TIM1->CCR1 = 0; // Set duty cycle to 0%
    TIM1->CCR2 = 0; // Set duty cycle to 0%
    TIM1->CCR3 = 0; // Set duty cycle to 0%
    TIM1->CCR4 = 0; // Set duty cycle to 0%
    TIM1->BDTR |= TIM_BDTR_MOE; // Main output enable

    // Configure TIM8 for PWM output
    TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM8->CR1 &= ~TIM_CR1_DIR; // Up counting mode
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 on OC1
    TIM8->CCER |= TIM_CCER_CC1E; // Enable output on OC1
    TIM8->PSC = 0; // Set prescaler to 0
    TIM8->ARR = (168000000 / (38000 * 2)) - 1; // Set auto-reload value for 38kHz frequency
    TIM8->CCR1 = 0; // Set duty cycle to 0%
    TIM8->CCR2 = 0; // Set duty cycle to 0%
    TIM8->CCR3 = 0; // Set duty cycle to 0%
    TIM8->CCR4 = 0; // Set duty cycle to 0%
    TIM8->BDTR |= TIM_BDTR_MOE; // Main output enable

    // Start TIM1 and TIM8 PWM outputs
    TIM1->EGR |= TIM_EGR_UG; // Generate update event to load new settings
    TIM8->EGR |= TIM_EGR_UG; // Generate update event to load new settings
    TIM1->CR1 |= TIM_CR1_CEN; // Enable TIM1 counter
    TIM8->CR1 |= TIM_CR1_CEN; // Enable TIM8 counter

        
      return NULL;


}

/*

Yes, it makes sense to have these two parts next to each other. The first part of the code initializes and starts the PWM outputs of the timers (TIM1 and TIM8),
 while the second part of the code configures the timers for PWM output and sets their various parameters, such as duty cycle, frequency, and so on.

By having these two parts of the code next to each other, the programmer can ensure that the timers are properly configured and started before any other code is executed. 
This can be important in systems where the timing of signals is critical, as it can prevent race conditions and other timing issues that might arise if the timers were not properly configured 
and started before other code began executing.

Furthermore, the second part of the code relies on the first part to have started the timers before it can properly configure them. Therefore, it is logical to have these two parts of 
the code next to each other to ensure that the timers are properly initialized and configured for PWM output.

*/




        



// function setting the pwm duty cycle to the hardware
// - DC motor - 1PWM setting
// - hardware speciffic
void _writeDutyCycle1PWM(float dc_a, void* params){
  // transform duty cycle from [0,1] to [0,255]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
//- hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_c, _PWM_RESOLUTION);
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
//- hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_1a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_1b, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_2a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[3], ((STM32DriverParams*)params)->channels[3], _PWM_RANGE*dc_2b, _PWM_RESOLUTION);
}









   int period;
       

   void _writeDutyCycle8PWM(float duty_cycle1_h1, float duty_cycle1_h2, float duty_cycle2_h1, float duty_cycle2_h2,
                    float duty_cycle3_h1, float duty_cycle3_h2, float duty_cycle4_h1, float duty_cycle4_h2) {

    // Scale duty cycles to the PWM period
    uint16_t duty1_h1 = (uint16_t)(duty_cycle1_h1 * period);
    uint16_t duty1_h2 = (uint16_t)(duty_cycle1_h2 * period);
    uint16_t duty2_h1 = (uint16_t)(duty_cycle2_h1 * period);
    uint16_t duty2_h2 = (uint16_t)(duty_cycle2_h2 * period);
    uint16_t duty3_h1 = (uint16_t)(duty_cycle3_h1 * period);
    uint16_t duty3_h2 = (uint16_t)(duty_cycle3_h2 * period);
    uint16_t duty4_h1 = (uint16_t)(duty_cycle4_h1 * period);
    uint16_t duty4_h2 = (uint16_t)(duty_cycle4_h2 * period);

    // Set duty cycles for half-bridge driver 1
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH1, duty1_h1);
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH1N, duty1_h2);

    // Set duty cycles for half-bridge driver 2
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH2, duty2_h1);
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH2N, duty2_h2);

    // Set duty cycles for half-bridge driver 3
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH3, duty3_h1);
    __HAL_TIM_SET_COMPARE(&htim1, LL_TIM_CHANNEL_CH3N, duty3_h2);

    // Set duty cycles for half-bridge driver 4
    __HAL_TIM_SET_COMPARE(&htim8, LL_TIM_CHANNEL_CH2, duty4_h1);
    __HAL_TIM_SET_COMPARE(&htim8, LL_TIM_CHANNEL_CH2N, duty4_h2);


}


  

/*

Yes, you can call the "__HAL_TIM_SET_COMPARE" function without calling "HAL_TIM_PWM_Start" 
and "HAL_TIMEx_PWMN_Start" functions every time, as long as the timer 
and channel configuration have been properly set up beforehand.

The "__HAL_TIM_SET_COMPARE" function is used to set the compare value for a specific 
timer channel. This function does not start the PWM output on the channel, it simply 
sets the compare value. Once the compare value is set, the timer will automatically 
generate PWM output on the corresponding channel when the timer counter reaches the 
compare value.

Therefore, if you have already set up the timer and channel configuration and called 
the "HAL_TIM_PWM_Start" and "HAL_TIMEx_PWMN_Start" functions once, you do not need to call 
them again every time you update the compare value using "__HAL_TIM_SET_COMPARE". You can 
simply call "__HAL_TIM_SET_COMPARE" to update the compare value and the timer will generate the
 PWM output automatically on the corresponding channel.
*/




// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l, const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  if (numTimerPinsUsed+6 > SIMPLEFOC_STM32_MAX_PINTIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many pins used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to |%0kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  // find configuration
  int pins[6] = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l };
  PinMap* pinTimers[6] = { NP, NP, NP, NP, NP, NP };
  int score = findBestTimerCombination(6, pins, pinTimers);

  STM32DriverParams* params;
  // configure accordingly
  if (score<0)
    params = (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  else if (score<10)  // hardware pwm
    params = _initHardware6PWMInterface(pwm_frequency, dead_zone, pinTimers[0], pinTimers[1], pinTimers[2], pinTimers[3], pinTimers[4], pinTimers[5]);
  else {  // software pwm
    HardwareTimer* HT1 = _initPinPWMHigh(pwm_frequency, pinTimers[0]);
    HardwareTimer* HT2 = _initPinPWMLow(pwm_frequency, pinTimers[1]);
    HardwareTimer* HT3 = _initPinPWMHigh(pwm_frequency, pinTimers[2]);
    HardwareTimer* HT4 = _initPinPWMLow(pwm_frequency, pinTimers[3]);
    HardwareTimer* HT5 = _initPinPWMHigh(pwm_frequency, pinTimers[4]);
    HardwareTimer* HT6 = _initPinPWMLow(pwm_frequency, pinTimers[5]);
    uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
    uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
    uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);
    uint32_t channel4 = STM_PIN_CHANNEL(pinTimers[3]->function);
    uint32_t channel5 = STM_PIN_CHANNEL(pinTimers[4]->function);
    uint32_t channel6 = STM_PIN_CHANNEL(pinTimers[5]->function);
    params = new STM32DriverParams {
      .timers = { HT1, HT2, HT3, HT4, HT5, HT6 },
      .channels = { channel1, channel2, channel3, channel4, channel5, channel6 },
      .pwm_frequency = pwm_frequency,
      .dead_zone = dead_zone,
      .interface_type = _SOFTWARE_6PWM
    };
  }
  if (score>=0) {
    for (int i=0; i<6; i++)
      timerPinsUsed[numTimerPinsUsed++] = pinTimers[i];
    _alignTimersNew();
  }
  return params; // success
}



void _setSinglePhaseState(PhaseState state, HardwareTimer *HT, int channel1,int channel2) {
  _UNUSED(channel2);
  switch (state) {
    case PhaseState::PHASE_OFF:
      // Due to a weird quirk in the arduino timer API, pauseChannel only disables the complementary channel (e.g. CC1NE).
      // To actually make the phase floating, we must also set pwm to 0.
      HT->pauseChannel(channel1);
      break;
    default:
      HT->resumeChannel(channel1);
      break;
  }
}


// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState* phase_state, void* params){
  switch(((STM32DriverParams*)params)->interface_type){
    case _HARDWARE_6PWM:
      // phase a
      _setSinglePhaseState(phase_state[0], ((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], ((STM32DriverParams*)params)->channels[1]);
      if(phase_state[0] == PhaseState::PHASE_OFF) dc_a = 0.0f;
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
      // phase b
      _setSinglePhaseState(phase_state[1], ((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], ((STM32DriverParams*)params)->channels[3]);
      if(phase_state[1] == PhaseState::PHASE_OFF) dc_b = 0.0f;
      _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
      // phase c
      _setSinglePhaseState(phase_state[2], ((STM32DriverParams*)params)->timers[4], ((STM32DriverParams*)params)->channels[4], ((STM32DriverParams*)params)->channels[5]);
      if(phase_state[2] == PhaseState::PHASE_OFF) dc_c = 0.0f;
      _setPwm(((STM32DriverParams*)params)->timers[4], ((STM32DriverParams*)params)->channels[4], _PWM_RANGE*dc_c, _PWM_RESOLUTION);
      break;
    case _SOFTWARE_6PWM:
      float dead_zone = ((STM32DriverParams*)params)->dead_zone  / 2.0f;
      if (phase_state[0] == PhaseState::PHASE_ON || phase_state[0] == PhaseState::PHASE_HI)
        _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _constrain(dc_a - dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], 0.0f, _PWM_RESOLUTION);
      if (phase_state[0] == PhaseState::PHASE_ON || phase_state[0] == PhaseState::PHASE_LO)
        _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _constrain(dc_a + dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], 0.0f, _PWM_RESOLUTION);

      if (phase_state[1] == PhaseState::PHASE_ON || phase_state[1] == PhaseState::PHASE_HI)
        _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _constrain(dc_b - dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], 0.0f, _PWM_RESOLUTION);
      if (phase_state[1] == PhaseState::PHASE_ON || phase_state[1] == PhaseState::PHASE_LO)
        _setPwm(((STM32DriverParams*)params)->timers[3], ((STM32DriverParams*)params)->channels[3], _constrain(dc_b + dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[3], ((STM32DriverParams*)params)->channels[3], 0.0f, _PWM_RESOLUTION);

      if (phase_state[2] == PhaseState::PHASE_ON || phase_state[2] == PhaseState::PHASE_HI)
        _setPwm(((STM32DriverParams*)params)->timers[4], ((STM32DriverParams*)params)->channels[4], _constrain(dc_c - dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[4], ((STM32DriverParams*)params)->channels[4], 0.0f, _PWM_RESOLUTION);
      if (phase_state[2] == PhaseState::PHASE_ON || phase_state[2] == PhaseState::PHASE_LO)
        _setPwm(((STM32DriverParams*)params)->timers[5], ((STM32DriverParams*)params)->channels[5], _constrain(dc_c + dead_zone, 0.0f, 1.0f)*_PWM_RANGE, _PWM_RESOLUTION);
      else
        _setPwm(((STM32DriverParams*)params)->timers[5], ((STM32DriverParams*)params)->channels[5], 0.0f, _PWM_RESOLUTION);
      break;
  }
  _UNUSED(phase_state);
}



#ifdef SIMPLEFOC_STM32_DEBUG

int getTimerNumber(int timerIndex) {
  #if defined(TIM1_BASE)
    if (timerIndex==TIMER1_INDEX) return 1;
  #endif
  #if defined(TIM2_BASE)
    if (timerIndex==TIMER2_INDEX) return 2;
  #endif
  #if defined(TIM3_BASE)
    if (timerIndex==TIMER3_INDEX) return 3;
  #endif
  #if defined(TIM4_BASE)
    if (timerIndex==TIMER4_INDEX) return 4;
  #endif
  #if defined(TIM5_BASE)
    if (timerIndex==TIMER5_INDEX) return 5;
  #endif
  #if defined(TIM6_BASE)
    if (timerIndex==TIMER6_INDEX) return 6;
  #endif
  #if defined(TIM7_BASE)
    if (timerIndex==TIMER7_INDEX) return 7;
  #endif
  #if defined(TIM8_BASE)
    if (timerIndex==TIMER8_INDEX) return 8;
  #endif
  #if defined(TIM9_BASE)
    if (timerIndex==TIMER9_INDEX) return 9;
  #endif
  #if defined(TIM10_BASE)
    if (timerIndex==TIMER10_INDEX) return 10;
  #endif
  #if defined(TIM11_BASE)
    if (timerIndex==TIMER11_INDEX) return 11;
  #endif
  #if defined(TIM12_BASE)
    if (timerIndex==TIMER12_INDEX) return 12;
  #endif
  #if defined(TIM13_BASE)
    if (timerIndex==TIMER13_INDEX) return 13;
  #endif
  #if defined(TIM14_BASE)
    if (timerIndex==TIMER14_INDEX) return 14;
  #endif
  #if defined(TIM15_BASE)
    if (timerIndex==TIMER15_INDEX) return 15;
  #endif
  #if defined(TIM16_BASE)
    if (timerIndex==TIMER16_INDEX) return 16;
  #endif
  #if defined(TIM17_BASE)
    if (timerIndex==TIMER17_INDEX) return 17;
  #endif
  #if defined(TIM18_BASE)
    if (timerIndex==TIMER18_INDEX) return 18;
  #endif
  #if defined(TIM19_BASE)
    if (timerIndex==TIMER19_INDEX) return 19;
  #endif
  #if defined(TIM20_BASE)
    if (timerIndex==TIMER20_INDEX) return 20;
  #endif
  #if defined(TIM21_BASE)
    if (timerIndex==TIMER21_INDEX) return 21;
  #endif
  #if defined(TIM22_BASE)
    if (timerIndex==TIMER22_INDEX) return 22;
  #endif
  return -1;
}


void printTimerCombination(int numPins, PinMap* timers[], int score) {
  for (int i=0; i<numPins; i++) {
    if (timers[i] == NP)
      SimpleFOCDebug::print("NP");
    else {
      SimpleFOCDebug::print("TIM");
      SimpleFOCDebug::print(getTimerNumber(get_timer_index((TIM_TypeDef*)timers[i]->peripheral)));
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
