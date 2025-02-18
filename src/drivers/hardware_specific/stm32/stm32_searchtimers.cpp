
#include "./stm32_searchtimers.h"
#include "./stm32_timerutils.h"

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)



/*
  timer combination scoring function
  assigns a score, and also checks the combination is valid
  returns <0 if combination is invalid, >=0 if combination is valid. lower (but positive) score is better
  for 6 pwm, hardware 6-pwm is preferred over software 6-pwm
  hardware 6-pwm is possible if each low channel is the inverted counterpart of its high channel
  inverted channels are not allowed except when using hardware 6-pwm (in theory they could be but lets not complicate things)
*/
int _stm32_scoreCombination(int numPins, PinMap* pinTimers[]) {
  // check already used - TODO move this to outer loop also...
  for (int i=0; i<numPins; i++) {
    if (stm32_isChannelUsed(pinTimers[i]))
      return -2; // bad combination - timer channel already used
  }
  
  // TODO LPTIM and HRTIM are ignored for now, it would be cool to support them in the future

  // TODO penalize combinations that re-use the same timers used by other motors
  
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
    // check for inverted high-side channels
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




int _stm32_findIndexOfFirstPinMapEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  int i = 0;
  while (PinMap_TIM[i].pin!=NC) {
    if (pinName == PinMap_TIM[i].pin)
      return i;
    i++;
  }
  return -1;
}


int _stm32_findIndexOfLastPinMapEntry(int pin) {
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

int _stm32_findBestTimerCombination(int numPins, int index, int pins[], PinMap* pinTimers[]) {
  PinMap* searchArray[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
  for (int j=0;j<numPins;j++)
        searchArray[j] = pinTimers[j];
  int bestScore = NOT_FOUND;
  int startIndex = _stm32_findIndexOfFirstPinMapEntry(pins[index]);
  int endIndex = _stm32_findIndexOfLastPinMapEntry(pins[index]);
  if (startIndex == -1 || endIndex == -1) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: no timer on pin ", pins[index]);
    return -1; // pin is not connected to any timer
  }
  for (int i=startIndex;i<=endIndex;i++) {
    searchArray[index] = (PinMap*)&PinMap_TIM[i];
    int score = NOT_FOUND;
    if (index<numPins-1)
      score = _stm32_findBestTimerCombination(numPins, index+1, pins, searchArray);
    else {
      score = _stm32_scoreCombination(numPins, searchArray);
      #ifdef SIMPLEFOC_STM32_DEBUG
      stm32_printTimerCombination(numPins, searchArray, score);
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





int stm32_findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]) {
  int bestScore = _stm32_findBestTimerCombination(numPins, 0, pins, pinTimers);
  if (bestScore == NOT_FOUND) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::println("STM32-DRV: no workable combination found on these pins");
    #endif
    return -10; // no workable combination found
  }
  else if (bestScore >= 0) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::print("STM32-DRV: best: ");
    stm32_printTimerCombination(numPins, pinTimers, bestScore);
    #endif
  }
  return bestScore;
};





#endif

