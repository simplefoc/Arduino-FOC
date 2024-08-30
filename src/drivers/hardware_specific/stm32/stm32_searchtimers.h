#pragma once

#include "./stm32_mcu.h"

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)


int stm32_findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]);


#endif
