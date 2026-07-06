
#pragma once

#include "./stm32_mcu.h"

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)

#include "stm32_def.h"

#if defined(HAL_TIM_MODULE_ONLY)
void enableTimerClock(TIM_HandleTypeDef *htim);

constexpr uint8_t getTimerClkSrc(TIM_TypeDef *tim)
{
  uint8_t clkSrc = 0;

  if (tim != (TIM_TypeDef *)NC)
#if defined(STM32C0xx) || defined(STM32F0xx) || defined(STM32G0xx)
    /* TIMx source CLK is PCKL1 */
    clkSrc = 1;
#else
  {
    /* Get source clock depending on TIM instance */
    switch ((uintptr_t)tim) {
#if defined(TIM2_BASE)
      case TIM2_BASE:
#endif
#if defined(TIM3_BASE)
      case TIM3_BASE:
#endif
#if defined(TIM4_BASE)
      case TIM4_BASE:
#endif
#if defined(TIM5_BASE)
      case TIM5_BASE:
#endif
#if defined(TIM6_BASE)
      case TIM6_BASE:
#endif
#if defined(TIM7_BASE)
      case TIM7_BASE:
#endif
#if defined(TIM12_BASE)
      case TIM12_BASE:
#endif
#if defined(TIM13_BASE)
      case TIM13_BASE:
#endif
#if defined(TIM14_BASE)
      case TIM14_BASE:
#endif
#if defined(TIM18_BASE)
      case TIM18_BASE:
#endif
        clkSrc = 1;
        break;
#if defined(TIM1_BASE)
      case TIM1_BASE:
#endif
#if defined(TIM8_BASE)
      case TIM8_BASE:
#endif
#if defined(TIM9_BASE)
      case TIM9_BASE:
#endif
#if defined(TIM10_BASE)
      case TIM10_BASE:
#endif
#if defined(TIM11_BASE)
      case TIM11_BASE:
#endif
#if defined(TIM15_BASE)
      case TIM15_BASE:
#endif
#if defined(TIM16_BASE)
      case TIM16_BASE:
#endif
#if defined(TIM17_BASE)
      case TIM17_BASE:
#endif
#if defined(TIM19_BASE)
      case TIM19_BASE:
#endif
#if defined(TIM20_BASE)
      case TIM20_BASE:
#endif
#if defined(TIM21_BASE)
      case TIM21_BASE:
#endif
#if defined(TIM22_BASE)
      case TIM22_BASE:
#endif
        clkSrc = 2;
        break;
      default:
        _Error_Handler("TIM: Unknown timer instance", (int)tim);
        break;
    }
  }
#endif
  return clkSrc;
}
#endif

void stm32_pauseTimer(TIM_HandleTypeDef* handle);
void stm32_resumeTimer(TIM_HandleTypeDef* handle);
void stm32_refreshTimer(TIM_HandleTypeDef* handle);
void stm32_pauseChannel(TIM_HandleTypeDef* handle, uint32_t llchannels);
void stm32_resumeChannel(TIM_HandleTypeDef* handle, uint32_t llchannels);
uint32_t stm32_setClockAndARR(TIM_HandleTypeDef* handle, uint32_t PWM_freq);
uint8_t stm32_countTimers(TIM_HandleTypeDef *timers[], uint8_t num_timers);
uint8_t stm32_distinctTimers(TIM_HandleTypeDef* timers_in[], uint8_t num_timers, TIM_HandleTypeDef* timers_out[]);
uint32_t stm32_getHALChannel(uint32_t channel);
uint32_t stm32_getLLChannel(PinMap* timer);
int stm32_getInternalSourceTrigger(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave);
TIM_HandleTypeDef* stm32_alignTimers(TIM_HandleTypeDef *timers_in[], uint8_t num_timers_in);
void stm32_setPwm(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
uint32_t stm32_getTimerClockFreq(TIM_HandleTypeDef* handle);

#if defined(__MBED__)
void enableTimerClock(TIM_HandleTypeDef *htim);
uint8_t getTimerClkSrc(TIM_TypeDef *tim);
#endif

#if defined(SIMPLEFOC_STM32_DEBUG)
void stm32_printTimerCombination(int numPins, PinMap* timers[], int score);
int stm32_getTimerNumber(TIM_TypeDef *instance);
#endif

#endif
