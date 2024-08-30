
#pragma once

#include "./stm32_mcu.h"

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)

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
