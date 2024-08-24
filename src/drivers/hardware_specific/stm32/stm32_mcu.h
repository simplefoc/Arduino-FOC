#pragma once

#include "../../hardware_api.h"

#if defined(_STM32_DEF_) || defined(TARGET_PORTENTA_H7)

#ifndef SIMPLEFOC_STM32_MAX_TIMERSUSED
#define SIMPLEFOC_STM32_MAX_TIMERSUSED 6
#endif
#ifndef SIMPLEFOC_STM32_MAX_MOTORSUSED
#define SIMPLEFOC_STM32_MAX_MOTORSUSED 4
#endif


#ifndef SIMPLEFOC_STM32_DEBUG
// comment me out to disable debug output
#define SIMPLEFOC_STM32_DEBUG
#endif

#if defined(__MBED__)
#define PinMap_TIM PinMap_PWM
#define ALTX_MASK 0
#endif



// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_RANGE 4095.0f // 2^12 -1 = 4095
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// 6pwm parameters
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1


typedef struct STM32DriverParams {
  TIM_HandleTypeDef* timers_handle[6] = {NULL};
  uint32_t channels[6];
  uint32_t llchannels[6];
  long pwm_frequency;
  uint8_t num_timers;
  TIM_HandleTypeDef* master_timer = NULL;
  float dead_zone;
  uint8_t interface_type;
} STM32DriverParams;


// timer allocation functions
int stm32_getNumTimersUsed();
int stm32_getNumMotorsUsed();
STM32DriverParams* stm32_getMotorUsed(int index);
bool stm32_isTimerUsed(TIM_HandleTypeDef* timer);
bool stm32_isChannelUsed(PinMap* pin);
TIM_HandleTypeDef* stm32_getTimer(PinMap* timer);
TIM_HandleTypeDef* stm32_useTimer(PinMap* timer);

void stm32_pause(STM32DriverParams* params);
void stm32_resume(STM32DriverParams* params);

// // timer synchornisation functions
// void _stopTimers(TIM_HandleTypeDef **timers_to_stop, int timer_num=6);
// void _startTimers(TIM_HandleTypeDef **timers_to_start, int timer_num=6);

// // timer query functions
// bool _getPwmState(void* params);

#endif