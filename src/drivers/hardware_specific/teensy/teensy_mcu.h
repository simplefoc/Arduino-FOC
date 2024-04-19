#ifndef TEENSY_MCU_DRIVER_H
#define TEENSY_MCU_DRIVER_H

#include "../../hardware_api.h"

#if defined(__arm__) && defined(CORE_TEENSY)

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// debugging output 
#define SIMPLEFOC_TEENSY_DEBUG 

typedef struct TeensyDriverParams {
  int pins[6] = {(int)NOT_SET};
  long pwm_frequency;
  void* additional_params;
} TeensyDriverParams;

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin);

void* _configureCenterAligned3PMW(long pwm_frequency, const int pinA, const int pinB, const int pinC);
void _writeCenterAligned3PMW(float dc_a,  float dc_b, float dc_c, void* params);

#endif
#endif