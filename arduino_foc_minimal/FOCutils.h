#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"

// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI_2 1.57079632679
#define _2PI 6.28318530718

// High PWM frequency
void setPwmFrequency(int pin);

// funciton buffering delay() 
// arduino funciton doesn't work well with interrupts
void _delay(unsigned long ms);

// funciton buffering _micros() 
// arduino funciton doesn't work well with interrupts
unsigned long _micros();

#endif