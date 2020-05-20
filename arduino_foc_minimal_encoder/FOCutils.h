#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"

// sign function
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038

// High PWM frequency
void setPwmFrequency(int pin);

// function buffering delay() 
// arduino function doesn't work well with interrupts
void _delay(unsigned long ms);

// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros();

// function approximating the sine calculation by using fixed size array
// ~40us
// it has to receive an angle in between 0 and 2PI
float _sin(float a);
// function approximating cosine calculation by using fixed size array
// ~50us
// it has to receive an angle in between 0 and 2PI
float _cos(float a);

#endif