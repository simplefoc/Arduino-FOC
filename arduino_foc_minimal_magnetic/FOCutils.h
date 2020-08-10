#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"


#if defined(ESP_H) // if esp32 boards

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#endif

// sign function
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038

/** 
 * High PWM frequency setting function
 * - hardware specific
 * 
 * @param pinA pinA number to configure
 * @param pinB pinB number to configure
 * @param pinC pinC number to configure
 */
void _setPwmFrequency(int pinA, int pinB, int pinC);

/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 * @param pinC  phase C hardware pin number
 */ 
void _writeDutyCycle(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC );


/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 * 
 * @param a angle in between 0 and 2PI
 */
float _sin(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 * 
 * @param a angle in between 0 and 2PI
 */
float _cos(float a);

#endif