#ifndef HARDWARE_UTILS_H
#define HARDWARE_UTILS_H

#include "Arduino.h"
#include "foc_utils.h"


#if defined(ESP_H) // if esp32 boards

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#endif

/** 
 * High PWM frequency setting function
 * - hardware specific
 * 
 * @param pinA pinA number to configure
 * @param pinB pinB number to configure
 * @param pinC pinC number to configure
 * @param pinD pinC number to configure
 */
void _setPwmFrequency(int pinA, int pinB, int pinC, int pinD = 0);

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
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * hardware specific
 * 
 * @param dc_1a  duty cycle phase 1A [0, 1]
 * @param dc_1b  duty cycle phase 1B [0, 1]
 * @param dc_2a  duty cycle phase 2A [0, 1]
 * @param dc_2b  duty cycle phase 2B [0, 1]
 * @param pin1A  phase 1A hardware pin number
 * @param pin1B  phase 1B hardware pin number
 * @param pin2A  phase 2A hardware pin number
 * @param pin2B  phase 2B hardware pin number
 */ 
void _writeDutyCycle(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B);

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


#endif