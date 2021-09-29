#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/** 
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 */
void _configure2PWM(long pwm_frequency, const int pinA, const int pinB);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 3PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 * @param pinC pinC bldc driver
 */
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 4PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin1A pin1A stepper driver
 * @param pin1B pin1B stepper driver
 * @param pin2A pin2A stepper driver
 * @param pin2B pin2B stepper driver
 */
void _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 6PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param dead_zone  duty cycle protection zone [0, 1] - both low and high side low - if applicable
 * @param pinA_h pinA high-side bldc driver 
 * @param pinA_l pinA low-side bldc driver 
 * @param pinB_h pinA high-side bldc driver 
 * @param pinB_l pinA low-side bldc driver 
 * @param pinC_h pinA high-side bldc driver 
 * @param pinC_l pinA low-side bldc driver 
 * 
 * @return 0 if config good, -1 if failed
 */
int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 */ 
void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 * @param pinC  phase C hardware pin number
 */ 
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 4PWM setting
 * - hardware specific
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
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B);


/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 6PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param dead_zone  duty cycle protection zone [0, 1] - both low and high side low
 * @param pinA_h  phase A high-side hardware pin number
 * @param pinA_l  phase A low-side hardware pin number
 * @param pinB_h  phase B high-side hardware pin number
 * @param pinB_l  phase B low-side hardware pin number
 * @param pinC_h  phase C high-side hardware pin number
 * @param pinC_l  phase C low-side hardware pin number
 * 
 */ 
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l);

#endif