#include "../hardware_api.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  // if mcu is not atmega328

#elif defined(__AVR_ATmega2560__) // if mcu is not atmega2560

#elif defined(__arm__) && defined(CORE_TEENSY)  // or teensy

#elif defined(__arm__) && defined(__SAM3X8E__)  // or due

#elif defined(ESP_H)  // or esp32

#elif defined(_STM32_DEF_) // or stm32

#elif defined(_SAMD21_)  // samd21

#elif defined(_SAMD51_)  // samd51

#elif defined(__SAME51J19A__) || defined(__ATSAME51J19A__)  // samd51

#elif defined(TARGET_RP2040)

#else

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
// in generic case dont do anything
void _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  return;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  return;
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
// in generic case dont do anything
void _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  return;
}

// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  return -1;
}


// function setting the pwm duty cycle to the hardware 
// - Stepper motor - 2PWM setting
// - hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0*dc_a);
  analogWrite(pinB, 255.0*dc_b);
}

// function setting the pwm duty cycle to the hardware 
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0*dc_a);
  analogWrite(pinB, 255.0*dc_b);
  analogWrite(pinC, 255.0*dc_c);
}

// function setting the pwm duty cycle to the hardware  
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pin1A, 255.0*dc_1a);
  analogWrite(pin1B, 255.0*dc_1b);
  analogWrite(pin2A, 255.0*dc_2a);
  analogWrite(pin2B, 255.0*dc_2b);
}


// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
  return;
}


#endif
