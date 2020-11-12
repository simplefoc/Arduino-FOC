
#include "../hardware_utils.h"

#if defined(_STM32_DEF_)

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin){
  analogWrite(pin, 0);
  analogWriteFrequency(freq); 
  analogWriteResolution(12); // resolution 12 bit 0 - 4096
}


// function setting the high pwm frequency to the supplied pins
// - hardware speciffic
// supports Arudino/ATmega328, STM32 and ESP32 
void _setPwmFrequency(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(pwm_frequency == NOT_SET) pwm_frequency = 50000; // default frequency 50khz
  else pwm_frequency = constrain(pwm_frequency, 0, 50000); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  _setHighFrequency(pwm_frequency, pinC);
  if(pinD != NOT_SET) _setHighFrequency(pwm_frequency, pinD); // stepper motor
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,4095]
  analogWrite(pinA, 4095.0*dc_a);
  analogWrite(pinB, 4095.0*dc_b);
  analogWrite(pinC, 4095.0*dc_c);
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,4095]
  analogWrite(pin1A, 4095.0*dc_1a);
  analogWrite(pin1B, 4095.0*dc_1b);
  analogWrite(pin2A, 4095.0*dc_2a);
  analogWrite(pin2B, 4095.0*dc_2b);
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
  // regular micros
  delay(ms);
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  // regular micros
  return micros();
}
#endif