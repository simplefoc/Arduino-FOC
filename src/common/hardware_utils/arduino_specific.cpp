#include "../hardware_utils.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // if arduino uno and other ATmega328p chips
  //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
   if (pin == 5 || pin == 6  ) {
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
      TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1
  }
  if (pin == 9 || pin == 10 )
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
  if (pin == 3 || pin == 11) 
      TCCR2B = ((TCCR2B & 0b11111000) | 0x01);// set prescaler to 1
  
#endif
}


// function setting the high pwm frequency to the supplied pins
// - hardware speciffic
// supports Arudino/ATmega328, STM32 and ESP32 
void _setPwmFrequency(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // if arduino uno and other ATmega328p chips
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  _pinHighFrequency(pinC);
  if(pinD != NOT_SET) _pinHighFrequency(pinD); // stepper motor
#endif
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0*dc_a);
  analogWrite(pinB, 255.0*dc_b);
  analogWrite(pinC, 255.0*dc_c);
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pin1A, 255.0*dc_1a);
  analogWrite(pin1B, 255.0*dc_1b);
  analogWrite(pin2A, 255.0*dc_2a);
  analogWrite(pin2B, 255.0*dc_2b);
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // if arduino uno and other atmega328p chips
  // use while instad of delay, 
  // due to wrong measurement based on changed timer0
  unsigned long t = _micros() + ms*1000;
  while( _micros() < t ){}; 
#else
  // regular micros
  delay(ms);
#endif
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// if arduino uno and other atmega328p chips
    //return the value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
#else
  // regular micros
  return micros();
#endif
}

#endif