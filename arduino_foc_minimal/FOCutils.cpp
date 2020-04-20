#include "FOCutils.h"

/*
  High PWM frequency
*/
void setPwmFrequency(int pin) {
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if (pin == 5 || pin == 6) {
      // configure the pwm phase-corrected mode
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
      // set prescaler to 1
      TCCR0B = ((TCCR0B & 0b11110000) | 0x01);
    } else {
      // set prescaler to 1
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    }
  }
  else if (pin == 3 || pin == 11) {
      // set prescaler to 1
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);
  }
}

// funciton buffering delay() 
// arduino funciton doesn't work well with interrupts
void _delay(unsigned long ms){
  long t = _micros();
  while((_micros() - t)/1000 < ms){};
}


// funciton buffering _micros() 
// arduino funciton doesn't work well with interrupts
unsigned long _micros(){
    // return teh value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
}