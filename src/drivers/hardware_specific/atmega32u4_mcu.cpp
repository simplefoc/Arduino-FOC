
#include "../hardware_api.h"

#if defined(__AVR_ATmega32U4__)

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin){
  //  High PWM frequency
  // reference： http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html
   if ( pin == 6 || pin == 13 ) {
        TCCR4A=0x82;
        TCCR4B=2;   // TCCR4B=2  32khz; TCCR4B=1  64khz;  TTCR4B=3 16khz
        TCCR4C|=0x09;  
        TCCR4D=0;
  }
    if ( pin == 5 ){
        TCCR3B=1;
    }

    if (pin == 9 || pin == 10 ){
        TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
    }

    if (pin == 3 || pin == 11 ) {
        TCCR0B = ((TCCR0B & 0b11111000) | 0x01);  // 62kHZ
      //TCCR0B = ((TCCR0B & 0b11111000) | 0x02);  // 7.8 khz
    }
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  _pinHighFrequency(pinC);
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

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
// supports Arudino/ATmega328 
void _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pin1A);
  _pinHighFrequency(pin1B);
  _pinHighFrequency(pin2A);
  _pinHighFrequency(pin2B); 
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



// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
// supports Arudino/ATmega328 
int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
    return -1;
}

// function setting the 
void _setPwmPair(int pinH, int pinL, float val, int dead_time)
{
  int pwm_h = _constrain(val-dead_time/2,0,255);
  int pwm_l = _constrain(val+dead_time/2,0,255);
  
  analogWrite(pinH, pwm_h);
  if(pwm_l == 255 || pwm_l == 0)
    digitalWrite(pinL, pwm_l ? LOW : HIGH);
  else
    analogWrite(pinL, pwm_l);
}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
//  - BLDC driver - 6PWM setting
//  - hardware specific
// supports Arudino/ATmega328 
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
  _setPwmPair(pinA_h, pinA_l, dc_a*255.0, dead_zone*255.0);
  _setPwmPair(pinB_h, pinB_l, dc_b*255.0, dead_zone*255.0);
  _setPwmPair(pinC_h, pinC_l, dc_c*255.0, dead_zone*255.0);
}

#endif
