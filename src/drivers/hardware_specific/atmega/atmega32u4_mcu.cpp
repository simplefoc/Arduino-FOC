
#include "../../hardware_api.h"

#if defined(__AVR_ATmega32U4__)

#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino/ATmega32U4")
#pragma message("")

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin){
  //  High PWM frequency
  // reference： http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html
  if (pin == 3 || pin == 11 ) {
    TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
    TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1
  }
  else if (pin == 9 || pin == 10 )
    TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
  else if (pin == 5 )
    TCCR3B = ((TCCR3B & 0b11111000) | 0x01);     // set prescaler to 1
  else if ( pin == 6 || pin == 13 ) {  // a bit more complicated 10 bit timer
    // PLL Configuration
    PLLFRQ= ((PLLFRQ & 0b11001111) | 0x20); // Use 96MHz / 1.5 = 64MHz
    TCCR4B = ((TCCR4B & 0b11110000) | 0xB); // configure prescaler to get 64M/2/1024 = 31.25 kHz
    TCCR4D = ((TCCR4D & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
    
    if (pin == 6) TCCR4A = 0x82; // activate channel A - pin 13
    else if (pin == 13) TCCR4C |= 0x09;  // Activate channel D - pin 6
  }  

}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure1PWM(long pwm_frequency,const int pinA) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  _pinHighFrequency(pinC);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}



// function setting the pwm duty cycle to the hardware 
// - Stepper motor - 2PWM setting
// - hardware speciffic
void _writeDutyCycle1PWM(float dc_a, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
}


// function setting the pwm duty cycle to the hardware 
// - Stepper motor - 2PWM setting
// - hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
}

// function setting the pwm duty cycle to the hardware 
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_c);
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pin1A);
  _pinHighFrequency(pin1B);
  _pinHighFrequency(pin2A);
  _pinHighFrequency(pin2B);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin1A, pin1B, pin2A, pin2B },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the pwm duty cycle to the hardware  
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_1a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_1b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_2a);
  analogWrite(((GenericDriverParams*)params)->pins[3], 255.0f*dc_2b);
}




// function configuring pair of high-low side pwm channels, 32khz frequency and center aligned pwm
int _configureComplementaryPair(int pinH, int pinL) {
  if( (pinH == 3 && pinL == 11 ) || (pinH == 11 && pinL == 3 ) ){
    // configure the pwm phase-corrected mode
    TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 11 ) TCCR0A = 0b10110000 | (TCCR0A & 0b00001111) ;
    else TCCR0A = 0b11100000 | (TCCR0A & 0b00001111) ;
    // set prescaler to 1 - 32kHz freq
    TCCR0B = ((TCCR0B & 0b11110000) | 0x01);
  }else if( (pinH == 9 && pinL == 10 ) || (pinH == 10 && pinL == 9 ) ){
    // set prescaler to 1 - 32kHz freq
    TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 9 ) TCCR1A = 0b10110000 | (TCCR1A & 0b00001111) ;
    else TCCR1A = 0b11100000 | (TCCR1A & 0b00001111) ;
  }else if((pinH == 6 && pinL == 13 ) || (pinH == 13 && pinL == 6 ) ){
    // PLL Configuration
    PLLFRQ= ((PLLFRQ & 0b11001111) | 0x20); // Use 96MHz / 1.5 = 64MHz
    TCCR4B = ((TCCR4B & 0b11110000) | 0xB); // configure prescaler to get 64M/2/1024 = 31.25 kHz
    TCCR4D = ((TCCR4D & 0b11111100) | 0x01); // configure the pwm phase-corrected mode

    // configure complementary pwm on low side
    if(pinH == 13 ){
      TCCR4A = 0x82; // activate channel A - pin 13
      TCCR4C |= 0x0d;  // Activate complementary channel D - pin 6
    }else {
      TCCR4C |= 0x09;  // Activate channel D - pin 6
      TCCR4A = 0xc2; // activate complementary channel A - pin 13
    }
  }else{
    return -1;
  }
  return 0;
}


// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  //  High PWM frequency
  // - always max 32kHz
  int ret_flag = 0;
  ret_flag += _configureComplementaryPair(pinA_h, pinA_l);
  ret_flag += _configureComplementaryPair(pinB_h, pinB_l);
  ret_flag += _configureComplementaryPair(pinC_h, pinC_l);
  if (ret_flag!=0) return SIMPLEFOC_DRIVER_INIT_FAILED;
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency,
    .dead_zone = dead_zone
  };
  return params;
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
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _setPwmPair(((GenericDriverParams*)params)->pins[0], ((GenericDriverParams*)params)->pins[1], dc_a*255.0, ((GenericDriverParams*)params)->dead_zone*255.0);
  _setPwmPair(((GenericDriverParams*)params)->pins[2], ((GenericDriverParams*)params)->pins[3], dc_b*255.0, ((GenericDriverParams*)params)->dead_zone*255.0);
  _setPwmPair(((GenericDriverParams*)params)->pins[4], ((GenericDriverParams*)params)->pins[5], dc_c*255.0, ((GenericDriverParams*)params)->dead_zone*255.0);

  _UNUSED(phase_state);
}

#endif
