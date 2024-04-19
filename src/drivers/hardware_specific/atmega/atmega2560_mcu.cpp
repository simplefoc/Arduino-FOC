#include "../../hardware_api.h"

#if defined(__AVR_ATmega2560__) || defined(AVR_ATmega1280)


#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino/ATmega2560 or Arduino/ATmega1280")
#pragma message("")


#define _PWM_FREQUENCY 32000
#define _PWM_FREQUENCY_MAX 32000
#define _PWM_FREQUENCY_MIN 4000

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin, const long frequency){
  bool high_fq = false;
  // set 32kHz frequency if requested freq is higher than the middle of the range (14kHz)
  // else set the 4kHz
  if(frequency >= 0.5*(_PWM_FREQUENCY_MAX-_PWM_FREQUENCY_MIN))  high_fq=true; 
  //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-ATmega2560
  //  https://forum.arduino.cc/index.php?topic=72092.0
  if (pin == 13 || pin == 4  ) {
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
      if(high_fq) TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR0B = ((TCCR0B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  }
  else if (pin == 12 || pin == 11 )
      if(high_fq) TCCR1B = ((TCCR1B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR1B = ((TCCR1B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  else if (pin == 10 || pin == 9 )
      if(high_fq) TCCR2B = ((TCCR2B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR2B = ((TCCR2B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  else if (pin == 5 || pin == 3 || pin == 2)
      if(high_fq) TCCR3B = ((TCCR3B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR3B = ((TCCR3B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  else if (pin == 8 || pin == 7 || pin == 6)
      if(high_fq) TCCR4B = ((TCCR4B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR4B = ((TCCR4B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  else if (pin == 44 || pin == 45 || pin == 46)
      if(high_fq) TCCR5B = ((TCCR5B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
      else TCCR5B = ((TCCR5B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz

}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware specific
// supports Arduino/ATmega2560
void* _configure1PWM(long pwm_frequency,const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 32kHz
  else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX); // constrain to 4-32kHz max
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA, pwm_frequency);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware specific
// supports Arduino/ATmega2560
void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) { 
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 32kHz
  else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX); // constrain to 4-32kHz max
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA, pwm_frequency);
  _pinHighFrequency(pinB, pwm_frequency);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware specific
// supports Arduino/ATmega2560
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 32kHz
  else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX); // constrain to 4-32kHz max
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA, pwm_frequency);
  _pinHighFrequency(pinB, pwm_frequency);
  _pinHighFrequency(pinC, pwm_frequency);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  // _syncAllTimers();
  return params;
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware specific
void _writeDutyCycle1PWM(float dc_a, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware specific
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware specific
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_c);
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware specific
void* _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 32kHz
  else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX); // constrain to 4-32kHz max
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pin1A,pwm_frequency);
  _pinHighFrequency(pin1B,pwm_frequency);
  _pinHighFrequency(pin2A,pwm_frequency);
  _pinHighFrequency(pin2B,pwm_frequency);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin1A, pin1B, pin2A, pin2B },
    .pwm_frequency = pwm_frequency,
    .dead_zone = 0.0f
  };
  return params;
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware specific
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params) {
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_1a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_1b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_2a);
  analogWrite(((GenericDriverParams*)params)->pins[3], 255.0f*dc_2b);
}


// function configuring pair of high-low side pwm channels, 32khz frequency and center aligned pwm
// supports Arduino/ATmega2560
// https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf
// https://docs.arduino.cc/hacking/hardware/PinMapping2560
int _configureComplementaryPair(const int pinH,const int pinL, long frequency) {
  bool high_fq = false;
  // set 32kHz frequency if requested freq is higher than the middle of the range (14kHz)
  // else set the 4kHz
  if(frequency >= 0.5*(_PWM_FREQUENCY_MAX-_PWM_FREQUENCY_MIN))  high_fq=true; 

  // configure pin pairs
  if( (pinH == 4 && pinL == 13 ) || (pinH == 13 && pinL == 4 ) ){
    // configure the pwm phase-corrected mode
    TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 13 ) TCCR0A = 0b10110000 | (TCCR0A & 0b00001111) ;
    else TCCR0A = 0b11100000 | (TCCR0A & 0b00001111) ;
    // set frequency
    if(high_fq) TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR0B = ((TCCR0B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
  }else if( (pinH == 11 && pinL == 12 ) || (pinH == 12 && pinL == 11 ) ){
    // set frequency
    if(high_fq) TCCR1B = ((TCCR1B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR1B = ((TCCR1B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
    // configure complementary pwm on low side
    if(pinH == 11 ) TCCR1A = 0b10110000 | (TCCR1A & 0b00001111) ;
    else TCCR1A = 0b11100000 | (TCCR1A & 0b00001111) ;
  }else if((pinH == 10 && pinL == 9 ) || (pinH == 9 && pinL == 10 ) ){
    // set frequency
    if(high_fq) TCCR2B = ((TCCR2B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR2B = ((TCCR2B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
    // configure complementary pwm on low side
    if(pinH == 10 ) TCCR2A = 0b10110000 | (TCCR2A & 0b00001111) ;
    else TCCR2A = 0b11100000 | (TCCR2A & 0b00001111) ;
  }else if((pinH == 5 && pinL == 2 ) || (pinH == 2 && pinL == 5 ) ){
    // set frequency
    if(high_fq) TCCR3B = ((TCCR3B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR3B = ((TCCR3B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
    // configure complementary pwm on low side
    if(pinH == 5 ) TCCR3A = 0b10110000 | (TCCR3A & 0b00001111) ;
    else TCCR3A = 0b11100000 | (TCCR3A & 0b00001111) ;
  }else if((pinH == 6 && pinL == 7 ) || (pinH == 7 && pinL == 6 ) ){
    // set frequency
    if(high_fq) TCCR4B = ((TCCR4B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR4B = ((TCCR4B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
    // configure complementary pwm on low side
    if(pinH == 6 ) TCCR4A = 0b10110000 | (TCCR4A & 0b00001111) ;
    else TCCR4A = 0b11100000 | (TCCR4A & 0b00001111) ;
  }else if((pinH == 46 && pinL == 45 ) || (pinH == 45 && pinL == 46 ) ){
    // set frequency
    if(high_fq) TCCR5B = ((TCCR5B & 0b11110000) | 0x01); // set prescaler to 1 - 32kHz
    else TCCR5B = ((TCCR5B & 0b11110000) | 0x02); // set prescaler to 2 - 4kHz
    // configure complementary pwm on low side
    if(pinH == 46 ) TCCR5A = 0b10110000 | (TCCR5A & 0b00001111) ;
    else TCCR5A = 0b11100000 | (TCCR5A & 0b00001111) ;
  }else{
    return -1;
  }
  return 0;
}

// Configuring PWM frequency, resolution and alignment
// - BLDC driver -  setting
// - hardware specific
// supports Arduino/ATmega2560
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 32kHz
  else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX); // constrain to 4-32kHz max
  //  High PWM frequency
  // - always max 32kHz
  int ret_flag = 0;
  ret_flag += _configureComplementaryPair(pinA_h, pinA_l, pwm_frequency);
  ret_flag += _configureComplementaryPair(pinB_h, pinB_l, pwm_frequency);
  ret_flag += _configureComplementaryPair(pinC_h, pinC_l, pwm_frequency);
  if (ret_flag!=0) return SIMPLEFOC_DRIVER_INIT_FAILED;
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency,
    .dead_zone = dead_zone
  };
  // _syncAllTimers();
  return params;
}

// function setting the
void _setPwmPair(int pinH, int pinL, float val, int dead_time, PhaseState ps)
{
  int pwm_h = _constrain(val-dead_time/2,0,255);
  int pwm_l = _constrain(val+dead_time/2,0,255);
  // determine the phase state and set the pwm accordingly
  // deactivate phases if needed
  if((ps == PhaseState::PHASE_OFF) || (ps == PhaseState::PHASE_LO)){
    digitalWrite(pinH, LOW);
  }else{
    analogWrite(pinH, pwm_h);
  }
  if((ps == PhaseState::PHASE_OFF) || (ps == PhaseState::PHASE_HI)){
    digitalWrite(pinL, LOW);
  }else{
    if(pwm_l == 255 || pwm_l == 0)
      digitalWrite(pinL, pwm_l ? LOW : HIGH);
    else
      analogWrite(pinL, pwm_l);
  }

}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
//  - BLDC driver - 6PWM setting
//  - hardware specific
// supports Arduino/ATmega328
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _setPwmPair(((GenericDriverParams*)params)->pins[0], ((GenericDriverParams*)params)->pins[1], dc_a*255.0, ((GenericDriverParams*)params)->dead_zone*255.0, phase_state[0]);
  _setPwmPair(((GenericDriverParams*)params)->pins[2], ((GenericDriverParams*)params)->pins[3], dc_b*255.0, ((GenericDriverParams*)params)->dead_zone*255.0, phase_state[1]);
  _setPwmPair(((GenericDriverParams*)params)->pins[4], ((GenericDriverParams*)params)->pins[5], dc_c*255.0, ((GenericDriverParams*)params)->dead_zone*255.0, phase_state[2]);
}

#endif
