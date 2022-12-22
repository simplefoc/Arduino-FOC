#include "../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP8266)

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin){
  analogWrite(pin, 0);
  analogWriteFreq(freq);
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  _setHighFrequency(pwm_frequency, pinC);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  _setHighFrequency(pwm_frequency, pinC);
  _setHighFrequency(pwm_frequency, pinD);
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC, pinD },
    .pwm_frequency = pwm_frequency
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
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_c);
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

#endif
