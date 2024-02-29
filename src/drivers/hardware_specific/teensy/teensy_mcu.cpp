#include "teensy_mcu.h"

#if defined(__arm__) && defined(CORE_TEENSY)

#include "../../../communication/SimpleFOCDebug.h"

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin){
  analogWrite(pin, 0);
  analogWriteFrequency(pin, freq);
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware specific
void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  TeensyDriverParams* params = new TeensyDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency,
    .additional_params = nullptr
  };
  return params;
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware specific
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  TeensyDriverParams* params = new TeensyDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency,
    .additional_params = nullptr
  };
  return params;
}

// inital weak implementation of the center aligned 3pwm configuration
// teensy 4 and 3 have center aligned pwm 
__attribute__((weak)) void* _configureCenterAligned3PMW(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  return SIMPLEFOC_DRIVER_INIT_FAILED;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware specific
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // try configuring center aligned pwm
  void* p = _configureCenterAligned3PMW(pwm_frequency, pinA, pinB, pinC);
  if(p != SIMPLEFOC_DRIVER_INIT_FAILED){
    return p; // if center aligned pwm is available return the params
  }else{ // if center aligned pwm is not available use fast pwm
    SIMPLEFOC_DEBUG("TEENSY-DRV: Configuring 3PWM with fast pwm. Please consider using center aligned pwm for better performance!");
    _setHighFrequency(pwm_frequency, pinA);
    _setHighFrequency(pwm_frequency, pinB);
    _setHighFrequency(pwm_frequency, pinC);
    TeensyDriverParams* params = new TeensyDriverParams {
      .pins = { pinA, pinB, pinC },
      .pwm_frequency = pwm_frequency,
      .additional_params = nullptr
    };
  return params;
  }
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware specific
void* _configure4PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA);
  _setHighFrequency(pwm_frequency, pinB);
  _setHighFrequency(pwm_frequency, pinC);
  _setHighFrequency(pwm_frequency, pinD);
  TeensyDriverParams* params = new TeensyDriverParams {
    .pins = { pinA, pinB, pinC, pinD },
    .pwm_frequency = pwm_frequency,
    .additional_params = nullptr
  };
  return params;
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware specific
void _writeDutyCycle1PWM(float dc_a, void* params) {
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((TeensyDriverParams*)params)->pins[0], 255.0f*dc_a);
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware specific
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params) {
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((TeensyDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((TeensyDriverParams*)params)->pins[1], 255.0f*dc_b);
}

// inital weak implementation of the center aligned 3pwm configuration
// teensy 4 and 3 have center aligned pwm implementation of this function
__attribute__((weak)) void _writeCenterAligned3PMW(float dc_a,  float dc_b, float dc_c, void* params){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(params);
}


// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware specific
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){

  TeensyDriverParams* p = (TeensyDriverParams*)params;
  if(p->additional_params != nullptr){
    _writeCenterAligned3PMW(dc_a, dc_b, dc_c, p);
  }else{
    // transform duty cycle from [0,1] to [0,255]
    analogWrite(p->pins[0], 255.0f*dc_a);
    analogWrite(p->pins[1], 255.0f*dc_b);
    analogWrite(p->pins[2], 255.0f*dc_c);
  }
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware specific
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((TeensyDriverParams*)params)->pins[0], 255.0f*dc_1a);
  analogWrite(((TeensyDriverParams*)params)->pins[1], 255.0f*dc_1b);
  analogWrite(((TeensyDriverParams*)params)->pins[2], 255.0f*dc_2a);
  analogWrite(((TeensyDriverParams*)params)->pins[3], 255.0f*dc_2b);
}

#endif