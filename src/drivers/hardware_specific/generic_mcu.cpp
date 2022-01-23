#include "../hardware_api.h"

// if the mcu doen't have defiend analogWrite
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)
  __attribute__((weak)) void analogWrite(uint8_t pin, int value){ };
#endif

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) HardwareDriverParams _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  HardwareDriverParams params = new struct DriverParamsBase {
    .pins = { pinA, pinB, -1, -1, -1, -1 },
    .pwm_frequency = pwm_frequency,
    .initSuccess = true
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) HardwareDriverParams _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  HardwareDriverParams params = new struct DriverParamsBase {
    .pins = { pinA, pinB, pinC, -1, -1, -1 },
    .pwm_frequency = pwm_frequency,
    .initSuccess = true
  };
  return params;
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) HardwareDriverParams _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  HardwareDriverParams params = new struct DriverParamsBase {
    .pins = { pin1A, pin1B, pin2A, pin2B, -1, -1 },
    .pwm_frequency = pwm_frequency,
    .initSuccess = true
  };
  return params;
}

// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
__attribute__((weak)) HardwareDriverParams _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  HardwareDriverParams params = new struct DriverParamsBase {
    .pins = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency,
    .dead_zone = dead_zone,
    .initSuccess = true
  };
  return params;
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware speciffic
__attribute__((weak)) void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0f*dc_a);
  analogWrite(pinB, 255.0f*dc_b);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
__attribute__((weak)) void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0f*dc_a);
  analogWrite(pinB, 255.0f*dc_b);
  analogWrite(pinC, 255.0f*dc_c);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware speciffic
__attribute__((weak)) void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pin1A, 255.0f*dc_1a);
  analogWrite(pin1B, 255.0f*dc_1b);
  analogWrite(pin2A, 255.0f*dc_2a);
  analogWrite(pin2B, 255.0f*dc_2b);
}


// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
__attribute__((weak)) void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(dead_zone);
  _UNUSED(pinA_h);
  _UNUSED(pinB_h);
  _UNUSED(pinC_h);
  _UNUSED(pinA_l);
  _UNUSED(pinB_l);
  _UNUSED(pinC_l);
  return;
}
