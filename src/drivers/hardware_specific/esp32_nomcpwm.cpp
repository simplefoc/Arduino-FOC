#include "../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && !defined(SOC_MCPWM_SUPPORTED)

#include "driver/ledc.h"

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 38000 // 38khz max to be able to have 10 bit pwm resolution
#define _PWM_RES_BIT 10  // 10 bir resolution
#define _PWM_RES 1023 // 2^10-1 = 1023-1


//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin, const int channel){
  ledcSetup(channel, freq, _PWM_RES_BIT );
  ledcAttachPin(pin, channel);
}

void _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA, LEDC_CHANNEL_0);
  _setHighFrequency(pwm_frequency, pinB, LEDC_CHANNEL_1);
}

void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA, LEDC_CHANNEL_0);
  _setHighFrequency(pwm_frequency, pinB, LEDC_CHANNEL_1);
  _setHighFrequency(pwm_frequency, pinC, LEDC_CHANNEL_2);
}

void _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  _setHighFrequency(pwm_frequency, pinA, LEDC_CHANNEL_0);
  _setHighFrequency(pwm_frequency, pinB, LEDC_CHANNEL_1);
  _setHighFrequency(pwm_frequency, pinC, LEDC_CHANNEL_2);
  _setHighFrequency(pwm_frequency, pinD, LEDC_CHANNEL_3);
}

void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
  uint32_t dutyA = _constrain(_PWM_RES*dc_a, 0, _PWM_RES);
  uint32_t dutyB = _constrain(_PWM_RES*dc_b, 0, _PWM_RES);
  ledcWrite(LEDC_CHANNEL_0, dutyA);
  ledcWrite(LEDC_CHANNEL_1, dutyB);
}

void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  uint32_t dutyA = _constrain(_PWM_RES*dc_a, 0, _PWM_RES);
  uint32_t dutyB = _constrain(_PWM_RES*dc_b, 0, _PWM_RES);
  uint32_t dutyC = _constrain(_PWM_RES*dc_c, 0, _PWM_RES);
  ledcWrite(LEDC_CHANNEL_0, dutyA);
  ledcWrite(LEDC_CHANNEL_1, dutyB);
  ledcWrite(LEDC_CHANNEL_2, dutyC);
}

void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  uint32_t duty1A = _constrain(_PWM_RES*dc_1a, 0, _PWM_RES);
  uint32_t duty1B = _constrain(_PWM_RES*dc_1b, 0, _PWM_RES);
  uint32_t duty2A = _constrain(_PWM_RES*dc_2a, 0, _PWM_RES);
  uint32_t duty2B = _constrain(_PWM_RES*dc_2b, 0, _PWM_RES);
  ledcWrite(LEDC_CHANNEL_0, duty1A);
  ledcWrite(LEDC_CHANNEL_1, duty1B);
  ledcWrite(LEDC_CHANNEL_2, duty2A);
  ledcWrite(LEDC_CHANNEL_3, duty2B);
}

#endif