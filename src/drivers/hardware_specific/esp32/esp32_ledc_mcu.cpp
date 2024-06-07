#include "../../hardware_api.h"

/*
For the moment the LEDC driver implements the simplest possible way to set the PWM on esp32 while enabling to set the frequency and resolution.

The pwm is not center aligned and moreover there are no guarantees on the proper alignement between the PWM signals.
Therefore this driver is not recommended for boards that have MCPWM.

There are however ways to improve the LEDC driver, by not using directly espressif sdk: 
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#ledc-api-change-pwm-signal
- We could potentially use the ledc_set_duty_with_hpoint function to set the duty cycle and the high time point to make the signals center-aligned
- We could force the use of the same ledc timer within one driver to ensure the alignement between the signals

*/

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && ( !defined(SOC_MCPWM_SUPPORTED) || defined(SIMPLEFOC_ESP32_USELEDC) )

#pragma message("")
#pragma message("SimpleFOC: compiling for ESP32 LEDC driver")
#pragma message("")

#include "driver/ledc.h"

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 38000 // 38khz max to be able to have 10 bit pwm resolution
#define _PWM_RES_BIT 10  // 10 bir resolution
#define _PWM_RES 1023 // 2^10-1 = 1024-1


// figure out how many ledc channels are available
// esp32   - 2x8=16
// esp32s2 - 8
// esp32c3 - 6
#include "soc/soc_caps.h"
#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif


// currently used ledc channels
// support for multiple motors
// esp32 has 16 channels
// esp32s2 has 8 channels
// esp32c3 has 6 channels
int channels_used = 0;


typedef struct ESP32LEDCDriverParams {
  int pins[6];
  long pwm_frequency;
} ESP32LEDCDriverParams;





//  configure High PWM frequency
bool _setHighFrequency(const long freq, const int pin){
  // sets up the pwm resolution and frequency on this pin
  // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
  // from v5.x no more need to deal with channels
  return ledcAttach(pin, freq, _PWM_RES_BIT);
}






void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channels_used + 1 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;
  channels_used++;

  // setup the channel
  if (!_setHighFrequency(pwm_frequency, pinA)) return SIMPLEFOC_DRIVER_INIT_FAILED;

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency
  };
  return params;
}








void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channels_used + 2 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;
  channels_used += 2;

  // setup the channels
  if(!_setHighFrequency(pwm_frequency, pinA) || !_setHighFrequency(pwm_frequency, pinB)) 
    return SIMPLEFOC_DRIVER_INIT_FAILED;

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channels_used + 3 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;
  channels_used += 3;

  // setup the channels
  if(!_setHighFrequency(pwm_frequency, pinA) || !_setHighFrequency(pwm_frequency, pinB) || !_setHighFrequency(pwm_frequency, pinC)) 
    return SIMPLEFOC_DRIVER_INIT_FAILED;

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channels_used + 4 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;
  channels_used += 4;

  // setup the channels
  if(!_setHighFrequency(pwm_frequency, pinA) ||  !_setHighFrequency(pwm_frequency, pinB) ||
      !_setHighFrequency(pwm_frequency, pinC)||  !_setHighFrequency(pwm_frequency, pinD)) 
    return SIMPLEFOC_DRIVER_INIT_FAILED;

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .pins = { pinA, pinB, pinC, pinD },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




void _writeDutyCycle1PWM(float dc_a, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
}



void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[1], _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[1], _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[2], _constrain(_PWM_RES*dc_c, 0, _PWM_RES));
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[0], _constrain(_PWM_RES*dc_1a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[1], _constrain(_PWM_RES*dc_1b, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[2], _constrain(_PWM_RES*dc_2a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->pins[3], _constrain(_PWM_RES*dc_2b, 0, _PWM_RES));
}

#endif