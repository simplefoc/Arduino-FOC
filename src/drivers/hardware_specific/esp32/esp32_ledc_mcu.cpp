#include "../../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && ( !defined(SOC_MCPWM_SUPPORTED) || defined(SIMPLEFOC_ESP32_USELEDC) )

#include "driver/ledc.h"

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 38000 // 38khz max to be able to have 10 bit pwm resolution
#define _PWM_RES_BIT 10  // 10 bir resolution
#define _PWM_RES 1023 // 2^10-1 = 1023-1


// empty motor slot
#define _EMPTY_SLOT -20
#define _TAKEN_SLOT -21

// figure out how many ledc channels are avaible
// esp32   - 2x8=16
// esp32s2 - 8
// esp32c3 - 6
#include "soc/soc_caps.h"
#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif


// current channel stack index
// support for multiple motors
// esp32 has 16 channels
// esp32s2 has 8 channels
// esp32c3 has 6 channels
int channel_index = 0;




typedef struct ESP32LEDCDriverParams {
  int channels[6];
  long pwm_frequency;
} ESP32LEDCDriverParams;





//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin, const int channel){
  ledcSetup(channel, freq, _PWM_RES_BIT );
  ledcAttachPin(pin, channel);
}






void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 1 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;

  int ch1 = channel_index++;
  _setHighFrequency(pwm_frequency, pinA, ch1);

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = { ch1 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}








void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 2 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;

  int ch1 = channel_index++;
  int ch2 = channel_index++;
  _setHighFrequency(pwm_frequency, pinA, ch1);
  _setHighFrequency(pwm_frequency, pinB, ch2);

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = { ch1, ch2 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 3 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;

  int ch1 = channel_index++;
  int ch2 = channel_index++;
  int ch3 = channel_index++;
  _setHighFrequency(pwm_frequency, pinA, ch1);
  _setHighFrequency(pwm_frequency, pinB, ch2);
  _setHighFrequency(pwm_frequency, pinC, ch3);

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = { ch1, ch2, ch3 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 4 >= LEDC_CHANNELS ) return SIMPLEFOC_DRIVER_INIT_FAILED;

  int ch1 = channel_index++;
  int ch2 = channel_index++;
  int ch3 = channel_index++;
  int ch4 = channel_index++;
  _setHighFrequency(pwm_frequency, pinA, ch1);
  _setHighFrequency(pwm_frequency, pinB, ch2);
  _setHighFrequency(pwm_frequency, pinC, ch3);
  _setHighFrequency(pwm_frequency, pinD, ch4);

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = { ch1, ch2, ch3, ch4 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




void _writeDutyCycle1PWM(float dc_a, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
}



void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[1], _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[0], _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[1], _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[2], _constrain(_PWM_RES*dc_c, 0, _PWM_RES));
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[0], _constrain(_PWM_RES*dc_1a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[1], _constrain(_PWM_RES*dc_1b, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[2], _constrain(_PWM_RES*dc_2a, 0, _PWM_RES));
  ledcWrite(((ESP32LEDCDriverParams*)params)->channels[3], _constrain(_PWM_RES*dc_2b, 0, _PWM_RES));
}

#endif