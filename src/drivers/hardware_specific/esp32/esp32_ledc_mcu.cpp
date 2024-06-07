#include "../../hardware_api.h"

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

#define LEDC_CHANNELS_GROUP0   (LEDC_CHANNELS < 8 ? LEDC_CHANNELS : 8)
#define LEDC_CHANNELS_GROUP1   (LEDC_CHANNELS < 8 ? 0 : LEDC_CHANNELS - 8)


// currently used ledc channels
// support for multiple motors
// esp32 has 16 channels
// esp32s2 has 8 channels
// esp32c3 has 6 channels
// channels from 0-7 are in group 0 and 8-15 in group 1 
// - only esp32 as of mid 2024 has the second group, all the s versions don't
int group_channels_used[2] = {0};


typedef struct ESP32LEDCDriverParams {
  ledc_channel_t channels[6];
  ledc_mode_t groups[6];
  long pwm_frequency;
  float dead_zone;
} ESP32LEDCDriverParams;


/*
  Function to attach a channel to a pin with advanced settings
  - freq - pwm frequency
  - resolution - pwm resolution
  - channel - ledc channel
  - inverted - output inverted
  - group - ledc group

  This function is a workaround for the ledcAttachPin function that is not available in the ESP32 Arduino core, in which the 
  PWM signals are synchronized in pairs, while the simplefoc requires a bit more flexible configuration.
  This function sets also allows configuring a channel as inverted, which is not possible with the ledcAttachPin function.

  Function returns true if the channel was successfully attached, false otherwise.
*/
bool _ledcAttachChannelAdvanced(uint8_t pin, int _channel, int _group, uint32_t freq, uint8_t resolution,  bool inverted) {


  ledc_channel_t channel = static_cast<ledc_channel_t>(_channel);
  ledc_mode_t group = static_cast<ledc_mode_t>(_group);

  ledc_timer_bit_t res = static_cast<ledc_timer_bit_t>(resolution);
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode = group;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.duty_resolution = res;
  ledc_timer.freq_hz = freq;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&ledc_timer) != ESP_OK) {
    return false;
  }

  // if active high is false invert
  int pin_high_level = SIMPLEFOC_PWM_ACTIVE_HIGH ? 1 : 0;
  if (inverted) pin_high_level = !pin_high_level;

  uint32_t duty = ledc_get_duty(group, channel);
  ledc_channel_config_t ledc_channel;
  ledc_channel.speed_mode = group;
  ledc_channel.channel =  channel;
  ledc_channel.timer_sel = LEDC_TIMER_0; 
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = pin;
  ledc_channel.duty = duty;
  ledc_channel.hpoint = 0;
  ledc_channel.flags.output_invert = pin_high_level;
  if (ledc_channel_config(&ledc_channel)!= ESP_OK) {
    return false;
  }

  return true;
}

// returns the number of the group that has enough channels available
// returns -1 if no group has enough channels
// 
// NOT IMPLEMENTED BUT COULD BE USEFUL
// returns 2 if no group has enough channels but combined they do 
int _findGroupWithChannelsAvailable(int no_channels){
  if (group_channels_used[0] + no_channels < LEDC_CHANNELS_GROUP0){
    return 0;
  }else if (group_channels_used[1] + no_channels < LEDC_CHANNELS_GROUP1){
    return 1;
  }
  // else if (group_channels_used[0] + group_channels_used[1] + no_channels < LEDC_CHANNELS){
  //   return 2;
  // }
  return -1;
}


void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  SIMPLEFOC_DEBUG("EP32-DRV: Configuring 1PWM");
  // check if enough channels available
  int group = _findGroupWithChannelsAvailable(1);
  if (group < 0){
    SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Not enough channels available!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
 SIMPLEFOC_DEBUG("EP32-DRV: 1PWM setup in group: ", (group));

  // configure the channel
  group_channels_used[group] += 1;
  if(!_ledcAttachChannelAdvanced(pinA, group_channels_used[group], group, pwm_frequency, _PWM_RES_BIT, false)) {
    SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Failed to attach channel:", group_channels_used[group]);
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
    
  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = {  static_cast<ledc_channel_t>(group_channels_used[group]) },
    .groups = { (ledc_mode_t)group },
    .pwm_frequency = pwm_frequency
  };
  SIMPLEFOC_DEBUG("EP32-DRV: 1PWM setup successful in group: ", (group));
  return params;
}








void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  SIMPLEFOC_DEBUG("EP32-DRV: Configuring 2PWM");

  // check if enough channels available
  int group = _findGroupWithChannelsAvailable(2);
  if (group < 0) {
   SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Not enough channels available!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 2PWM setup in group: ", (group));

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = {  static_cast<ledc_channel_t>(0)},
    .groups = { (ledc_mode_t)0 },
    .pwm_frequency = pwm_frequency
  };

  int pins[2] = {pinA, pinB};
  for(int i = 0; i < 2; i++){
    group_channels_used[group]++;
    if(!_ledcAttachChannelAdvanced(pins[i], group_channels_used[group], group, pwm_frequency, _PWM_RES_BIT, false)){
      SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Failed to attach channel:", group_channels_used[group]);
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
    params->channels[i] = static_cast<ledc_channel_t>(group_channels_used[group]);
    params->groups[i] = (ledc_mode_t)group;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 2PWM setup successful in group: ", (group));
  return params;
}



void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  SIMPLEFOC_DEBUG("EP32-DRV: Configuring 3PWM");

  // check if enough channels available
  int group = _findGroupWithChannelsAvailable(3);
  if (group < 0) {
    SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Not enough channels available!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 3PWM setup in group: ", (group));

  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = {  static_cast<ledc_channel_t>(0)},
    .groups = { (ledc_mode_t)0 },
    .pwm_frequency = pwm_frequency
  };

  int pins[3] = {pinA, pinB, pinC};
  for(int i = 0; i < 3; i++){
    group_channels_used[group]++;
    if(!_ledcAttachChannelAdvanced(pins[i], group_channels_used[group], group, pwm_frequency, _PWM_RES_BIT, false)){
      SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Failed to attach channel:", group_channels_used[group]);
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
    params->channels[i] = static_cast<ledc_channel_t>(group_channels_used[group]);
    params->groups[i] = (ledc_mode_t)group;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 3PWM setup successful in group: ", (group));
  return params;
}



void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  SIMPLEFOC_DEBUG("EP32-DRV: Configuring 4PWM");
  // check if enough channels available
  int group = _findGroupWithChannelsAvailable(4);
  if (group < 0){
    SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Not enough channels available!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 4PWM setup in group: ", (group));
  ESP32LEDCDriverParams* params = new ESP32LEDCDriverParams {
    .channels = {  static_cast<ledc_channel_t>(0)},
    .groups = { (ledc_mode_t)0 },
    .pwm_frequency = pwm_frequency
  };

  int pins[4] = {pinA, pinB, pinC, pinD};
  for(int i = 0; i < 4; i++){
    group_channels_used[group]++;
    if(!_ledcAttachChannelAdvanced(pins[i], group_channels_used[group], group, pwm_frequency, _PWM_RES_BIT, false)){
      SIMPLEFOC_DEBUG("EP32-DRV: ERROR - Failed to attach channel:", group_channels_used[group]);
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
    params->channels[i] = static_cast<ledc_channel_t>(group_channels_used[group]);
    params->groups[i] = (ledc_mode_t)group;
  }
  SIMPLEFOC_DEBUG("EP32-DRV: 4PWM setup successful in group: ", (group));
  return params;
}


void _writeDutyCycle(float dc, void* params, int index){
  ledc_set_duty_with_hpoint(((ESP32LEDCDriverParams*)params)->groups[index],((ESP32LEDCDriverParams*)params)->channels[index], _PWM_RES*dc, _PWM_RES/2.0*(1.0-dc));
  ledc_update_duty(((ESP32LEDCDriverParams*)params)->groups[index],((ESP32LEDCDriverParams*)params)->channels[index]);
}

void _writeDutyCycle1PWM(float dc_a, void* params){
  _writeDutyCycle(dc_a, params, 0);
}



void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  _writeDutyCycle(dc_a, params, 0);
  _writeDutyCycle(dc_b, params, 1);
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  _writeDutyCycle(dc_a, params, 0);
  _writeDutyCycle(dc_b, params, 1);
  _writeDutyCycle(dc_c, params, 2);
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  _writeDutyCycle(dc_1a, params, 0);
  _writeDutyCycle(dc_1b, params, 1);
  _writeDutyCycle(dc_2a, params, 2);
  _writeDutyCycle(dc_2b, params, 3);
}


// TODO - implement 6PWM
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  SIMPLEFOC_DEBUG("EP32-DRV:  6PWM not supported!");
  return SIMPLEFOC_DRIVER_INIT_FAILED;
}
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, void* params) {
  SIMPLEFOC_DEBUG("EP32-DRV: 6PWM not supported");
}

#endif