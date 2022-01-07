#include "../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && !defined(SOC_MCPWM_SUPPORTED) 

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

// slot for the 3pwm bldc motors
typedef struct {
  int pinID;
  int ch1;
  int ch2;
  int ch3;
} bldc_3pwm_motor_slots_t;

// slot for the 2pwm stepper motors
typedef struct {
  int pinID;
  int ch1;
  int ch2;
} stepper_2pwm_motor_slots_t;

// slot for the 4pwm stepper motors
typedef struct {
  int pinID;
  int ch1;
  int ch2;
  int ch3;
  int ch4;
} stepper_4pwm_motor_slots_t;


// define motor slots array
bldc_3pwm_motor_slots_t esp32_bldc_3pwm_motor_slots[4] = {
  {_EMPTY_SLOT, 0,0,0}, // 1st motor
  {_EMPTY_SLOT, 0,0,0}, // 2nd motor
  {_EMPTY_SLOT, 0,0,0}, // 3st motor // esp32s2 & esp32
  {_EMPTY_SLOT, 0,0,0}, // 4nd motor // esp32 only
};
stepper_2pwm_motor_slots_t esp32_stepper_2pwm_motor_slots[4]={
  {_EMPTY_SLOT, 0,0}, // 1st motor
  {_EMPTY_SLOT, 0,0}, // 2nd motor
  {_EMPTY_SLOT, 0,0}, // 3rd motor
  {_EMPTY_SLOT, 0,0}  // 4th motor - esp32s2 and esp32
};
stepper_4pwm_motor_slots_t esp32_stepper_4pwm_motor_slots[4]={
  {_EMPTY_SLOT, 0,0,0,0}, // 1st motor
  {_EMPTY_SLOT, 0,0,0,0}, // 2nd motor - esp32s2 and esp32
  {_EMPTY_SLOT, 0,0,0,0}, // 3st motor - only esp32
  {_EMPTY_SLOT, 0,0,0,0}, // 4st motor - only esp32
};

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin, const int channel){
  ledcSetup(channel, freq, _PWM_RES_BIT );
  ledcAttachPin(pin, channel);
}

void _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 2 >= LEDC_CHANNELS ) return;

  stepper_2pwm_motor_slots_t m_slot = {};

  // determine which motor are we connecting
  // and set the appropriate configuration parameters
  for(int slot_num = 0; slot_num < 4; slot_num++){
    if(esp32_stepper_2pwm_motor_slots[slot_num].pinID == _EMPTY_SLOT){ // put the new motor in the first empty slot
      esp32_stepper_2pwm_motor_slots[slot_num].pinID = pinA;
      esp32_stepper_2pwm_motor_slots[slot_num].ch1 = channel_index++;
      esp32_stepper_2pwm_motor_slots[slot_num].ch2 = channel_index++;
      m_slot = esp32_stepper_2pwm_motor_slots[slot_num];
      break;
    }
  }

  _setHighFrequency(pwm_frequency, pinA, m_slot.ch1);
  _setHighFrequency(pwm_frequency, pinB, m_slot.ch2);
}

void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 3 >= LEDC_CHANNELS ) return;

  bldc_3pwm_motor_slots_t m_slot = {};

  // determine which motor are we connecting
  // and set the appropriate configuration parameters
  for(int slot_num = 0; slot_num < 4; slot_num++){
    if(esp32_bldc_3pwm_motor_slots[slot_num].pinID == _EMPTY_SLOT){ // put the new motor in the first empty slot
      esp32_bldc_3pwm_motor_slots[slot_num].pinID = pinA;
      esp32_bldc_3pwm_motor_slots[slot_num].ch1 = channel_index++;
      esp32_bldc_3pwm_motor_slots[slot_num].ch2 = channel_index++;
      esp32_bldc_3pwm_motor_slots[slot_num].ch3 = channel_index++;
      m_slot = esp32_bldc_3pwm_motor_slots[slot_num];
      break;
    }
  }

  _setHighFrequency(pwm_frequency, pinA, m_slot.ch1);
  _setHighFrequency(pwm_frequency, pinB, m_slot.ch2);
  _setHighFrequency(pwm_frequency, pinC, m_slot.ch3);
}

void _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  // check if enough channels available
  if ( channel_index + 4 >= LEDC_CHANNELS ) return;


  stepper_4pwm_motor_slots_t m_slot = {};

  // determine which motor are we connecting
  // and set the appropriate configuration parameters
  for(int slot_num = 0; slot_num < 4; slot_num++){
    if(esp32_stepper_4pwm_motor_slots[slot_num].pinID == _EMPTY_SLOT){ // put the new motor in the first empty slot
      esp32_stepper_4pwm_motor_slots[slot_num].pinID = pinA;
      esp32_stepper_4pwm_motor_slots[slot_num].ch1 = channel_index++;
      esp32_stepper_4pwm_motor_slots[slot_num].ch2 = channel_index++;
      esp32_stepper_4pwm_motor_slots[slot_num].ch3 = channel_index++;
      esp32_stepper_4pwm_motor_slots[slot_num].ch4 = channel_index++;
      m_slot = esp32_stepper_4pwm_motor_slots[slot_num];
      break;
    }
  }

  _setHighFrequency(pwm_frequency, pinA, m_slot.ch1);
  _setHighFrequency(pwm_frequency, pinB, m_slot.ch2);
  _setHighFrequency(pwm_frequency, pinC, m_slot.ch3);
  _setHighFrequency(pwm_frequency, pinD, m_slot.ch4);
}

void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
// determine which motor slot is the motor connected to
  for(int i = 0; i < 4; i++){
    if(esp32_stepper_2pwm_motor_slots[i].pinID == pinA){ // if motor slot found
      ledcWrite(esp32_stepper_2pwm_motor_slots[i].ch1, _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
      ledcWrite(esp32_stepper_2pwm_motor_slots[i].ch2, _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
      break;
    }
  }
}

void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // determine which motor slot is the motor connected to
  for(int i = 0; i < 4; i++){
    if(esp32_bldc_3pwm_motor_slots[i].pinID == pinA){ // if motor slot found
      ledcWrite(esp32_bldc_3pwm_motor_slots[i].ch1, _constrain(_PWM_RES*dc_a, 0, _PWM_RES));
      ledcWrite(esp32_bldc_3pwm_motor_slots[i].ch2, _constrain(_PWM_RES*dc_b, 0, _PWM_RES));
      ledcWrite(esp32_bldc_3pwm_motor_slots[i].ch3, _constrain(_PWM_RES*dc_c, 0, _PWM_RES));
      break;
    }
  }
}

void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // determine which motor slot is the motor connected to
  for(int i = 0; i < 4; i++){
    if(esp32_stepper_4pwm_motor_slots[i].pinID == pin1A){ // if motor slot found
      ledcWrite(esp32_stepper_4pwm_motor_slots[i].ch1, _constrain(_PWM_RES*dc_1a, 0, _PWM_RES));
      ledcWrite(esp32_stepper_4pwm_motor_slots[i].ch2, _constrain(_PWM_RES*dc_1b, 0, _PWM_RES));
      ledcWrite(esp32_stepper_4pwm_motor_slots[i].ch3, _constrain(_PWM_RES*dc_2a, 0, _PWM_RES));
      ledcWrite(esp32_stepper_4pwm_motor_slots[i].ch4, _constrain(_PWM_RES*dc_2b, 0, _PWM_RES));
      break;
    }
  }
}

#endif