#include "esp32_driver_mcpwm.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)
  
#pragma message("")
#pragma message("SimpleFOC: compiling for ESP32 MCPWM driver")
#pragma message("")

// function setting the high pwm frequency to the supplied pins
// - DC motor  - 1PWM setting
// - hardware specific
void* _configure1PWM(long pwm_frequency, const int pinA) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25hz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 40kHz max

  int group, timer;
  if(!_findBestGroup(1, pwm_frequency, &group, &timer)) {
   SIMPLEFOC_ESP32_DRV_DEBUG("Not enough pins available for 1PWM!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 1PWM in group: "+String(group)+" on timer: "+String(timer));
  // configure the timer
  int pins[1] = {pinA};
  return _configurePinsMCPWM(pwm_frequency, group, timer, 1, pins);
}

  
// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware specific
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25hz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 40kHz max

  int group, timer;
  int ret = _findBestGroup(2, pwm_frequency, &group, &timer);
  if(!ret) {
    SIMPLEFOC_ESP32_DRV_DEBUG("Not enough pins available for 2PWM!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if(ret == 1){
    // configure the 2pwm on only one group
    SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 2PWM in group: "+String(group)+" on timer: "+String(timer));
    // configure the timer
    int pins[2] = {pinA,  pinB};
    return _configurePinsMCPWM(pwm_frequency, group, timer, 2, pins);
  }else{
    SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 2PWM as two 1PWM drivers");
    ESP32MCPWMDriverParams* params[2];

    // the code is a bit huge for what it does
    // it just instantiates two 2PMW drivers and combines the returned params
    int pins[2][1] = {{pinA},  {pinB}};
    for(int i =0; i<2; i++){
      int timer = _findLastTimer(i); //find last created timer in group i
      SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 1PWM in group: "+String(i)+" on timer: "+String(timer));
      void* p = _configurePinsMCPWM(pwm_frequency, i, timer, 1, pins[i]);
      if(p == SIMPLEFOC_DRIVER_INIT_FAILED){
         SIMPLEFOC_ESP32_DRV_DEBUG("Error configuring  1PWM");
          return SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{
        params[i] = (ESP32MCPWMDriverParams*)p;
      }
    }
    // combine the driver parameters
    ESP32MCPWMDriverParams* ret_params = new ESP32MCPWMDriverParams{
      .pwm_frequency = params[0]->pwm_frequency,
      .group_id = 2, // both groups
    };
    for(int i =0; i<2; i++){
      ret_params->timers[i] = params[i]->timers[0];
      ret_params->oper[i] = params[i]->oper[0];
      ret_params->comparator[i] = params[i]->comparator[0];
      ret_params->generator[i] = params[i]->generator[0];
    }
    ret_params->mcpwm_period = params[0]->mcpwm_period;
    return ret_params;
  }
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware specific
void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25hz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 40kHz max

  int group, timer;
  if(!_findBestGroup(3, pwm_frequency, &group, &timer)) {
    SIMPLEFOC_ESP32_DRV_DEBUG("Not enough pins available for 3PWM!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 3PWM in group: "+String(group)+" on timer: "+String(timer));
  // configure the timer
  int pins[3] = {pinA,  pinB, pinC};
  return _configurePinsMCPWM(pwm_frequency, group, timer, 3, pins);
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware specific
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD){
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25hz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 40kHz max

  int group, timer;
  int ret = _findBestGroup(4, pwm_frequency, &group, &timer);
  if(!ret) {
    SIMPLEFOC_ESP32_DRV_DEBUG("Not enough pins available for 4PWM!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if(ret == 1){
    SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 4PWM in group: "+String(group)+" on timer: "+String(timer));
    // configure the timer
    int pins[4] = {pinA,  pinB, pinC, pinD};
    return _configurePinsMCPWM(pwm_frequency, group, timer, 4, pins);
  }else{
    SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 4PWM as two 2PWM drivers");
    ESP32MCPWMDriverParams* params[2];

    // the code is a bit huge for what it does
    // it just instantiates two 2PMW drivers and combines the returned params
    int pins[2][2] = {{pinA,  pinB},{pinC, pinD}};
    for(int i =0; i<2; i++){
      int timer = _findNextTimer(i); //find next available timer in group i
      SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 2PWM in group: "+String(i)+" on timer: "+String(timer));
      void* p = _configurePinsMCPWM(pwm_frequency, i, timer, 2, pins[i]);
      if(p == SIMPLEFOC_DRIVER_INIT_FAILED){
        SIMPLEFOC_ESP32_DRV_DEBUG("Error configuring  2PWM");
         return SIMPLEFOC_DRIVER_INIT_FAILED;
      }else{
        params[i] = (ESP32MCPWMDriverParams*)p;
      }
    }
    // combine the driver parameters
    ESP32MCPWMDriverParams* ret_params = new ESP32MCPWMDriverParams{
      .pwm_frequency = params[0]->pwm_frequency,
      .group_id = 2, // both groups
      .timers = {params[0]->timers[0], params[1]->timers[0]},
      .oper = {params[0]->oper[0], params[1]->oper[0]}
    };
    for(int i =0; i<4; i++){
      ret_params->comparator[i] = params[(int)floor(i/2)]->comparator[i%2];
      ret_params->generator[i] = params[(int)floor(i/2)]->generator[i%2];
    }
    ret_params->mcpwm_period = params[0]->mcpwm_period;
    return ret_params;
  }
}


void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25hz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 40kHz max

  int group, timer;
  if(!_findBestGroup(6, pwm_frequency, &group, &timer)) {
    SIMPLEFOC_ESP32_DRV_DEBUG("Not enough pins available for 6PWM!");
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 6PWM in group: "+String(group)+" on timer: "+String(timer));
  // configure the timer
  int pins[6] = {pinA_h,pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l};
  return _configure6PWMPinsMCPWM(pwm_frequency, group, timer, dead_zone, pins);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_a);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[1], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_b);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[2], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_c);
}

// function setting the pwm duty cycle to the hardware
// - DCMotor -1PWM setting
// - hardware specific
void _writeDutyCycle1PWM(float dc_a, void* params){
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_a);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware specific
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_a);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[1], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_b);
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware specific
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // se the PWM on the slot timers
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_1a);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[1], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_1b);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[2], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_2a);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[3], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_2b);
}

void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
#if SIMPLEFOC_ESP32_HW_DEADTIME == true
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_a);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[1], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_b);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[2], ((ESP32MCPWMDriverParams*)params)->mcpwm_period, dc_c);

  // set the phase state
  _forcePhaseState(((ESP32MCPWMDriverParams*)params)->generator[0], ((ESP32MCPWMDriverParams*)params)->generator[1], phase_state[0]);
  _forcePhaseState(((ESP32MCPWMDriverParams*)params)->generator[2], ((ESP32MCPWMDriverParams*)params)->generator[3], phase_state[1]);
  _forcePhaseState(((ESP32MCPWMDriverParams*)params)->generator[4], ((ESP32MCPWMDriverParams*)params)->generator[5], phase_state[2]);
#else
  uint32_t period = ((ESP32MCPWMDriverParams*)params)->mcpwm_period;
  float dead_zone = (float)((ESP32MCPWMDriverParams*)params)->dead_zone /2.0f;
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[0], period,  (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_HI) ? dc_a-dead_zone : 0.0f);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[1], period,  (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_LO) ? dc_a+dead_zone : 1.0f);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[2], period,  (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_HI) ? dc_b-dead_zone : 0.0f);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[3], period,  (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_LO) ? dc_b+dead_zone : 1.0f);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[4], period,  (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_HI) ? dc_c-dead_zone : 0.0f);
  _setDutyCycle(((ESP32MCPWMDriverParams*)params)->comparator[5], period,  (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_LO) ? dc_c+dead_zone : 1.0f);
#endif
}
#endif
