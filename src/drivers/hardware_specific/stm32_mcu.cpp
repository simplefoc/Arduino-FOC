
#include "../hardware_api.h"
#include "stm32_mcu.h"

#if defined(_STM32_DEF_)


// setting pwm to hardware pin - instead analogWrite()
void _setPwm(HardwareTimer *HT, uint32_t channel, uint32_t value, int resolution)
{
  // PinName pin = digitalPinToPinName(ulPin);
  // TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  // uint32_t index = get_timer_index(Instance);
  // HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  //uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}


// init pin pwm
HardwareTimer* _initPinPWM(uint32_t PWM_freq, int ulPin)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  if (Instance == NP) {
    Serial.print("No timer on pin ");
    Serial.println(ulPin);
    delay(1000);
    return NP;
  }

  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->pause();
  HT->refresh();
  return HT;
}


// init high side pin
HardwareTimer* _initPinPWMHigh(uint32_t PWM_freq, int ulPin)
{
  return _initPinPWM(PWM_freq, ulPin);
}

// init low side pin
HardwareTimer* _initPinPWMLow(uint32_t PWM_freq, int ulPin)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);

  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    TIM_OC_InitTypeDef sConfigOC = TIM_OC_InitTypeDef();
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
#if defined(TIM_OCIDLESTATE_SET)
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_SET;
#endif
#if defined(TIM_OCNIDLESTATE_RESET)
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
#endif
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    HAL_TIM_PWM_ConfigChannel(&(HardwareTimer_Handle[index]->handle), &sConfigOC, channel);
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, pin);
  HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->pause();
  HT->refresh();
  return HT;
}


// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1,HardwareTimer *HT2,HardwareTimer *HT3)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
}

// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1,HardwareTimer *HT2,HardwareTimer *HT3,HardwareTimer *HT4)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT4->pause();
  HT4->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
  HT4->resume();
}

// configure hardware 6pwm interface only one timer with inverted channels
STM32DriverParams* _initHardware6PWMInterface(long PWM_freq, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l)
{
  
#if !defined(STM32L0xx) // L0 boards dont have hardware 6pwm interface 
  PinName uhPinName = digitalPinToPinName(pinA_h);
  PinName ulPinName = digitalPinToPinName(pinA_l);
  PinName vhPinName = digitalPinToPinName(pinB_h);
  PinName vlPinName = digitalPinToPinName(pinB_l);
  PinName whPinName = digitalPinToPinName(pinC_h);
  PinName wlPinName = digitalPinToPinName(pinC_l);
  uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(uhPinName, PinMap_PWM));
  uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(ulPinName, PinMap_PWM));
  uint32_t channel3 = STM_PIN_CHANNEL(pinmap_function(vhPinName, PinMap_PWM));
  uint32_t channel4 = STM_PIN_CHANNEL(pinmap_function(vlPinName, PinMap_PWM));
  uint32_t channel5 = STM_PIN_CHANNEL(pinmap_function(whPinName, PinMap_PWM));
  uint32_t channel6 = STM_PIN_CHANNEL(pinmap_function(wlPinName, PinMap_PWM));

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM);

  uint32_t index = get_timer_index(Instance);

  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow((uint32_t)PWM_freq, HERTZ_FORMAT);
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  HT->setMode(channel1, TIMER_OUTPUT_COMPARE_PWM1, uhPinName);
  HT->setMode(channel2, TIMER_OUTPUT_COMPARE_PWM1, ulPinName);
  HT->setMode(channel3, TIMER_OUTPUT_COMPARE_PWM1, vhPinName);
  HT->setMode(channel4, TIMER_OUTPUT_COMPARE_PWM1, vlPinName);
  HT->setMode(channel5, TIMER_OUTPUT_COMPARE_PWM1, whPinName);
  HT->setMode(channel6, TIMER_OUTPUT_COMPARE_PWM1, wlPinName);

  // dead time is set in nanoseconds
  uint32_t dead_time_ns = (float)(1e9f/PWM_freq)*dead_zone;
  uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(HT->getHandle()->Instance), dead_time_ns);
  LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, dead_time); // deadtime is non linear!
  LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

  HT->pause();
  HT->refresh();

  // maybe should be removed I am not sure if its necessary for BG431
  // adjust the initial timer state such that the trigger for DMA transfer aligns with the pwm peaks instead of throughs.
  HT->getHandle()->Instance->CR1 |= TIM_CR1_DIR;
  HT->getHandle()->Instance->CNT = TIM1->ARR;

  HT->resume();

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT },
    .channels = { channel1, channel3, channel5 },
    .pwm_frequency = PWM_freq,
    .dead_zone = dead_zone,
    .interface_type = _HARDWARE_6PWM
  };
    
  return params;
#else 
  return SIMPLEFOC_DRIVER_INIT_FAILED; // return nothing
#endif
}


// returns 0 if each pair of pwm channels has same channel
// returns 1 all the channels belong to the same timer - hardware inverted channels
// returns -1 if neither - avoid configuring - error!!!
int _interfaceType(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  PinName nameAH = digitalPinToPinName(pinA_h);
  PinName nameAL = digitalPinToPinName(pinA_l);
  PinName nameBH = digitalPinToPinName(pinB_h);
  PinName nameBL = digitalPinToPinName(pinB_l);
  PinName nameCH = digitalPinToPinName(pinC_h);
  PinName nameCL = digitalPinToPinName(pinC_l);
  int tim1 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAH, PinMap_PWM));
  int tim2 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAL, PinMap_PWM));
  int tim3 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBH, PinMap_PWM));
  int tim4 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBL, PinMap_PWM));
  int tim5 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCH, PinMap_PWM));
  int tim6 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCL, PinMap_PWM));

#if defined(STM32L0xx) // L0 boards dont have hardware 6pwm interface 

  if(tim1 == tim2 && tim3==tim4  && tim5==tim6)
    return _SOFTWARE_6PWM; // software 6pwm interface - each pair of high-low side same timer
  else
    return _ERROR_6PWM; // might be error avoid configuration
#else // the rest of stm32 boards

  if(tim1 == tim2 && tim2==tim3 && tim3==tim4  && tim4==tim5 && tim5==tim6)
    return _HARDWARE_6PWM; // hardware 6pwm interface - only on timer
  else if(tim1 == tim2 && tim3==tim4  && tim5==tim6)
    return _SOFTWARE_6PWM; // software 6pwm interface - each pair of high-low side same timer
  else
    return _ERROR_6PWM; // might be error avoid configuration
#endif
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT2);
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2 },
    .channels = { channel1, channel2 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinC);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT3);
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
  uint32_t channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2, HT3 },
    .channels = { channel1, channel2, channel3 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinC);
  HardwareTimer* HT4 = _initPinPWM(pwm_frequency, pinD);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT3, HT4);

  uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
  uint32_t channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));
  uint32_t channel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinD), PinMap_PWM));

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2, HT3, HT4 },
    .channels = { channel1, channel2, channel3, channel4 },
    .pwm_frequency = pwm_frequency
  };
  return params;
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
//- hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_c, _PWM_RESOLUTION);
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
//- hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_1a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_1b, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_2a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[3], ((STM32DriverParams*)params)->channels[3], _PWM_RANGE*dc_2b, _PWM_RESOLUTION);
}




// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to |%0kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  // find configuration
  int config = _interfaceType(pinA_h, pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l);
  STM32DriverParams* params;
  // configure accordingly
  switch(config){
    case _ERROR_6PWM:
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    case _HARDWARE_6PWM:
      params = _initHardware6PWMInterface(pwm_frequency, dead_zone, pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
      break;
    case _SOFTWARE_6PWM:
      HardwareTimer* HT1 = _initPinPWMHigh(pwm_frequency, pinA_h);
      _initPinPWMLow(pwm_frequency, pinA_l);
      HardwareTimer* HT2 = _initPinPWMHigh(pwm_frequency, pinB_h);
      _initPinPWMLow(pwm_frequency, pinB_l);
      HardwareTimer* HT3 = _initPinPWMHigh(pwm_frequency, pinC_h);
      _initPinPWMLow(pwm_frequency, pinC_l);
      _alignPWMTimers(HT1, HT2, HT3);
      uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA_h), PinMap_PWM));
      uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA_l), PinMap_PWM));
      uint32_t channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB_h), PinMap_PWM));
      uint32_t channel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB_l), PinMap_PWM));
      uint32_t channel5 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC_h), PinMap_PWM));
      uint32_t channel6 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC_l), PinMap_PWM));
      params = new STM32DriverParams {
        .timers = { HT1, HT2, HT3 },
        .channels = { channel1, channel2, channel3, channel4, channel5, channel6 },
        .pwm_frequency = pwm_frequency,
        .dead_zone = dead_zone,
        .interface_type = _SOFTWARE_6PWM
      };
      break;
  }
  return params; // success
}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, void* params){
  switch(((STM32DriverParams*)params)->interface_type){
    case _HARDWARE_6PWM:
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_c, _PWM_RESOLUTION);
      break;
    case _SOFTWARE_6PWM:
      float dead_zone = ((STM32DriverParams*)params)->dead_zone;
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _constrain(dc_a + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[1], _constrain(dc_a - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[2], _constrain(dc_b + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[3], _constrain(dc_b - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[4], _constrain(dc_c + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[5], _constrain(dc_c - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      break;
  }
}
#endif