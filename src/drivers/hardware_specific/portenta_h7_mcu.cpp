
#include "../hardware_api.h"

#if defined(TARGET_PORTENTA_H7)


#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino/Portenta_H7")
#pragma message("")


#include "pwmout_api.h"
#include "pinDefinitions.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_power_mgmt.h"
#include "platform/mbed_assert.h"
#include "PeripheralPins.h"
#include "pwmout_device.h"

// default pwm parameters
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz


// // 6pwm parameters
// #define _HARDWARE_6PWM 1
// #define _SOFTWARE_6PWM 0
// #define _ERROR_6PWM -1



typedef struct PortentaDriverParams {
  pwmout_t pins[4];
  long pwm_frequency;
//  float dead_zone;
} PortentaDriverParams;



/* Convert STM32 Cube HAL channel to LL channel */
uint32_t _TIM_ChannelConvert_HAL2LL(uint32_t channel, pwmout_t *obj)
{
#if !defined(PWMOUT_INVERTED_NOT_SUPPORTED)
    if (obj->inverted) {
        switch (channel) {
            case TIM_CHANNEL_1  :
                return LL_TIM_CHANNEL_CH1N;
            case TIM_CHANNEL_2  :
                return LL_TIM_CHANNEL_CH2N;
            case TIM_CHANNEL_3  :
                return LL_TIM_CHANNEL_CH3N;
#if defined(LL_TIM_CHANNEL_CH4N)
            case TIM_CHANNEL_4  :
                return LL_TIM_CHANNEL_CH4N;
#endif
            default : /* Optional */
                return 0;
        }
    } else
#endif
    {
        switch (channel) {
            case TIM_CHANNEL_1  :
                return LL_TIM_CHANNEL_CH1;
            case TIM_CHANNEL_2  :
                return LL_TIM_CHANNEL_CH2;
            case TIM_CHANNEL_3  :
                return LL_TIM_CHANNEL_CH3;
            case TIM_CHANNEL_4  :
                return LL_TIM_CHANNEL_CH4;
            default : /* Optional */
                return 0;
        }
    }
}



// int _pwm_init(pwmout_t *obj, uint32_t pin, long frequency){
//   return _pwm_init(obj, digitalPinToPinName(pin), frequency);
// }

int _pwm_init(pwmout_t *obj, uint32_t pin, long frequency){
    int peripheral = (int)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
    int function = (int)pinmap_find_function(digitalPinToPinName(pin), PinMap_PWM);

    const PinMap static_pinmap = {digitalPinToPinName(pin), peripheral, function};

    pwmout_init_direct(obj, &static_pinmap);

    TIM_HandleTypeDef TimHandle;
    TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    uint32_t PclkFreq = 0;
    uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
    uint8_t i = 0;


    __HAL_TIM_DISABLE(&TimHandle);

    // Get clock configuration
    // Note: PclkFreq contains here the Latency (not used after)
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

    /*  Parse the pwm / apb mapping table to find the right entry */
    while (pwm_apb_map_table[i].pwm != obj->pwm) i++;
    // sanity check
    if (pwm_apb_map_table[i].pwm == 0) return -1;
    

    if (pwm_apb_map_table[i].pwmoutApb == PWMOUT_ON_APB1) {
        PclkFreq = HAL_RCC_GetPCLK1Freq();
        APBxCLKDivider = RCC_ClkInitStruct.APB1CLKDivider;
    } else {
#if !defined(PWMOUT_APB2_NOT_SUPPORTED)
        PclkFreq = HAL_RCC_GetPCLK2Freq();
        APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;
#endif
    }

    long period_us = 500000.0/((float)frequency);
    /* By default use, 1us as SW pre-scaler */
    obj->prescaler = 1;
    // TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
    if (APBxCLKDivider == RCC_HCLK_DIV1) {
        TimHandle.Init.Prescaler = (((PclkFreq) / 1000000)) - 1; // 1 us tick
    } else {
        TimHandle.Init.Prescaler = (((PclkFreq * 2) / 1000000)) - 1; // 1 us tick
    }
    TimHandle.Init.Period = (period_us - 1);

    /*  In case period or pre-scalers are out of range, loop-in to get valid values */
    while ((TimHandle.Init.Period > 0xFFFF) || (TimHandle.Init.Prescaler > 0xFFFF)) {
        obj->prescaler = obj->prescaler * 2;
        if (APBxCLKDivider == RCC_HCLK_DIV1) {
            TimHandle.Init.Prescaler = (((PclkFreq) / 1000000) * obj->prescaler) - 1;
        } else {
            TimHandle.Init.Prescaler = (((PclkFreq * 2) / 1000000) * obj->prescaler) - 1;
        }
        TimHandle.Init.Period = (period_us - 1) / obj->prescaler;
        /*  Period decreases and prescaler increases over loops, so check for
         *  possible out of range cases */
        if ((TimHandle.Init.Period < 0xFFFF) && (TimHandle.Init.Prescaler > 0xFFFF)) {
            break;
        }
    }

    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3; // center aligned

    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        return -1;
    }
    
    TIM_OC_InitTypeDef sConfig;
    // Configure channels
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.Pulse        = obj->pulse / obj->prescaler;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
#if defined(TIM_OCIDLESTATE_RESET)
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
#endif
#if defined(TIM_OCNIDLESTATE_RESET)
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
#endif

    int channel = 0;
    switch (obj->channel) {
        case 1:
            channel = TIM_CHANNEL_1;
            break;
        case 2:
            channel = TIM_CHANNEL_2;
            break;
        case 3:
            channel = TIM_CHANNEL_3;
            break;
        case 4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return -1;
    }
    
    if (LL_TIM_CC_IsEnabledChannel(TimHandle.Instance, _TIM_ChannelConvert_HAL2LL(channel, obj)) == 0) {
        // If channel is not enabled, proceed to channel configuration
        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, channel) != HAL_OK) {
            return -1;
        }
    } 

    // Save for future use
    obj->period = period_us;
#if !defined(PWMOUT_INVERTED_NOT_SUPPORTED)
    if (obj->inverted) {
        HAL_TIMEx_PWMN_Start(&TimHandle, channel);
    } else
#endif
    {
        HAL_TIM_PWM_Start(&TimHandle, channel);
    }
    
    return 0;
}

// setting pwm to hardware pin - instead analogWrite()
void _pwm_write(pwmout_t *obj, float value){
  TIM_HandleTypeDef TimHandle;
  int channel = 0;

  TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);
  
  if (value < (float)0.0) {
      value = 0.0;
  } else if (value > (float)1.0) {
      value = 1.0;
  }

  obj->pulse = (uint32_t)((float)obj->period * value + 0.5);

  switch (obj->channel) {
      case 1:
          channel = TIM_CHANNEL_1;
          break;
      case 2:
          channel = TIM_CHANNEL_2;
          break;
      case 3:
          channel = TIM_CHANNEL_3;
          break;
      case 4:
          channel = TIM_CHANNEL_4;
          break;
      default:
          return;
  }
  
  // If channel already enabled, only update compare value to avoid glitch
  __HAL_TIM_SET_COMPARE(&TimHandle, channel, obj->pulse / obj->prescaler);
} 

// init low side pin
// HardwareTimer* _initPinPWMLow(uint32_t PWM_freq, int ulPin)
// {
//   PinName pin = digitalPinToPinName(ulPin);
//   TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
//   uint32_t index = get_timer_index(Instance);

//   if (HardwareTimer_Handle[index] == NULL) {
//     HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
//     HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
//     HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
//     TIM_OC_InitTypeDef sConfigOC = TIM_OC_InitTypeDef();
//     sConfigOC.OCMode = TIM_OCMODE_PWM2;
//     sConfigOC.Pulse = 100;
//     sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//     sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//     sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//     sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
//     sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//     uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
//     HAL_TIM_PWM_ConfigChannel(&(HardwareTimer_Handle[index]->handle), &sConfigOC, channel);
//   }
//   HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
//   uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
//   HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, pin);
//   HT->setOverflow(PWM_freq, HERTZ_FORMAT);
//   HT->pause();
//   HT->refresh();
//   return HT;
// }


// align the timers to end the init
void _alignPWMTimers(pwmout_t *t1, pwmout_t *t2){
    TIM_HandleTypeDef TimHandle1, TimHandle2;
    TimHandle1.Instance = (TIM_TypeDef *)(t1->pwm);
    TimHandle2.Instance = (TIM_TypeDef *)(t2->pwm);
    __HAL_TIM_DISABLE(&TimHandle1);
    __HAL_TIM_DISABLE(&TimHandle2);
    __HAL_TIM_ENABLE(&TimHandle1); 
    __HAL_TIM_ENABLE(&TimHandle2); 
}

// align the timers to end the init
void _alignPWMTimers(pwmout_t *t1, pwmout_t *t2, pwmout_t *t3){
    TIM_HandleTypeDef TimHandle1, TimHandle2, TimHandle3;
    TimHandle1.Instance = (TIM_TypeDef *)(t1->pwm);
    TimHandle2.Instance = (TIM_TypeDef *)(t2->pwm);
    TimHandle3.Instance = (TIM_TypeDef *)(t3->pwm);
    __HAL_TIM_DISABLE(&TimHandle1);
    __HAL_TIM_DISABLE(&TimHandle2);
    __HAL_TIM_DISABLE(&TimHandle3);
    __HAL_TIM_ENABLE(&TimHandle1); 
    __HAL_TIM_ENABLE(&TimHandle2); 
    __HAL_TIM_ENABLE(&TimHandle3); 
}

// align the timers to end the init
void _alignPWMTimers(pwmout_t *t1, pwmout_t *t2, pwmout_t *t3, pwmout_t *t4){
    TIM_HandleTypeDef TimHandle1, TimHandle2, TimHandle3, TimHandle4;
    TimHandle1.Instance = (TIM_TypeDef *)(t1->pwm);
    TimHandle2.Instance = (TIM_TypeDef *)(t2->pwm);
    TimHandle3.Instance = (TIM_TypeDef *)(t3->pwm);
    TimHandle4.Instance = (TIM_TypeDef *)(t4->pwm);
    __HAL_TIM_DISABLE(&TimHandle1);
    __HAL_TIM_DISABLE(&TimHandle2);
    __HAL_TIM_DISABLE(&TimHandle3);
    __HAL_TIM_DISABLE(&TimHandle4);
    __HAL_TIM_ENABLE(&TimHandle1); 
    __HAL_TIM_ENABLE(&TimHandle2); 
    __HAL_TIM_ENABLE(&TimHandle3);  
    __HAL_TIM_ENABLE(&TimHandle4); 
}

// // configure hardware 6pwm interface only one timer with inverted channels
// HardwareTimer* _initHardware6PWMInterface(uint32_t PWM_freq, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l)
// {
//   PinName uhPinName = digitalPinToPinName(pinA_h);
//   PinName ulPinName = digitalPinToPinName(pinA_l);
//   PinName vhPinName = digitalPinToPinName(pinB_h);
//   PinName vlPinName = digitalPinToPinName(pinB_l);
//   PinName whPinName = digitalPinToPinName(pinC_h);
//   PinName wlPinName = digitalPinToPinName(pinC_l);

//   TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM);

//   uint32_t index = get_timer_index(Instance);

//   if (HardwareTimer_Handle[index] == NULL) {
//     HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM));
//     HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
//     HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
//     ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow(PWM_freq, HERTZ_FORMAT);
//   }
//   HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(uhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, uhPinName);
//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(ulPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, ulPinName);
//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(vhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vhPinName);
//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(vlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vlPinName);
//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(whPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, whPinName);
//   HT->setMode(STM_PIN_CHANNEL(pinmap_function(wlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, wlPinName);

//   // dead time is set in nanoseconds
//   uint32_t dead_time_ns = (float)(1e9f/PWM_freq)*dead_zone;
//   uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(HT->getHandle()->Instance), dead_time_ns);
//   LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, dead_time); // deadtime is non linear!
//   LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

//   HT->pause();
//   HT->refresh();
//   HT->resume();
//   return HT;
// }


// // returns 0 if each pair of pwm channels has same channel
// // returns 1 all the channels belong to the same timer - hardware inverted channels
// // returns -1 if neither - avoid configuring - error!!!
// int _interfaceType(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
//   PinName nameAH = digitalPinToPinName(pinA_h);
//   PinName nameAL = digitalPinToPinName(pinA_l);
//   PinName nameBH = digitalPinToPinName(pinB_h);
//   PinName nameBL = digitalPinToPinName(pinB_l);
//   PinName nameCH = digitalPinToPinName(pinC_h);
//   PinName nameCL = digitalPinToPinName(pinC_l);
//   int tim1 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAH, PinMap_PWM));
//   int tim2 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAL, PinMap_PWM));
//   int tim3 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBH, PinMap_PWM));
//   int tim4 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBL, PinMap_PWM));
//   int tim5 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCH, PinMap_PWM));
//   int tim6 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCL, PinMap_PWM));
//   if(tim1 == tim2 && tim2==tim3 && tim3==tim4  && tim4==tim5 && tim5==tim6)
//     return _HARDWARE_6PWM; // hardware 6pwm interface - only on timer
//   else if(tim1 == tim2 && tim3==tim4  && tim5==tim6)
//     return _SOFTWARE_6PWM; // software 6pwm interface - each pair of high-low side same timer
//   else
//     return _ERROR_6PWM; // might be error avoid configuration
// }



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  
  PortentaDriverParams* params = new PortentaDriverParams();
  params->pwm_frequency = pwm_frequency;

  core_util_critical_section_enter();
  _pwm_init(&(params->pins[0]), pinA, (long)pwm_frequency);
  _pwm_init(&(params->pins[1]), pinB, (long)pwm_frequency);
  // allign the timers
  _alignPWMTimers(&(params->pins[0]), &(params->pins[1]));
  core_util_critical_section_exit();
  return params;
}


// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  PortentaDriverParams* params = new PortentaDriverParams();
  params->pwm_frequency = pwm_frequency;

  core_util_critical_section_enter();
  _pwm_init(&(params->pins[0]), pinA, (long)pwm_frequency);
  _pwm_init(&(params->pins[1]), pinB, (long)pwm_frequency);
  _pwm_init(&(params->pins[2]), pinC, (long)pwm_frequency);
  // allign the timers
  _alignPWMTimers(&(params->pins[0]), &(params->pins[1]), &(params->pins[2]));
  core_util_critical_section_exit();

  return params;
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

  PortentaDriverParams* params = new PortentaDriverParams();
  params->pwm_frequency = pwm_frequency;

  core_util_critical_section_enter();
  _pwm_init(&(params->pins[0]), pinA, (long)pwm_frequency);
  _pwm_init(&(params->pins[1]), pinB, (long)pwm_frequency);
  _pwm_init(&(params->pins[2]), pinC, (long)pwm_frequency);
  _pwm_init(&(params->pins[3]), pinD, (long)pwm_frequency);
  // allign the timers
  _alignPWMTimers(&(params->pins[0]), &(params->pins[1]), &(params->pins[2]), &(params->pins[3]));
  core_util_critical_section_exit();

  return params;
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
//- hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
    core_util_critical_section_enter();
    _pwm_write(&(((PortentaDriverParams*)params)->pins[0]), (float)dc_a);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[1]), (float)dc_b);
    core_util_critical_section_exit();
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
    core_util_critical_section_enter();
    _pwm_write(&(((PortentaDriverParams*)params)->pins[0]), (float)dc_a);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[1]), (float)dc_b);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[2]), (float)dc_c);
    core_util_critical_section_exit();
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
//- hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
    core_util_critical_section_enter();
    _pwm_write(&(((PortentaDriverParams*)params)->pins[0]), (float)dc_1a);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[1]), (float)dc_1b);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[2]), (float)dc_2a);
    _pwm_write(&(((PortentaDriverParams*)params)->pins[3]), (float)dc_2b);
    core_util_critical_section_exit();
}


// 6-PWM currently not supported, defer to generic, which also doesn't support it ;-)

// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
//void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  // if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  // else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to |%0kHz max
  // // center-aligned frequency is uses two periods
  // pwm_frequency *=2;

  // // find configuration
  // int config = _interfaceType(pinA_h, pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l);
  // // configure accordingly
  // switch(config){
  //   case _ERROR_6PWM:
  //     return -1; // fail
  //     break;
  //   case _HARDWARE_6PWM:
  //     _initHardware6PWMInterface(pwm_frequency, dead_zone, pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
  //     break;
  //   case _SOFTWARE_6PWM:
  //     HardwareTimer* HT1 = _initPinPWMHigh(pwm_frequency, pinA_h);
  //     _initPinPWMLow(pwm_frequency, pinA_l);
  //     HardwareTimer* HT2 = _initPinPWMHigh(pwm_frequency,pinB_h);
  //     _initPinPWMLow(pwm_frequency, pinB_l);
  //     HardwareTimer* HT3 = _initPinPWMHigh(pwm_frequency,pinC_h);
  //     _initPinPWMLow(pwm_frequency, pinC_l);
  //     _alignPWMTimers(HT1, HT2, HT3);
  //     break;
  // }
//   return -1; // success
// }

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
//void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // // find configuration
  // int config = _interfaceType(pinA_h, pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l);
  // // set pwm accordingly
  // switch(config){
  //   case _HARDWARE_6PWM:
  //     _setPwm(pinA_h, _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  //     _setPwm(pinB_h, _PWM_RANGE*dc_b, _PWM_RESOLUTION);
  //     _setPwm(pinC_h, _PWM_RANGE*dc_c, _PWM_RESOLUTION);
  //     break;
  //   case _SOFTWARE_6PWM:
  //     _setPwm(pinA_l, _constrain(dc_a + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     _setPwm(pinA_h, _constrain(dc_a - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     _setPwm(pinB_l, _constrain(dc_b + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     _setPwm(pinB_h, _constrain(dc_b - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     _setPwm(pinC_l, _constrain(dc_c + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     _setPwm(pinC_h, _constrain(dc_c - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
  //     break;
  // }
//}
#endif