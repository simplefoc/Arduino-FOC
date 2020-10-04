#include "FOCutils.h"

#if defined(ESP_H) // if ESP32 board
// empty motor slot 
#define _EMPTY_SLOT -20

// structure containing motor slot configuration
// this library supports up to 4 motors
typedef struct {
  int pinA;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator;
  mcpwm_io_signals_t mcpwm_a;
  mcpwm_io_signals_t mcpwm_b;
  mcpwm_io_signals_t mcpwm_c;

} motor_slots_t;

// define motor slots array
motor_slots_t esp32_motor_slots[4] =  { 
  {_EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A}, // 1st motor will be MCPWM0 channel A
  {_EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B}, // 2nd motor will be MCPWM0 channel B
  {_EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A}, // 3rd motor will be MCPWM1 channel A
  {_EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B}  // 4th motor will be MCPWM1 channel B
  };

#endif


// function setting the high pwm frequency to the supplied pins
// - hardware speciffic
// supports Arudino/ATmega328, STM32 and ESP32 
void _setPwmFrequency(const int pinA, const int pinB, const int pinC) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // if arduino uno and other ATmega328p chips
  //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
  if (pinA == 5 || pinA == 6 || pinB == 5 || pinB == 6 || pinC == 5 || pinC == 6 ) {
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
      TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1
  }
  if (pinA == 9 || pinA == 10 || pinB == 9 || pinB == 10 || pinC == 9 || pinC == 10 )
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
  if (pinA == 3 || pinA == 11 || pinB == 3 || pinB == 11 || pinC == 3 || pinC == 11 ) 
      TCCR2B = ((TCCR2B & 0b11111000) | 0x01);// set prescaler to 1
  
#elif defined(_STM32_DEF_) // if stm chips

  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceA = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinA), PinMap_PWM);
  uint32_t channelA = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimA = new HardwareTimer(InstanceA);
  MyTimA->setMode(channelA, TIMER_OUTPUT_COMPARE_PWM1, pinA);
  MyTimA->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimA->resume();
  
  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceB = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinB), PinMap_PWM);
  uint32_t channelB = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimB = new HardwareTimer(InstanceB);
  MyTimB->setMode(channelB, TIMER_OUTPUT_COMPARE_PWM1, pinB);
  MyTimB->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimB->resume();
  
  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceC = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinC), PinMap_PWM);
  uint32_t channelC = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimC = new HardwareTimer(InstanceC);
  MyTimC->setMode(channelC, TIMER_OUTPUT_COMPARE_PWM1, pinC);
  MyTimC->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimC->resume();
  
#elif defined(ESP_H) // if esp32 boards

  motor_slots_t m_slot = {};

  // determine which motor are we connecting
  // and set the appropriate configuration parameters 
  for(int i = 0; i < 4; i++){
    if(esp32_motor_slots[i].pinA == _EMPTY_SLOT){ // put the new motor in the first empty slot
      esp32_motor_slots[i].pinA = pinA;
      m_slot = esp32_motor_slots[i];
      break;
    }
  }
        
  // configure pins
  mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_a, pinA);
  mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_b, pinB);
  mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_c, pinC);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 4000000;  //frequency = 20000Hz
  pwm_config.cmpr_a = 0;      //duty cycle of PWMxA = 50.0%
  pwm_config.cmpr_b = 0;      //duty cycle of PWMxB = 50.0%
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Up-down counter (triangle wave)
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
  mcpwm_init(m_slot.mcpwm_unit, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(m_slot.mcpwm_unit, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(m_slot.mcpwm_unit, MCPWM_TIMER_2, &pwm_config);    //Configure PWM0A & PWM0B with above settings

  _delay(100);

  mcpwm_stop(m_slot.mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_stop(m_slot.mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_stop(m_slot.mcpwm_unit, MCPWM_TIMER_2);
  m_slot.mcpwm_num->clk_cfg.prescale = 0;

  m_slot.mcpwm_num->timer[0].period.prescale = 4;
  m_slot.mcpwm_num->timer[1].period.prescale = 4;
  m_slot.mcpwm_num->timer[2].period.prescale = 4;    
  _delay(1);
  m_slot.mcpwm_num->timer[0].period.period = 2048;
  m_slot.mcpwm_num->timer[1].period.period = 2048;
  m_slot.mcpwm_num->timer[2].period.period = 2048;
  _delay(1);
  m_slot.mcpwm_num->timer[0].period.upmethod = 0;
  m_slot.mcpwm_num->timer[1].period.upmethod = 0;
  m_slot.mcpwm_num->timer[2].period.upmethod = 0;
  _delay(1); 
  mcpwm_start(m_slot.mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_start(m_slot.mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_start(m_slot.mcpwm_unit, MCPWM_TIMER_2);

  mcpwm_sync_enable(m_slot.mcpwm_unit, MCPWM_TIMER_0, MCPWM_SELECT_SYNC_INT0, 0);
  mcpwm_sync_enable(m_slot.mcpwm_unit, MCPWM_TIMER_1, MCPWM_SELECT_SYNC_INT0, 0);
  mcpwm_sync_enable(m_slot.mcpwm_unit, MCPWM_TIMER_2, MCPWM_SELECT_SYNC_INT0, 0);
  _delay(1);
  m_slot.mcpwm_num->timer[0].sync.out_sel = 1;
  _delay(1);
  m_slot.mcpwm_num->timer[0].sync.out_sel = 0;
#endif
}

void _setPwmFrequencyLow(const int pinA, const int pinB, const int pinC) {
  
  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceA = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinA), PinMap_PWM);
  uint32_t channelA = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimA = new HardwareTimer(InstanceA);
  MyTimA->setMode(channelA, TIMER_OUTPUT_COMPARE_PWM2, pinA);
  MyTimA->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimA->resume();

  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceB = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinB), PinMap_PWM);
  uint32_t channelB = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimB = new HardwareTimer(InstanceB);
  MyTimB->setMode(channelB, TIMER_OUTPUT_COMPARE_PWM2, pinB);
  MyTimB->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimB->resume();
  
  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *InstanceC = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinC), PinMap_PWM);
  uint32_t channelC = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTimC = new HardwareTimer(InstanceC);
  MyTimC->setMode(channelC, TIMER_OUTPUT_COMPARE_PWM2, pinC);
  MyTimC->setOverflow(20, MICROSEC_FORMAT); // 50khz  
  MyTimC->resume();

  
}
// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
#if defined(ESP_H) // if ESP32 boards
  // determine which motor slot is the motor connected to 
  for(int i = 0; i < 4; i++){
    if(esp32_motor_slots[i].pinA == pinA){ // if motor slot found
      // se the PWM on the slot timers
      // transform duty cycle from [0,1] to [0,2047]
      esp32_motor_slots[i].mcpwm_num->channel[0].cmpr_value[esp32_motor_slots[i].mcpwm_operator].cmpr_val = dc_a*2047;
      esp32_motor_slots[i].mcpwm_num->channel[1].cmpr_value[esp32_motor_slots[i].mcpwm_operator].cmpr_val = dc_b*2047;
      esp32_motor_slots[i].mcpwm_num->channel[2].cmpr_value[esp32_motor_slots[i].mcpwm_operator].cmpr_val = dc_c*2047;
      break;
    }
  }
  
#elif  defined(_STM32_DEF_) // if stm chips

  // Automatically retrieve TIM instance and channel associated to pin
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinA), PinMap_PWM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setCaptureCompare(channel, dc_a*100.0, PERCENT_COMPARE_FORMAT); // 50%

  // Automatically retrieve TIM instance and channel associated to pin
  Instance  = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinB), PinMap_PWM);
  channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  MyTim = new HardwareTimer(Instance);
  MyTim->setCaptureCompare(channel, dc_b*100.0, PERCENT_COMPARE_FORMAT); // 50%
  
  // Automatically retrieve TIM instance and channel associated to pin
  Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinC), PinMap_PWM);
  channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  MyTim = new HardwareTimer(Instance);  
  MyTim->setCaptureCompare(channel, dc_c*100.0, PERCENT_COMPARE_FORMAT); // 50% 
  
//  MyTimA->setPWM(channelA, pinA, 50000, dc_a*100.0);
//  MyTimB->setPWM(channelB, pinB, 50000, dc_b*100.0);
//  MyTimC->setPWM(channelC, pinC, 50000, dc_c*100.0);
#else // Arduino 
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255*dc_a);
  analogWrite(pinB, 255*dc_b);
  analogWrite(pinC, 255*dc_c);
#endif
}
void _writeDutyCycleLow(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
//  
//  // Automatically retrieve TIM instance and channel associated to pin
//  TIM_TypeDef *InstanceA = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinA), PinMap_PWM);
//  uint32_t channelA = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinA), PinMap_PWM));
//  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
//  HardwareTimer *MyTimA = new HardwareTimer(InstanceA);
//
//  // Automatically retrieve TIM instance and channel associated to pin
//  TIM_TypeDef *InstanceB = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinB), PinMap_PWM);
//  uint32_t channelB = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinB), PinMap_PWM));
//  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
//  HardwareTimer *MyTimB = new HardwareTimer(InstanceB);
//  
//  // Automatically retrieve TIM instance and channel associated to pin
//  TIM_TypeDef *InstanceC = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinC), PinMap_PWM);
//  uint32_t channelC = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pinC), PinMap_PWM));
//  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
//  HardwareTimer *MyTimC = new HardwareTimer(InstanceC);
//
//  
//  MyTimA->setCaptureCompare(channelA, dc_a*100.0, PERCENT_COMPARE_FORMAT); // 50%
//  MyTimB->setCaptureCompare(channelB, dc_b*100.0, PERCENT_COMPARE_FORMAT); // 50%
//  MyTimC->setCaptureCompare(channelC, dc_c*100.0, PERCENT_COMPARE_FORMAT); // 50%
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // if arduino uno and other atmega328p chips
  // use while instad of delay, 
  // due to wrong measurement based on changed timer0
  unsigned long t = _micros() + ms*1000;
  while( _micros() < t ){}; 
#else
  // regular micros
  delay(ms);
#endif
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// if arduino uno and other atmega328p chips
    //return the value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
#else
  // regular micros
  return micros();
#endif
}


// int array instead of float array 
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
    //return sine_array[(int)(126.6873* a)];           // float array optimized
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
  }else if(a < _PI){
    // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
  }else if(a < _3PI_2){
    // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
  } else {
    // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
  }
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
