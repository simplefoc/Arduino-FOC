#include "../hardware_utils.h"

#if defined(ESP_H)

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

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
} bldc_motor_slots_t;
typedef struct {
  int pin1A;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator1;
  mcpwm_operator_t mcpwm_operator2;
  mcpwm_io_signals_t mcpwm_1a;
  mcpwm_io_signals_t mcpwm_1b;
  mcpwm_io_signals_t mcpwm_2a;
  mcpwm_io_signals_t mcpwm_2b;
} stepper_motor_slots_t;

// define bldc motor slots array
bldc_motor_slots_t esp32_bldc_motor_slots[4] =  { 
  {_EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A}, // 1st motor will be MCPWM0 channel A
  {_EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B}, // 2nd motor will be MCPWM0 channel B
  {_EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A}, // 3rd motor will be MCPWM1 channel A
  {_EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B}  // 4th motor will be MCPWM1 channel B
  };

// define stepper motor slots array
stepper_motor_slots_t esp32_stepper_motor_slots[2] =  { 
  {_EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM_OPR_B, MCPWM0A, MCPWM1A, MCPWM0B, MCPWM1B}, // 1st motor will be on MCPWM1
  {_EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM_OPR_B, MCPWM0A, MCPWM1A, MCPWM0B, MCPWM1B}, // 1st motor will be on MCPWM0
  };  

// configuring high frequency pwm timer
// https://hackaday.io/project/169905-esp-32-bldc-robot-actuator-controller
void _configureTimerFrequency(long pwm_frequency, mcpwm_dev_t* mcpwm_num,  mcpwm_unit_t mcpwm_unit){

  mcpwm_config_t pwm_config;
  pwm_config.frequency = pwm_frequency;  //frequency
  pwm_config.cmpr_a = 0;      //duty cycle of PWMxA = 50.0%
  pwm_config.cmpr_b = 0;      //duty cycle of PWMxB = 50.0%
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Up-down counter (triangle wave)
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_2, &pwm_config);    //Configure PWM0A & PWM0B with above settings

  _delay(100);

  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_2);
  mcpwm_num->clk_cfg.prescale = 0;

  mcpwm_num->timer[0].period.prescale = 4;
  mcpwm_num->timer[1].period.prescale = 4;
  mcpwm_num->timer[2].period.prescale = 4;    
  _delay(1);
  mcpwm_num->timer[0].period.period = 2048;
  mcpwm_num->timer[1].period.period = 2048;
  mcpwm_num->timer[2].period.period = 2048;
  _delay(1);
  mcpwm_num->timer[0].period.upmethod = 0;
  mcpwm_num->timer[1].period.upmethod = 0;
  mcpwm_num->timer[2].period.upmethod = 0;
  _delay(1); 
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_2);

  mcpwm_sync_enable(mcpwm_unit, MCPWM_TIMER_0, MCPWM_SELECT_SYNC_INT0, 0);
  mcpwm_sync_enable(mcpwm_unit, MCPWM_TIMER_1, MCPWM_SELECT_SYNC_INT0, 0);
  mcpwm_sync_enable(mcpwm_unit, MCPWM_TIMER_2, MCPWM_SELECT_SYNC_INT0, 0);
  _delay(1);
  mcpwm_num->timer[0].sync.out_sel = 1;
  _delay(1);
  mcpwm_num->timer[0].sync.out_sel = 0;
}


// function setting the high pwm frequency to the supplied pins
// - hardware speciffic
// supports Arudino/ATmega328, STM32 and ESP32 
void _setPwmFrequency(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {

  if(pwm_frequency == NOT_SET) pwm_frequency = 40000; // default frequency 20khz - centered pwm has twice lower frequency
  else pwm_frequency = constrain(2*pwm_frequency, 0, 60000); // constrain to 30kHz max - centered pwm has twice lower frequency

  if(pinD == NOT_SET){
    bldc_motor_slots_t m_slot = {};

    // determine which motor are we connecting
    // and set the appropriate configuration parameters 
    for(int i = 0; i < 4; i++){
      if(esp32_bldc_motor_slots[i].pinA == _EMPTY_SLOT){ // put the new motor in the first empty slot
        esp32_bldc_motor_slots[i].pinA = pinA;
        m_slot = esp32_bldc_motor_slots[i];
        break;
      }
    }
          
    // configure pins
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_a, pinA);
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_b, pinB);
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_c, pinC);

    // configure the timer
    _configureTimerFrequency(pwm_frequency, m_slot.mcpwm_num,  m_slot.mcpwm_unit);

  }else{
    stepper_motor_slots_t m_slot = {};
    // determine which motor are we connecting
    // and set the appropriate configuration parameters 
    for(int i = 0; i < 2; i++){
      if(esp32_stepper_motor_slots[i].pin1A == _EMPTY_SLOT){ // put the new motor in the first empty slot
        esp32_stepper_motor_slots[i].pin1A = pinA;
        m_slot = esp32_stepper_motor_slots[i];
        break;
      }
    }
          
    // configure pins
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_1a, pinA);
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_1b, pinB);
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_2a, pinC);
    mcpwm_gpio_init(m_slot.mcpwm_unit, m_slot.mcpwm_2b, pinD);

    // configure the timer
    _configureTimerFrequency(pwm_frequency, m_slot.mcpwm_num,  m_slot.mcpwm_unit);
  }
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // determine which motor slot is the motor connected to 
  for(int i = 0; i < 4; i++){
    if(esp32_bldc_motor_slots[i].pinA == pinA){ // if motor slot found
      // se the PWM on the slot timers
      // transform duty cycle from [0,1] to [0,2047]
      esp32_bldc_motor_slots[i].mcpwm_num->channel[0].cmpr_value[esp32_bldc_motor_slots[i].mcpwm_operator].cmpr_val = dc_a*2047;
      esp32_bldc_motor_slots[i].mcpwm_num->channel[1].cmpr_value[esp32_bldc_motor_slots[i].mcpwm_operator].cmpr_val = dc_b*2047;
      esp32_bldc_motor_slots[i].mcpwm_num->channel[2].cmpr_value[esp32_bldc_motor_slots[i].mcpwm_operator].cmpr_val = dc_c*2047;
      break;
    }
  }
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // determine which motor slot is the motor connected to 
  for(int i = 0; i < 2; i++){
    if(esp32_stepper_motor_slots[i].pin1A == pin1A){ // if motor slot found
      // se the PWM on the slot timers
      // transform duty cycle from [0,1] to [0,2047]
      esp32_stepper_motor_slots[i].mcpwm_num->channel[0].cmpr_value[esp32_stepper_motor_slots[i].mcpwm_operator1].cmpr_val = dc_1a*2047;
      esp32_stepper_motor_slots[i].mcpwm_num->channel[1].cmpr_value[esp32_stepper_motor_slots[i].mcpwm_operator1].cmpr_val = dc_1b*2047;
      esp32_stepper_motor_slots[i].mcpwm_num->channel[0].cmpr_value[esp32_stepper_motor_slots[i].mcpwm_operator2].cmpr_val = dc_2a*2047;
      esp32_stepper_motor_slots[i].mcpwm_num->channel[1].cmpr_value[esp32_stepper_motor_slots[i].mcpwm_operator2].cmpr_val = dc_2b*2047;
      break;
    }
  }
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
  // regular micros
  delay(ms);
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  // regular micros
  return micros();
}


#endif