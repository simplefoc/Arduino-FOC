#include "hardware_utils.h"

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
void _setPwmFrequency(const int pinA, const int pinB, const int pinC, const int pinD) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // if arduino uno and other ATmega328p chips
   //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
  if (pinA == 5 || pinA == 6 || pinB == 5 || pinB == 6 || pinC == 5 || pinC == 6 || pinD == 5 || pinD == 6 ) {
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
      TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1
  }
  if (pinA == 9 || pinA == 10 || pinB == 9 || pinB == 10 || pinC == 9 || pinC == 10|| pinD == 9 || pinD == 10 )
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
  if (pinA == 3 || pinA == 11 || pinB == 3 || pinB == 11 || pinC == 3 || pinC == 11 || pinD == 3 || pinD == 11 ) 
      TCCR2B = ((TCCR2B & 0b11111000) | 0x01);// set prescaler to 1
  
#elif defined(_STM32_DEF_) // if stm chips

  analogWrite(pinA, 0);
  analogWriteFrequency(50000);  // set 50kHz
  analogWriteResolution(12); // resolution 12 bit 0 - 4096
  analogWrite(pinB, 0);
  analogWriteFrequency(50000);  // set 50kHz
  analogWriteResolution(12); // resolution 12 bit 0 - 4096
  analogWrite(pinC, 0);
  analogWriteFrequency(50000);  // set 50kHz
  analogWriteResolution(12); // resolution 12 bit 0 - 4096
  if(pinD) {
    analogWrite(pinD, 0);
    analogWriteFrequency(50000);  // set 50kHz
    analogWriteResolution(12); // resolution 12 bit 0 - 4096

#elif defined(__arm__) && defined(CORE_TEENSY) //if teensy 3x / 4x / LC boards
  analogWrite(pinA, 0);
  analogWriteFrequency(pinA, 50000);  // set 50kHz
  analogWrite(pinB, 0);
  analogWriteFrequency(pinB, 50000);  // set 50kHz
  analogWrite(pinC, 0);
  analogWriteFrequency(pinC, 50000);  // set 50kHz
  if(pinD) {
    analogWrite(pinD, 0);
    analogWriteFrequency(50000);  // set 50kHz

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
#elif defined(_STM32_DEF_) // STM32 devices
  // transform duty cycle from [0,1] to [0,4095]
  analogWrite(pinA, 4095.0*dc_a);
  analogWrite(pinB, 4095.0*dc_b);
  analogWrite(pinC, 4095.0*dc_c);
#else  // Arduino & Teensy
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255*dc_a);
  analogWrite(pinB, 255*dc_b);
  analogWrite(pinC, 255*dc_c);
#endif
}

// function setting the pwm duty cycle to the hardware
//- hardware speciffic
//
// Arduino and STM32 devices use analogWrite()
// ESP32 uses MCPWM
void _writeDutyCycle(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
#if defined(ESP_H) // if ESP32 boards
  // not sure how to handle this
#elif defined(_STM32_DEF_) // STM32 devices
  // transform duty cycle from [0,1] to [0,4095]
  analogWrite(pin1A, 4095.0*dc_1a);
  analogWrite(pin1B, 4095.0*dc_1b);
  analogWrite(pin2A, 4095.0*dc_2a);
  analogWrite(pin2B, 4095.0*dc_2b);
#else // Arduino & Teensy
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pin1A, 255*dc_1a);
  analogWrite(pin1B, 255*dc_1b);
  analogWrite(pin2A, 255*dc_2a);
  analogWrite(pin2B, 255*dc_2b);
#endif
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