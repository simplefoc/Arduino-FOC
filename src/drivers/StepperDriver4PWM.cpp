#include "StepperDriver4PWM.h"

StepperDriver4PWM::StepperDriver4PWM(int ph1A,int ph1B,int ph2A,int ph2B,int en1, int en2){
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  StepperDriver4PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
    if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
    // set zero to PWM
    setPwm(0,0);
}

// disable motor driver
void StepperDriver4PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
  if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);

}

// init hardware pins
int StepperDriver4PWM::init() {

  // PWM pins
  pinMode(pwm1A, OUTPUT);
  pinMode(pwm1B, OUTPUT);
  pinMode(pwm2A, OUTPUT);
  pinMode(pwm2B, OUTPUT);
  if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
  if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);

  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure4PWM(pwm_frequency, pwm1A, pwm1B, pwm2A, pwm2B);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);  
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}


// Set voltage to the pwm pin
void StepperDriver4PWM::setPwm(float Ualpha, float Ubeta) {
  float duty_cycle1A(0.0f),duty_cycle1B(0.0f),duty_cycle2A(0.0f),duty_cycle2B(0.0f);
  // limit the voltage in driver
  Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
  Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
  // hardware specific writing
  if( Ualpha > 0 )
    duty_cycle1B = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle1A = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);

  if( Ubeta > 0 )
    duty_cycle2B = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle2A = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  // write to hardware
  _writeDutyCycle4PWM(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, params);
}