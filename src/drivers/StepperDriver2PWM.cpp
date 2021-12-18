#include "StepperDriver2PWM.h"

StepperDriver2PWM::StepperDriver2PWM(int _pwm1, int* _in1, int _pwm2, int* _in2, int en1, int en2){
  // Pin initialization
  pwm1 = _pwm1; // phase 1 pwm pin number
  dir1a = _in1[0]; // phase 1 INA pin number
  dir1b = _in1[1]; // phase 1 INB pin number
  pwm2 = _pwm2; // phase 2 pwm pin number
  dir2a = _in2[0]; // phase 2 INA pin number
  dir2b = _in2[1]; // phase 2 INB pin number

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

StepperDriver2PWM::StepperDriver2PWM(int _pwm1, int _dir1, int _pwm2, int _dir2, int en1, int en2){
  // Pin initialization
  pwm1 = _pwm1; // phase 1 pwm pin number
  dir1a = _dir1; // phase 1 direction pin
  pwm2 = _pwm2; // phase 2 pwm pin number
  dir2a = _dir2; // phase 2 direction pin
  // these pins are not used
  dir1b = NOT_SET;
  dir2b = NOT_SET;

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  StepperDriver2PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
    if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
    // set zero to PWM
    setPwm(0,0);
}

// disable motor driver
void StepperDriver2PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
  if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);

}

// init hardware pins
int StepperDriver2PWM::init() {
  // PWM pins
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir1a, OUTPUT);
  pinMode(dir2a, OUTPUT);
  if( _isset(dir1b) ) pinMode(dir1b, OUTPUT);
  if( _isset(dir2b) ) pinMode(dir2b, OUTPUT);

  if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
  if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);

  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit)  || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _configure2PWM(pwm_frequency, pwm1, pwm2);
  initialized = true; // TODO atm the api gives no feedback if initialization succeeded
  return 0;
}


// Set voltage to the pwm pin
void StepperDriver2PWM::setPwm(float Ua, float Ub) {
  float duty_cycle1(0.0f),duty_cycle2(0.0f);
  // limit the voltage in driver
  Ua = _constrain(Ua, -voltage_limit, voltage_limit);
  Ub = _constrain(Ub, -voltage_limit, voltage_limit);
  // hardware specific writing
  duty_cycle1 = _constrain(abs(Ua)/voltage_power_supply,0.0f,1.0f);
  duty_cycle2 = _constrain(abs(Ub)/voltage_power_supply,0.0f,1.0f);

  // phase 1 direction
  digitalWrite(dir1a, Ua >= 0 ? LOW : HIGH);
  if( _isset(dir1b) ) digitalWrite(dir1b, Ua >= 0 ? HIGH : LOW );
  // phase 2 direction
  digitalWrite(dir2a, Ub >= 0 ? LOW : HIGH);
  if( _isset(dir2b) ) digitalWrite(dir2b, Ub >= 0 ? HIGH : LOW );

  // write to hardware
  _writeDutyCycle2PWM(duty_cycle1, duty_cycle2, pwm1, pwm2);
}