#include "BLDCDriver6PWM.h"

BLDCDriver6PWM::BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en){
  // Pin initialization
  pwmA_h = phA_h;
  pwmB_h = phB_h;
  pwmC_h = phC_h;
  pwmA_l = phA_l;
  pwmB_l = phB_l;
  pwmC_l = phC_l;

  // enable_pin pin
  enable_pin = en;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

  // dead zone initial - 2%
  dead_zone = 0.02f;

}

// enable motor driver
void  BLDCDriver6PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin) ) digitalWrite(enable_pin, enable_active_high);
    // set zero to PWM
    setPwm(0, 0, 0);
}

// disable motor driver
void BLDCDriver6PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin) ) digitalWrite(enable_pin, !enable_active_high);

}

// init hardware pins
int BLDCDriver6PWM::init() {

  // PWM pins
  pinMode(pwmA_h, OUTPUT);
  pinMode(pwmB_h, OUTPUT);
  pinMode(pwmC_h, OUTPUT);
  pinMode(pwmA_l, OUTPUT);
  pinMode(pwmB_l, OUTPUT);
  pinMode(pwmC_l, OUTPUT);
  if(_isset(enable_pin)) pinMode(enable_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // configure 6pwm
  // hardware specific function - depending on driver and mcu
  int result = _configure6PWM(pwm_frequency, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
  initialized = (result==0);
  return result;
}

// Set voltage to the pwm pin
void BLDCDriver6PWM::setPwm(float Ua, float Ub, float Uc) {
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, voltage_limit);
  Ub = _constrain(Ub, 0, voltage_limit);
  Uc = _constrain(Uc, 0, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle6PWM(dc_a, dc_b, dc_c, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
}


// Set voltage to the pwm pin
void BLDCDriver6PWM::setPhaseState(int sa, int sb, int sc) {
  // TODO implement disabling
}
