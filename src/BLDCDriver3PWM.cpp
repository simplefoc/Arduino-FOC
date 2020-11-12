#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enable_pin = en;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( enable_pin != NOT_SET ) digitalWrite(enable_pin, HIGH);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( enable_pin != NOT_SET ) digitalWrite(enable_pin, LOW);

}

// init hardware pins   
void BLDCDriver3PWM::init() {

  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if(enable_pin != NOT_SET) pinMode(enable_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _setPwmFrequency(pwm_frequency, pwmA, pwmB, pwmC);
}


// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {  
  // limit the voltage in driver
  Ua = _constrain(Ua, -voltage_limit, voltage_limit);
  Ub = _constrain(Ub, -voltage_limit, voltage_limit);
  Uc = _constrain(Uc, -voltage_limit, voltage_limit);    
  // calculate duty cycle
  // limited in [0,1]
  float dc_a = _constrain(Ua / voltage_power_supply, 0 , 1 );
  float dc_b = _constrain(Ub / voltage_power_supply, 0 , 1 );
  float dc_c = _constrain(Uc / voltage_power_supply, 0 , 1 );
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
}