#include "StepperDriver2PWM.h"

StepperDriver2PWM::StepperDriver2PWM(int ph1PWM, int ph1INA, int ph1INB, int ph2PWM, int ph2INA, int ph2INB, int en1, int en2){
  // Pin initialization
  pwm1 = ph1PWM; //!< phase 1 pwm pin number
  ina1 = ph1INA; //!< phase 1 INA pin number
  inb1 = ph1INB; //!< phase 1 INB pin number
  pwm2 = ph2PWM; //!< phase 2 pwm pin number
  ina2 = ph2INA; //!< phase 2 INA pin number
  inb2 = ph2INB; //!< phase 2 INB pin number

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;

}

// enable motor driver
void  StepperDriver2PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, HIGH);
    if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, HIGH);
    // set zero to PWM
    setPwm(0,0);
}

// disable motor driver
void StepperDriver2PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, LOW);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, LOW);

}

// init hardware pins   
int StepperDriver2PWM::init() {
  // a bit of separation
  _delay(1000);

  // PWM pins
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(ina1, OUTPUT);
  pinMode(ina2, OUTPUT);
  pinMode(inb1, OUTPUT);
  pinMode(inb2, OUTPUT);

  if(enable_pin1 != NOT_SET) pinMode(enable_pin1, OUTPUT);
  if(enable_pin2 != NOT_SET) pinMode(enable_pin2, OUTPUT);

  // sanity check for the voltage limit configuration
  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _configure2PWM(pwm_frequency, pwm1, pwm2);
  return 0;
}


// Set voltage to the pwm pin
void StepperDriver2PWM::setPwm(float Ualpha, float Ubeta) {  
  float duty_cycle1(0.0),duty_cycle2(0.0);
  // limit the voltage in driver
  Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
  Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
  // hardware specific writing
  duty_cycle1 = _constrain(abs(Ualpha)/voltage_power_supply,0.0,1.0);
  duty_cycle2 = _constrain(abs(Ubeta)/voltage_power_supply,0.0,1.0);
  // set direction
  if( Ualpha > 0 ){
    digitalWrite(inb1, LOW);
    digitalWrite(ina1, HIGH);
  }
  else{
    digitalWrite(ina1, LOW);
    digitalWrite(inb1, HIGH);
  }
    
  if( Ubeta > 0 ){
    digitalWrite(ina2, LOW);
    digitalWrite(inb2, HIGH);
  }
  else{
    digitalWrite(inb2, LOW);
    digitalWrite(ina2, HIGH);
  }

  // write to hardware
  _writeDutyCycle2PWM(duty_cycle1, duty_cycle2, pwm1, pwm2);
}