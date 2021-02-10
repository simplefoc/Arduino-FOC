#include "StepperDriver2PWM2Pin.h"

StepperDriver2PWM2Pin::StepperDriver2PWM2Pin(int ph1PWM, int ph1Dir,int ph2PWM, int ph2Dir, int en1, int en2) {
  // Pin initialization
  pwm1 = ph1PWM; //!< phase 1 pwm pin number
  dir1 = ph1Dir; //!< phase 1 dir pin number
  pwm2 = ph2PWM; //!< phase 2 pwm pin number
  dir2 = ph2Dir; //!< phase 2 dir pin number

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;

}

// enable motor driver
void  StepperDriver2PWM2Pin::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, HIGH);
    if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, HIGH);
    // set zero to PWM
    setPwm(0,0);
}

// disable motor driver
void StepperDriver2PWM2Pin::disable()
{
  // set zero to PWM
  setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, LOW);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, LOW);

}

// init hardware pins   
int StepperDriver2PWM2Pin::init() {
  // a bit of separation
  _delay(1000);

  // PWM pins
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

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
void StepperDriver2PWM2Pin::setPwm(float Ualpha, float Ubeta) {  
  float duty_cycle1(0.0),duty_cycle2(0.0);
  // limit the voltage in driver
  Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
  Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
  // hardware specific writing
  duty_cycle1 = _constrain(abs(Ualpha)/voltage_power_supply,0.0,1.0);
  duty_cycle2 = _constrain(abs(Ubeta)/voltage_power_supply,0.0,1.0);
  // set direction
  digitalWrite(dir2, Ualpha > 0 ? HIGH : LOW);
  digitalWrite(dir1, Ubeta > 0 ? HIGH : LOW);

  _delay(1);
  // Serial.print(Ualpha);
  // Serial.print("\t");
  // Serial.print(Ubeta);
  // Serial.print("\t");
  // Serial.print(duty_cycle1);
  // Serial.print("\t");
  // Serial.println(duty_cycle2);

  // write to hardware
  _writeDutyCycle2PWM(duty_cycle1, duty_cycle2, pwm1, pwm2);
}