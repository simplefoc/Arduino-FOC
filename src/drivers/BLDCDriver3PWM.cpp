#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( enableA_pin != NOT_SET ) digitalWrite(enableA_pin, HIGH);
    if ( enableB_pin != NOT_SET ) digitalWrite(enableB_pin, HIGH);
    if ( enableC_pin != NOT_SET ) digitalWrite(enableC_pin, HIGH);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( enableA_pin != NOT_SET ) digitalWrite(enableA_pin, LOW);
  if ( enableB_pin != NOT_SET ) digitalWrite(enableB_pin, LOW);
  if ( enableC_pin != NOT_SET ) digitalWrite(enableC_pin, LOW);

}

// init hardware pins   
int BLDCDriver3PWM::init() {
  // a bit of separation
  _delay(1000);

  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if(enableA_pin != NOT_SET) pinMode(enableA_pin, OUTPUT);
  if(enableB_pin != NOT_SET) pinMode(enableB_pin, OUTPUT);
  if(enableC_pin != NOT_SET) pinMode(enableC_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  return 0;
}


// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {  
  // disable if needed
  if(enableA_pin != NOT_SET ) digitalWrite(enableA_pin, Ua == HIGH_IMPEDANCE ? LOW : HIGH);
  if(enableB_pin != NOT_SET ) digitalWrite(enableB_pin, Ub == HIGH_IMPEDANCE ? LOW : HIGH);
  if(enableC_pin != NOT_SET ) digitalWrite(enableC_pin, Uc == HIGH_IMPEDANCE ? LOW : HIGH);
  
  // voltage of the high-impedance phase if it is not possible to disable it
  // will be in exactly in the middle of the other two
  if (Ua == HIGH_IMPEDANCE) Ua = (Ub + Uc)/2; 
  if (Ub == HIGH_IMPEDANCE) Ub = (Ua + Uc)/2; 
  if (Uc == HIGH_IMPEDANCE) Uc = (Ua + Ub)/2; 
  
  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0, voltage_limit);
  Ub = _constrain(Ub, 0.0, voltage_limit);
  Uc = _constrain(Uc, 0.0, voltage_limit);    
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0 , 1.0 );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0 , 1.0 );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0 , 1.0 );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
}