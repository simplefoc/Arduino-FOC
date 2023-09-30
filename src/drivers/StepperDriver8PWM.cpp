#include "StepperDriver8PWM.h"

StepperDriver8PWM::StepperDriver8PWM(int ph1A, int ph1B, int ph2A, int ph2B, int ph3A, int ph3B, int ph4A, int ph4B, int en1, int en2, int en3, int en4) {
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;
  pwm3A = ph3A;
  pwm3B = ph3B;
  pwm4A = ph4A;
  pwm4B = ph4B;

  // Enable pins
  enable_pin1 = en1;
  enable_pin2 = en2;
  enable_pin3 = en3;
  enable_pin4 = en4;

  // Default values
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

 

  pinMode(enable_pin1, OUTPUT);
  pinMode(enable_pin2, OUTPUT);
  disable();

   // dead zone initial - 2%
  dead_zone = 0.02f;
  
}


// enable motor driver
void  StepperDriver8PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
    if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
    if ( _isset(enable_pin3) ) digitalWrite(enable_pin3, HIGH);
    if ( _isset(enable_pin4) ) digitalWrite(enable_pin4, HIGH);
    // set zero to PWM
    //setPwm(0,0);
}

// disable motor driver
void StepperDriver8PWM::disable()
{
  // set zero to PWM
  //setPwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
  if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);
  if ( _isset(enable_pin3) ) digitalWrite(enable_pin3, LOW);
  if ( _isset(enable_pin4) ) digitalWrite(enable_pin4, LOW);

}





// init hardware pins for 8PWM control
int StepperDriver8PWM::init() {

     
              
        // PWM pins
        pinMode(pwm1A, OUTPUT);
        pinMode(pwm1B, OUTPUT);
        pinMode(pwm2A, OUTPUT);
        pinMode(pwm2B, OUTPUT);
        pinMode(pwm3A, OUTPUT);
        pinMode(pwm3B, OUTPUT);
        pinMode(pwm4A, OUTPUT);
        pinMode(pwm4B, OUTPUT);



  if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
  if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);
  if( _isset(enable_pin3) ) pinMode(enable_pin3, OUTPUT);
  if( _isset(enable_pin4) ) pinMode(enable_pin4, OUTPUT);

  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;


   phase_state[0] = PhaseState::PHASE_ON;
   phase_state[1] = PhaseState::PHASE_ON;
   phase_state[2] = PhaseState::PHASE_ON;
   phase_state[3] = PhaseState::PHASE_ON;

   // set zero to PWM
  dc_1a = dc_1b = dc_1c = dc_1d = 0;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure8PWM(pwm_frequency, dead_zone, pwm1A, pwm1B, pwm2A, pwm2B, pwm3A, pwm3B, pwm4A, pwm4B);
 

         initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);  
         return params!=SIMPLEFOC_DRIVER_INIT_FAILED;

        


}



      // Set voltage to the pwm pin for 8PWM control
      // Same as 4pwm control, only the phases are setup to be complimentarry. Setting the dutycycle to a complimentarry output, is equal to setting PWM to PWM 1-line (PWM) FET driver
      void StepperDriver8PWM::setPwm(float Ualpha, float Ubeta) {
      float duty_cycle1A(0.0f),duty_cycle1B(0.0f),duty_cycle2A(0.0f),duty_cycle2B(0.0f);

        // limit the voltage in driver
        Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
        Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);


        // hardware specific writing
       if( Ualpha > 0 )
       duty_cycle1A = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
       else
       duty_cycle2B = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);

       if( Ubeta > 0 )
       duty_cycle1B = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
       else
       duty_cycle2A = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);


        // write to hardware
        _writeDutyCycle8PWM(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, phase_state, params);
      }

      //Yes, with a duty cycle of 30%, and reversed polarity, the high side will be on for 
      //30% and off for 70%, and the low side will be off for 30% and on for 70%.