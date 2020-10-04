#include "StepperMotor.h"

// StepperMotor( int phA, int phB, int phC, int pp, int cpr, int en)
// - ph1A, ph1B    - motor phase 1 pwm pins
// - ph2A, ph2B    - motor phase 2 pwm pins
// - pp            - pole pair number
// - enable pin    - (optional input)
StepperMotor::StepperMotor(int ph1A, int ph1B, int ph2A, int ph2B, int pp, int en1, int en2)
: FOCMotor()
{
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;
}

// init hardware pins   
void StepperMotor::init() {
  if(monitor_port) monitor_port->println("MOT: Init pins.");
  // PWM pins
  pinMode(pwm1A, OUTPUT);
  pinMode(pwm1B, OUTPUT);
  pinMode(pwm2A, OUTPUT);
  pinMode(pwm2B, OUTPUT);
  if ( enable_pin1 != NOT_SET ) pinMode(enable_pin1, OUTPUT);
  if ( enable_pin2 != NOT_SET ) pinMode(enable_pin2, OUTPUT);

  if(monitor_port) monitor_port->println("MOT: PWM config.");
  // Increase PWM frequency
  // make silent
  _setPwmFrequency(pwm1A, pwm1B, pwm2A, pwm2B);
  
  // sanity check for the voltage limit configuration
  if(voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  _delay(500);
  // enable motor
  if(monitor_port) monitor_port->println("MOT: Enable.");
  enable();
  _delay(500);
  
}


// disable motor driver
void StepperMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, LOW);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, LOW);
  // set zero to PWM
  setPwm(0, 0);
}
// enable motor driver
void StepperMotor::enable()
{
  // set zero to PWM
  setPwm(0, 0);
  // enable_pin the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, HIGH);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, HIGH);

}


/**
  FOC functions
*/
// FOC initialization function
int  StepperMotor::initFOC( float zero_electric_offset, Direction sensor_direction ) {
  int exit_flag = 1;
  // align motor if necessary
  // alignment necessary for encoders!
  if(zero_electric_offset != NOT_SET){
    // abosolute zero offset provided - no need to align
    zero_electric_angle = zero_electric_offset;
    // set the sensor direction - default CW
    sensor->natural_direction = sensor_direction;
  }else{
    // sensor and motor alignment
    _delay(500);
    exit_flag = alignSensor();
    _delay(500);
    }
  if(monitor_port) monitor_port->println("MOT: Motor ready.");

  return exit_flag;
}
// Encoder alignment to electrical 0 angle
int StepperMotor::alignSensor() {
  if(monitor_port) monitor_port->println("MOT: Align sensor.");
  // align the electrical phases of the motor and sensor
  // set angle -90 degrees 

  float start_angle = shaftAngle();
  for (int i = 0; i <=5; i++ ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align,  angle);
    _delay(200);
  }
  float mid_angle = shaftAngle();
  for (int i = 5; i >=0; i-- ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align,  angle);
    _delay(200);
  }
  if (mid_angle < start_angle) {
    if(monitor_port) monitor_port->println("MOT: natural_direction==CCW");
    sensor->natural_direction = Direction::CCW;
  } else if (mid_angle == start_angle) {
    if(monitor_port) monitor_port->println("MOT: Sensor failed to notice movement");
  }

  // let the motor stabilize for 2 sec
  _delay(2000);
  // set sensor to zero
  sensor->initRelativeZero();
  _delay(500);
  setPhaseVoltage(0,0);
  _delay(200);

  // find the index if available
  int exit_flag = absoluteZeroAlign();
  _delay(500);
  if(monitor_port){
    if(exit_flag< 0 ) monitor_port->println("MOT: Error: Not found!");
    if(exit_flag> 0 ) monitor_port->println("MOT: Success!");
    else  monitor_port->println("MOT: Not available!");
  }
  return exit_flag;
}


// Encoder alignment the absolute zero angle 
// - to the index
int StepperMotor::absoluteZeroAlign() {

  if(monitor_port) monitor_port->println("MOT: Absolute zero align.");
    // if no absolute zero return
  if(!sensor->hasAbsoluteZero()) return 0;
  

  if(monitor_port && sensor->needsAbsoluteZeroSearch()) monitor_port->println("MOT: Searching...");
  // search the absolute zero with small velocity
  while(sensor->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();   
    voltage_q = PID_velocity(velocity_index_search - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // align absolute zero if it has been found
  if(!sensor->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = sensor->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = _normalizeAngle(_electricalAngle(zero_offset, pole_pairs));
  }
  // return bool if zero found
  return !sensor->needsAbsoluteZeroSearch() ? 1 : -1;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void StepperMotor::loopFOC() {
  // shaft angle 
  shaft_angle = shaftAngle();
  // set the phase voltage - FOC heart function :) 
  setPhaseVoltage(voltage_q, _electricalAngle(shaft_angle,pole_pairs));
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void StepperMotor::move(float new_target) {
  // set internal target variable
  if( new_target != NOT_SET ) target = new_target;
  // get angular velocity
  shaft_velocity = shaftVelocity();
  // choose control loop
  switch (controller) {
    case ControlType::voltage:
      voltage_q =  target;
      break;
    case ControlType::angle:
      // angle set point
      // include angle loop
      shaft_angle_sp = target;
      shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
      voltage_q = PID_velocity(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // include velocity loop
      shaft_velocity_sp = target;
      voltage_q = PID_velocity(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity_openloop:
      // velocity control in open loop
      // loopFOC should not be called
      shaft_velocity_sp = target;
      velocityOpenloop(shaft_velocity_sp);
      break;
    case ControlType::angle_openloop:
      // angle control in open loop
      // loopFOC should not be called
      shaft_angle_sp = target;
      angleOpenloop(shaft_angle_sp);
      break;
  }
}


// Method using FOC to set Uq to the motor at the optimal angle
// Function implementingSine PWM algorithms
// - space vector not implemented yet
// 
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void StepperMotor::setPhaseVoltage(float Uq, float angle_el) {
  // Sinusoidal PWM modulation 
  // Inverse Park transformation

  // angle normalization in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation functions
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // Inverse park transform
  Ualpha =  -_sin(angle_el) * Uq;  // -sin(angle) * Uq;
  Ubeta =  _cos(angle_el) * Uq;    //  cos(angle) * Uq;
  // set the voltages in hardware
  setPwm(Ualpha,Ubeta);
}



// Set voltage to the pwm pin
void StepperMotor::setPwm(float Ualpha, float Ubeta) {  
  float duty_cycle1A(0.0),duty_cycle1B(0.0),duty_cycle2A(0.0),duty_cycle2B(0.0);
  // hardware specific writing
  if( Ualpha > 0 )
    duty_cycle1B = constrain(abs(Ualpha)/voltage_power_supply,0,1);
  else 
    duty_cycle1A = constrain(abs(Ualpha)/voltage_power_supply,0,1);
    
  if( Ubeta > 0 )
    duty_cycle2B = constrain(abs(Ubeta)/voltage_power_supply,0,1);
  else
    duty_cycle2A = constrain(abs(Ubeta)/voltage_power_supply,0,1);
  // write to hardware
  _writeDutyCycle(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, pwm1A, pwm1B, pwm2A, pwm2B);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
void StepperMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;

  // calculate the necessary angle to achieve target velocity
  shaft_angle += target_velocity*Ts; 

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(voltage_limit, _electricalAngle(shaft_angle,pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
void StepperMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  
  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts))
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts; 
  else
    shaft_angle = target_angle;
  
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(voltage_limit, _electricalAngle(shaft_angle,pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}