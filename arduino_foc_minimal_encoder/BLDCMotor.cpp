#include "BLDCMotor.h"

// BLDCMotor( int phA, int phB, int phC, int pp, int cpr, int en)
// - phA, phB, phC - motor A,B,C phase pwm pins
// - pp            - pole pair number
// - cpr           - counts per rotation number (cpm=ppm*4)
// - enable pin    - (optional input)
BLDCMotor::BLDCMotor(int phA, int phB, int phC, int pp, int en)
{
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin = en;

  // Power supply voltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.P = DEF_PI_VEL_P;
  PI_velocity.I = DEF_PI_VEL_I;
  PI_velocity.timestamp = _micros();
  PI_velocity.voltage_limit = voltage_power_supply/2;
  PI_velocity.voltage_ramp = DEF_PI_VEL_U_RAMP;
  PI_velocity.voltage_prev = 0;
  PI_velocity.tracking_error_prev = 0;

  // velocity low pass filter 
  LPF_velocity.Tf = DEF_VEL_FILTER_Tf;
  LPF_velocity.timestamp = _micros();
  LPF_velocity.prev = 0;  

  // position loop config
  // P controller constant
  P_angle.P = DEF_P_ANGLE_P;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

  // index search PI controller
  PI_velocity_index_search.P = DEF_PI_VEL_INDEX_P;
  PI_velocity_index_search.I = DEF_PI_VEL_INDEX_I;
  PI_velocity_index_search.voltage_limit = voltage_power_supply/2;
  PI_velocity_index_search.voltage_ramp = DEF_PI_VEL_INDEX_U_RAMP;
  PI_velocity_index_search.timestamp = _micros();
  PI_velocity_index_search.voltage_prev = 0;
  PI_velocity_index_search.tracking_error_prev = 0;

  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_TARGET_VELOCITY;

  // electric angle of the zero angle
  zero_electric_angle = 0;

  //debugger 
  debugger = nullptr;
}

// init hardware pins   
void BLDCMotor::init() {
  if(debugger) debugger->println("DEBUG: Initialize the motor pins.");
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if(hasEnable()) pinMode(enable_pin, OUTPUT);

  if(debugger) debugger->println("DEBUG: Set high frequency PWM.");
  // Increase PWM frequency to 32 kHz
  // make silent
  setPwmFrequency(pwmA);
  setPwmFrequency(pwmB);
  setPwmFrequency(pwmC);

  // sanity check for the voltage limit configuration
  if(PI_velocity.voltage_limit > voltage_power_supply/2) PI_velocity.voltage_limit =  voltage_power_supply/2;
  if(PI_velocity_index_search.voltage_limit > voltage_power_supply/2) PI_velocity_index_search.voltage_limit = voltage_power_supply/2;

  _delay(500);
  // enable motor
  if(debugger) debugger->println("DEBUG: Enabling motor.");
  enable();
  _delay(500);
  
}


// disable motor driver
void BLDCMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if (hasEnable()) digitalWrite(enable_pin, LOW);
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
}
// enable motor driver
void BLDCMotor::enable()
{
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
  // enable_pin the driver - if enable_pin pin available
  if (hasEnable()) digitalWrite(enable_pin, HIGH);

}

void BLDCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}


// Encoder alignment to electrical 0 angle
int BLDCMotor::alignSensor() {
  if(debugger) debugger->println("DEBUG: Align the sensor's and motor electrical 0 angle.");
  // align the electrical phases of the motor and sensor
  setPwm(pwmA, voltage_power_supply/2.0);
  setPwm(pwmB,0);
  setPwm(pwmC,0);
  _delay(3000);
  // set sensor to zero
  sensor->initRelativeZero();
  _delay(500);
  setPhaseVoltage(0,0);
  _delay(200);

  // find the index if available
  int exit_flag = absoluteZeroAlign();
  _delay(500);
  if(debugger){
    if(exit_flag< 0 ) debugger->println("DEBUG: Error: Absolute zero not found!");
    if(exit_flag> 0 ) debugger->println("DEBUG: Success: Absolute zero found!");
    else  debugger->println("DEBUG: Absolute zero not availabe!");
  }
  return exit_flag;
}


// Encoder alignment the absolute zero angle 
// - to the index
int BLDCMotor::absoluteZeroAlign() {
  // if no absolute zero return
  if(!sensor->hasAbsoluteZero()) return 0;
  
  if(debugger) debugger->println("DEBUG: Aligning the absolute zero.");

  if(debugger && sensor->needsAbsoluteZeroSearch()) debugger->println("DEBUG: Searching for absolute zero.");
  // search the absolute zero with small velocity
  while(sensor->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();   
    voltage_q = velocityIndexSearchPI(index_search_velocity - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // align absolute zero if it has been found
  if(!sensor->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = sensor->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = electricAngle(zero_offset);
  }
  // return bool is zero found
  return !sensor->needsAbsoluteZeroSearch() ? 1 : -1;
}

/**
	State calculation methods
*/
// shaft angle calculation
float BLDCMotor::shaftAngle() {
  return sensor->getAngle();
}
// shaft velocity calculation
float BLDCMotor::shaftVelocity() {
  float Ts = (_micros() - LPF_velocity.timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 
  // calculate the filtering 
  float alpha = LPF_velocity.Tf/(LPF_velocity.Tf + Ts);
  float vel = alpha*LPF_velocity.prev + (1-alpha)*sensor->getVelocity();
  // save the variables
  LPF_velocity.prev = vel;
  LPF_velocity.timestamp = _micros();
  return vel;
}
// Electrical angle calculation
float BLDCMotor::electricAngle(float shaftAngle) {
  //return normalizeAngle(shaftAngle * pole_pairs);
  return (shaftAngle * pole_pairs);
}

/**
  FOC functions
*/

// FOC initialization function
int  BLDCMotor::initFOC() {
  // sensor and motor alignment
  _delay(500);
  int exit_flag = alignSensor();
  _delay(500);
  return exit_flag;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  // shaft angle 
  shaft_angle = shaftAngle();
  // set the phase voltage - FOC heart function :) 
  setPhaseVoltage(voltage_q, electricAngle(shaft_angle));
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
void BLDCMotor::move(float target) {
  // get angular velocity
  shaft_velocity = shaftVelocity();
  switch (controller) {
    case ControlType::voltage:
      voltage_q = target;
      break;
    case ControlType::angle:
      // angle set point
      // include angle loop
      shaft_angle_sp = target;
      shaft_velocity_sp = positionP( shaft_angle_sp - shaft_angle );
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // include velocity loop
      shaft_velocity_sp = target;
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
  }
}


// Method using FOC to set Uq to the motor at the optimal angle
void BLDCMotor::setPhaseVoltage(float Uq, float angle_el) {

  // angle normalisation in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation functions
  float angle = normalizeAngle(angle_el + zero_electric_angle);
  // Inverse park transform
  // regular sin + cos ~300us    (no memory usaage)
  // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
  // Ualpha =  -_sin(angle) * Uq;  // -sin(angle) * Uq;
  // Ubeta =  _cos(angle) * Uq;    //  cos(angle) * Uq;
  Ualpha =  -_sin(angle) * Uq;  // -sin(angle) * Uq;
  Ubeta =  _cos(angle) * Uq;    //  cos(angle) * Uq;
  
  // Clarke transform
  Ua = Ualpha;
  Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta;
  Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta;
  
  // set phase voltages
  setPwm(pwmA, Ua);
  setPwm(pwmB, Ub);
  setPwm(pwmC, Uc);
}


// Set voltage to the pwm pin
// - function a bit optimized to get better performance
void BLDCMotor::setPwm(int pinPwm, float U) {
  // max value
  float U_max = voltage_power_supply/2.0;
  
  // sets the voltage [-U_max,U_max] to pwm [0,255]
  // u_pwm = 255 * (U + U_max)/(2*U_max)
  // it can be further optimized 
  // (example U_max = 6 > U_pwm = 127.5 + 21.25*U)
  int U_pwm = 127.5 * (U/U_max + 1);
     
  // limit the values between 0 and 255
  U_pwm = (U_pwm < 0) ? 0 : (U_pwm >= 255) ? 255 : U_pwm;

  analogWrite(pinPwm, U_pwm);
}

/**
	Utility functions
*/
// normalizing radian angle to [0,2PI]
float BLDCMotor::normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}
// determining if the enable pin has been provided
int BLDCMotor::hasEnable(){
  return enable_pin != 0;
}


/**
	Motor control functions
*/
// PI controller function
float BLDCMotor::controllerPI(float tracking_error, PI_s& cont){
  float Ts = (_micros() - cont.timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // u(s) = (P + I/s)e(s)
  // Tustin transform of the PI controller ( a bit optimized )
  // uk = uk_1  + (I*Ts/2 + P)*ek + (I*Ts/2 - P)*ek_1
  float tmp = cont.I*Ts*0.5;
  float voltage = cont.voltage_prev + (tmp + cont.P) * tracking_error + (tmp - cont.P) * cont.tracking_error_prev;

  // antiwindup - limit the output voltage_q
  if (abs(voltage) > cont.voltage_limit) voltage = voltage > 0 ? cont.voltage_limit : -cont.voltage_limit;
  // limit the acceleration by ramping the the voltage
  float d_voltage = voltage - cont.voltage_prev;
  if (abs(d_voltage)/Ts > cont.voltage_ramp) voltage = d_voltage > 0 ? cont.voltage_prev + cont.voltage_ramp*Ts : cont.voltage_prev - cont.voltage_ramp*Ts;

  cont.voltage_prev = voltage;
  cont.tracking_error_prev = tracking_error;
  cont.timestamp = _micros();
  return voltage;
}
// velocity control loop PI controller
float BLDCMotor::velocityPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity);
}
// index search PI contoller
float BLDCMotor::velocityIndexSearchPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity_index_search);
}
// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.P * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
}

/**
 *  Debugger functions
 */
// function implementing the debugger setter
void BLDCMotor::useDebugging(Print &print){
  debugger = &print; //operate on the address of print
  if(debugger )debugger->println("DEBUG: Serial debugger enabled!");
}
// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void BLDCMotor::monitor() {
  if(!debugger) return;
  switch (controller) {
    case ControlType::velocity:
      debugger->print(voltage_q);
      debugger->print("\t");
      debugger->print(shaft_velocity_sp);
      debugger->print("\t");
      debugger->println(shaft_velocity);
      break;
    case ControlType::angle:
      debugger->print(voltage_q);
      debugger->print("\t");
      debugger->print(shaft_angle_sp);
      debugger->print("\t");
      debugger->println(shaft_angle);
      break;
    case ControlType::voltage:
      debugger->print(voltage_q);
      debugger->print("\t");
      debugger->print(shaft_angle);
      debugger->print("\t");
      debugger->println(shaft_velocity);
      break;
  }
}