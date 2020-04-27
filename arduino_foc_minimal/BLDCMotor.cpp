#include "BLDCMotor.h"

/*
  BLDCMotor( int phA, int phB, int phC, int pp , int cpr, int en)
  - phA, phB, phC - motor A,B,C phase pwm pins
  - pp            - pole pair number
  - cpr           - counts per rotation number (cpm=ppm*4)
  - enable pin    - (optionl input)
*/
BLDCMotor::BLDCMotor(int phA, int phB, int phC, int pp, int en)
{
  // Pin intialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin = en;

  // Power supply woltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.K = DEF_PI_VEL_K;
  PI_velocity.Ti = DEF_PI_VEL_TI;
  PI_velocity.timestamp = _micros();
  PI_velocity.voltage_limit = voltage_power_supply/2;
  PI_velocity.voltage_ramp = DEF_PI_VEL_U_RAMP;
  PI_velocity.voltage_prev = 0;
  PI_velocity.tracking_error_prev = 0;

  // Ultra slow velocity
  // PI contoroller
  PI_velocity_ultra_slow.K = DEF_PI_VEL_US_K;
  PI_velocity_ultra_slow.Ti = DEF_PI_VEL_US_TI;
  PI_velocity_ultra_slow.timestamp = _micros();
  PI_velocity_ultra_slow.voltage_limit = voltage_power_supply/2;
  PI_velocity_ultra_slow.voltage_ramp = DEF_PI_VEL_US_U_RAMP;
  PI_velocity_ultra_slow.voltage_prev = 0;
  PI_velocity_ultra_slow.tracking_error_prev = 0;
  // current estimated angle
  ultraslow_estimated_angle = 0;

  // position loop config
  // P controller constant
  P_angle.K = DEF_P_ANGLE_K;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

  // index search PI controller
  PI_velocity_index_search.K = DEF_PI_VEL_INDEX_K;
  PI_velocity_index_search.Ti = DEF_PI_VEL_INDEX_TI;
  PI_velocity_index_search.voltage_limit = voltage_power_supply/2;
  PI_velocity_index_search.voltage_ramp = DEF_PI_VEL_INDEX_U_RAMP;
  PI_velocity_index_search.timestamp = _micros();
  PI_velocity_index_search.voltage_prev = 0;
  PI_velocity_index_search.tracking_error_prev = 0;

  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_TARGET_VELOCITY;

  // electric angle og the zero angle
  // electric angle of the index for encoder
  zero_electric_angle = 0;

  //debugger 
  debugger = nullptr;
}

// init hardware pins   
void BLDCMotor::init() {
  if(debugger) debugger->println("DEBUG: Initilaise the motor pins.");
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  pinMode(enable_pin, OUTPUT);

  if(debugger) debugger->println("DEBUG: Set high frequency PWM.");
  // Increase PWM frequency to 32 kHz
  // make silent
  setPwmFrequency(pwmA);
  setPwmFrequency(pwmB);
  setPwmFrequency(pwmC);

  // sanity check for the voltage limit configuration
  if(PI_velocity.voltage_limit > voltage_power_supply/2) PI_velocity.voltage_limit = PI_velocity.voltage_limit > voltage_power_supply/2;
  if(PI_velocity.voltage_limit > voltage_power_supply/2) PI_velocity_ultra_slow.voltage_limit = voltage_power_supply/2;
  if(PI_velocity_index_search.voltage_limit > voltage_power_supply/2) PI_velocity_index_search.voltage_limit = voltage_power_supply/2;

  _delay(500);
  // enable motor
  if(debugger) debugger->println("DEBUG: Enabling motor.");
  enable();
  _delay(500);
  
}

/*
	initialization function
*/
int  BLDCMotor::initFOC() {
  // sensor and motor alignment
  _delay(500);
  int exit_flag = alignSensor();
  _delay(500);
  return exit_flag;
}

/*
	disable motor
*/
void BLDCMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if (enable_pin) digitalWrite(enable_pin, LOW);
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
}
/*
  disable motor
*/
void BLDCMotor::enable()
{
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
  // enable_pin the driver - if enable_pin pin available
  if (enable_pin) digitalWrite(enable_pin, HIGH);

}

void BLDCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}


/*
	Encoder alignment to electrical 0 angle
*/
int BLDCMotor::alignSensor() {
  if(debugger) debugger->println("DEBUG: Align the sensor's and motor electrical 0 angle.");
  // align the electircal phases of the motor and sensor
  setPwm(pwmA, voltage_power_supply/2.0);
  setPwm(pwmB,0);
  setPwm(pwmC,0);
  _delay(1000);
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
    else  debugger->println("DEBUG: Success: Absolute zero not availabe!");
  }
  return exit_flag;
}


/*
	Encoder alignment to electrical 0 angle
*/
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

  // align absoulute zero if it has been found
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
  return sensor->getVelocity();
}
/*
	Electrical angle calculation
*/
float BLDCMotor::electricAngle(float shaftAngle) {
  //return normalizeAngle(shaftAngle * pole_pairs);
  return (shaftAngle * pole_pairs);
}
/*
	Iterative function looping FOC algorithm, setting Uq on the Motor
  The faster it can be run the better
*/
void BLDCMotor::loopFOC() {
  // voltage open loop loop
  shaft_angle = shaftAngle();
  setPhaseVoltage(voltage_q, electricAngle(shaft_angle));
}

/*
  Iterative funciton running outer loop of the FOC algorithm
  Bahvior of this funciton is determined by the motor.controller variable
  It runs either angle, veloctiy, velocity ultra slow or voltage loop
  - needs to be called iteratively it is asynchronious function
*/
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
      // inlcude velocity loop
      shaft_velocity_sp = target;
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity_ultra_slow:
      // velocity set point
      shaft_velocity_sp = target;
      voltage_q = velocityUltraSlowPI(shaft_velocity_sp);
      break;
  }
}


/**
	FOC methods
*/
/*
  Method using FOC to set Uq to the motor at the optimal angle
*/
void BLDCMotor::setPhaseVoltage(float Uq, float angle_el) {

  // angle normalisation in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation funcitons
  float angle = normalizeAngle(angle_el + zero_electric_angle);
  // Inverse park transform
  // regular sin + cos ~300us    (no memeory usaage)
  // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
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


/*
	Set voltage to the pwm pin
  - function a bit optimised to get better performance
*/
void BLDCMotor::setPwm(int pinPwm, float U) {
  // max value
  float U_max = voltage_power_supply/2.0;
  
  // sets the voltage [-U_max,U_max] to pwm [0,255]
  // u_pwm = 255 * (U + U_max)/(2*U_max)
  // it can be further optimised 
  // (example U_max = 6 > U_pwm = 127.5 + 21.25*U)
  int U_pwm = 127.5 * (U/U_max + 1);
     
  // limit the values between 0 and 255
  U_pwm = (U_pwm < 0) ? 0 : (U_pwm >= 255) ? 255 : U_pwm;

  analogWrite(pinPwm, U_pwm);
}

/**
	Utility funcitons
*/
/*
	normalizing radian angle to [0,2PI]
*/
float BLDCMotor::normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}




/**
	Motor control functions
*/
// PI controller function
float BLDCMotor::controllerPI(float tracking_error, PI_s& cont){
  float Ts = (_micros() - cont.timestamp) * 1e-6;
  if(Ts > 0.5) Ts = 1e-3;

  // u(s) = Kr(1 + 1/(Ti.s))
  // tustin discretisation of the PI controller ( a bit optimised )
  // uk = uk_1  +  K*(Ts/(2*Ti) + 1)*ek + K*(Ts/(2*Ti)-1)*ek_1
  float tmp = 0.5 * Ts / cont.Ti;
  float voltage = cont.voltage_prev + cont.K * ((tmp + 1.0) * tracking_error + (tmp - 1.0) * cont.tracking_error_prev);

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
// PI controller for ultra slow velocity control
float BLDCMotor::velocityUltraSlowPI(float vel) {
  float Ts = (_micros() - PI_velocity_ultra_slow.timestamp) * 1e-6;

  // integrate the velocity to get the necessary angle
  ultraslow_estimated_angle += vel * Ts;
  // error of positioning
  float tracking_error = ultraslow_estimated_angle - shaft_angle;

  return controllerPI(tracking_error, PI_velocity_ultra_slow);
}

// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.K * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
}

/**
 *  Debugger functions
 */
// funciton implementing the debugger setter
void BLDCMotor::useDebugging(Print &print){
  debugger = &print; //operate on the adress of print
  if(debugger )debugger->println("DEBUG: Serial debugger enabled!");
}