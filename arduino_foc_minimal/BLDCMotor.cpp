#include "BLDCMotor.h"


/*
  BLDCMotor( int phA, int phB, int phC, int pp, int encA, int encB , int cpr, int en)
  - phA, phB, phC - motor A,B,C phase pwm pins
  - pp            - pole pair number
  - encA, encB    - encoder A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
  - enable pin    - (optional input)
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
  power_supply_voltage = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.K = DEF_PI_VEL_K;
  PI_velocity.Ti = DEF_PI_VEL_TI;
  PI_velocity.timestamp = _micros();
  PI_velocity.u_limit = -1;
  PI_velocity.uk_1 = 0;
  PI_velocity.ek_1 = 0;

  // Ultra slow velocity
  // PI contoroller
  PI_velocity_ultra_slow.K = DEF_PI_VEL_US_K;
  PI_velocity_ultra_slow.Ti = DEF_PI_VEL_US_TI;
  PI_velocity_ultra_slow.timestamp = _micros();
  PI_velocity_ultra_slow.u_limit = -1;
  PI_velocity_ultra_slow.uk_1 = 0;
  PI_velocity_ultra_slow.ek_1 = 0;
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
  PI_velocity_index_search.u_limit = -1;
  PI_velocity_index_search.timestamp = _micros();
  PI_velocity_index_search.uk_1 = 0;
  PI_velocity_index_search.ek_1 = 0;
  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_TARGET_VELOCITY;

  // electric angle og the zero angle
  // electric angle of the index for encoder
  index_electric_angle = 0;

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

  // check if u_limit configuration has been done. 
  // if not set it to the power_supply_voltage
  // or if set too high
  if(PI_velocity.u_limit == -1 || PI_velocity.u_limit > power_supply_voltage/2) PI_velocity.u_limit = power_supply_voltage/2;
  if(PI_velocity_ultra_slow.u_limit == -1 || PI_velocity.u_limit > power_supply_voltage/2) PI_velocity_ultra_slow.u_limit = power_supply_voltage/2;
  if(PI_velocity_index_search.u_limit == -1 || PI_velocity_index_search.u_limit > power_supply_voltage/2) PI_velocity_index_search.u_limit = power_supply_voltage/2;

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
  // encoder alignment
  _delay(500);
  int exit_flag = alignEncoder();
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

void BLDCMotor::linkEncoder(Encoder* enc) {
  encoder = enc;
}


/*
	Encoder alignment to electrical 0 angle
*/
int BLDCMotor::alignEncoder() {
  if(debugger) debugger->println("DEBUG: Align the encoder and motor electrical 0 angle.");
  // align the electircal phases of the motor and encoder
  setPwm(pwmA, power_supply_voltage/2.0);
  setPwm(pwmB,0);
  setPwm(pwmC,0);
  _delay(1000);
  // set encoder to zero
  encoder->setCounterZero();
  _delay(500);
  setPhaseVoltage(0,0);
  _delay(200);

  // find the index if available
  int exit_flag = indexSearch();
  _delay(500);
  if(debugger){
    if(exit_flag< 0 ) debugger->println("DEBUG: Error: Index not found!");
    if(exit_flag> 0 ) debugger->println("DEBUG: Success: Index found!");
  }
  return exit_flag;
}


/*
	Encoder alignment to electrical 0 angle
*/
int BLDCMotor::indexSearch() {
  // if no index return
  if(!encoder->hasIndex()) return 0;
  
  if(debugger) debugger->println("DEBUG: Search for the encoder index.");

  // search the index with small speed
  while(!encoder->indexFound() && shaft_angle < _2PI){
    voltage_q = velocityIndexSearchPI(index_search_velocity - shaftVelocity());
    loopFOC();   
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // set index to zero if it has been found
  if(encoder->indexFound()){
    encoder->setIndexZero();  
    // remember index electric angle
    index_electric_angle = electricAngle(encoder->getIndexAngle());
  }
  // return bool is index found
  return encoder->indexFound() ? 1 : -1;
}

/**
	State calculation methods
*/
// shaft angle calculation
float BLDCMotor::shaftAngle() {
  return encoder->getAngle();
}
// shaft velocity calculation
float BLDCMotor::shaftVelocity() {
  return encoder->getVelocity();
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
void BLDCMotor::setPhaseVoltage(double Uq, double angle_el) {
  
  // Inverse park transform
  Ualpha =  -sin(angle_el + index_electric_angle) * Uq;
  Ubeta =  cos(angle_el + index_electric_angle) * Uq;
  
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
  float U_max = power_supply_voltage/2.0;
  
  // sets the voltage [-U_max,U_max] to pwm [0,255]
  // u_pwm = 255 * (U + U_max)/(2*U_max)
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
double BLDCMotor::normalizeAngle(double angle){
  double a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}




/**
	Motor control functions
*/
// PI controller function
float BLDCMotor::controllerPI(float ek, PI_s& cont){

  float Ts = (_micros() - cont.timestamp) * 1e-6;
  
  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = cont.uk_1;
  uk += cont.K * (Ts / (2.0 * cont.Ti) + 1.0) * ek + cont.K * (Ts / (2.0 * cont.Ti) - 1.0) * cont.ek_1;
  // antiwindup - limit the output voltage
  if (abs(uk) > cont.u_limit) uk = uk > 0 ? cont.u_limit : -cont.u_limit;

  cont.uk_1 = uk;
  cont.ek_1 = ek;
  cont.timestamp = _micros();
  return uk;
}
// velocity control loop PI controller
float BLDCMotor::velocityPI(float ek) {
  return controllerPI(ek, PI_velocity);
}
// index search PI contoller
float BLDCMotor::velocityIndexSearchPI(float ek) {
  return controllerPI(ek, PI_velocity_index_search);
}
// PI controller for ultra slow velocity control
float BLDCMotor::velocityUltraSlowPI(float vel) {
  float Ts = (_micros() - PI_velocity_ultra_slow.timestamp) * 1e-6;

  // integrate the velocity to get the necessary angle
  ultraslow_estimated_angle += vel * Ts;
  // error of positioning
  float ek = ultraslow_estimated_angle - shaft_angle;

  return controllerPI(ek, PI_velocity_ultra_slow);
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