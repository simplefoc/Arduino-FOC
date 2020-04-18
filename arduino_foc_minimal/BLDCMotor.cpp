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

  // position loop config
  // P controller constant
  P_angle.K = DEF_P_ANGLE_K;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

  // electric angle og the zero angle
  // electric angle of the index for encoder
  index_electric_angle = 0;
  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_VELOCITY;

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
  if(PI_velocity.u_limit == -1 || PI_velocity.u_limit > power_supply_voltage/2) PI_velocity.u_limit = power_supply_voltage;
  if(PI_velocity_ultra_slow.u_limit == -1 || PI_velocity.u_limit > power_supply_voltage/2) PI_velocity_ultra_slow.u_limit = power_supply_voltage;

  delay(500);
  // enable motor
  enable();
  delay(500);
  
}

/*
	initialization function
*/
int  BLDCMotor::initFOC() {
  // encoder alignment
  delay(500);
  int exit_flag = alignEncoder();
  delay(500);
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
  setPwm(pwmA,12);
  setPwm(pwmB,0);
  setPwm(pwmC,0);
  delay(1000);
  // set encoder to zero
  encoder->setCounterZero();
  delay(500);
  setPhaseVoltage(0,0);
  delay(200);

  if(debugger) debugger->println("DEBUG: Search for the encoder index - if available.");
  // find the index if available
  int exit_flag = indexSearch();
  delay(500);
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
  // search the index with small speed
  while(!encoder->indexFound() && shaft_angle < 2 * M_PI){
    voltage_q = velocityPI(index_search_velocity - shaftVelocity());
    loopFOC();    
  }

  // set index to zero if it has been found
  if(encoder->indexFound()){
    encoder->setIndexZero();  
    // remember index electric angle
    index_electric_angle = electricAngle(encoder->getIndexAngle());
  }

  // disable motor
  setPhaseVoltage(0,0);

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
  return normalizeAngle(shaftAngle * pole_pairs);
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
    case ControlType::velocity_ultra_slow:
      // velocity set point
      shaft_velocity_sp = target;
      voltage_q = velocityUltraSlowPI(shaft_velocity_sp);
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
    case ControlType::voltage:
      voltage_q = target;
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
  
  // Uq sign compensation
  float angle = angle_el + index_electric_angle;
  // Uq sign compensation
  angle = normalizeAngle(Uq > 0 ? angle :  angle + M_PI );
  // Inverse park transform
  Ualpha = -sin(angle) * abs(Uq);
  Ubeta =  cos(angle) * abs(Uq);
  
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
*/
void BLDCMotor::setPwm(int pinPwm, float U) {
  int U_pwm = 0;
  float U_max = power_supply_voltage/2.0;

  // sets the voltage [-U_max,U_max] to pwm [0,255]
  U_pwm = ((float)U + (float)U_max) / (2.0 * (float)U_max) * 255.0;
     
  // limit the values between 0 and 255;
  U_pwm = U_pwm < 0 ? 0 : U_pwm;
  U_pwm = U_pwm >= 255 ? 255 : U_pwm;

  // write hardware pwm
  analogWrite(pinPwm, U_pwm);
}

/**
	Utility funcitons
*/
/*
	normalizing radian angle to [0,2PI]
*/
double BLDCMotor::normalizeAngle(double angle)
{
  double a = fmod(angle, 2 * M_PI);
  return a >= 0 ? a : (a + 2 * M_PI);
}




/**
	Motor control functions
*/
float BLDCMotor::velocityPI(float ek) {
  float Ts = (_micros() - PI_velocity.timestamp) * 1e-6;

  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = PI_velocity.uk_1;
  uk += PI_velocity.K * (Ts / (2.0 * PI_velocity.Ti) + 1.0) * ek + PI_velocity.K * (Ts / (2.0 * PI_velocity.Ti) - 1.0) * PI_velocity.ek_1;
  if (abs(uk) > PI_velocity.u_limit) uk = uk > 0 ? PI_velocity.u_limit : -PI_velocity.u_limit;

  PI_velocity.uk_1 = uk;
  PI_velocity.ek_1 = ek;
  PI_velocity.timestamp = _micros();
  return uk;
}

// PI controller for ultra slow velocity control
float BLDCMotor::velocityUltraSlowPI(float vel) {
  float Ts = (_micros() - PI_velocity_ultra_slow.timestamp) * 1e-6;
  static float angle;

  angle += vel * Ts;
  float ek = angle - shaft_angle;

  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = PI_velocity_ultra_slow.uk_1;
  uk += PI_velocity_ultra_slow.K * (Ts / (2 * PI_velocity_ultra_slow.Ti) + 1) * ek + PI_velocity_ultra_slow.K * (Ts / (2 * PI_velocity_ultra_slow.Ti) - 1) * PI_velocity_ultra_slow.ek_1;
  if (abs(uk) > PI_velocity_ultra_slow.u_limit) uk = uk > 0 ? PI_velocity_ultra_slow.u_limit : -PI_velocity_ultra_slow.u_limit;

  PI_velocity_ultra_slow.uk_1 = uk;
  PI_velocity_ultra_slow.ek_1 = ek;
  PI_velocity_ultra_slow.timestamp = _micros();
  return uk;
}
// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.K * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
}

void BLDCMotor::useDebugging(Print &print){
  debugger = &print; //operate on the adress of print
}


/*
  High PWM frequency
*/
void setPwmFrequency(int pin) {
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if (pin == 5 || pin == 6) {
      TCCR0B = ((TCCR0B & 0b11111000) | 0x01);
    } else {
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    }
  }
  else if (pin == 3 || pin == 11) {
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);
  }
}

