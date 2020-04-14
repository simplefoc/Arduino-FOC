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
  PI_velocity.timestamp = micros();
  PI_velocity.u_limit = -1;
  PI_velocity.uk_1 = 0;
  PI_velocity.ek_1 = 0;

  // Ultra slow velocity
  // PI contoroller
  PI_velocity_ultra_slow.K = DEF_PI_VEL_US_K;
  PI_velocity_ultra_slow.Ti = DEF_PI_VEL_US_TI;
  PI_velocity_ultra_slow.timestamp = micros();
  PI_velocity_ultra_slow.u_limit = -1;
  PI_velocity_ultra_slow.uk_1 = 0;
  PI_velocity_ultra_slow.ek_1 = 0;

  // position loop config
  // P controller constant
  P_angle.K = DEF_P_ANGLE_K;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;
  
  // driver deafault type
  driver = DriverType::half_bridge;


  // electric angle og the zero angle
  // electric angle of the index for encoder
  zero_electric_angle = 0;
}

// init hardware pins
void BLDCMotor::init() {
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  pinMode(enable_pin, OUTPUT);

  // Increase PWM frequency to 32 kHz
  // make silent
  setPwmFrequency(pwmA);
  setPwmFrequency(pwmB);
  setPwmFrequency(pwmC);

  // check if u_limit configuration has been done. 
  // if not set it to the power_supply_voltage
  if(PI_velocity.u_limit == -1) PI_velocity.u_limit = power_supply_voltage;
  if(PI_velocity_ultra_slow.u_limit == -1) PI_velocity_ultra_slow.u_limit = power_supply_voltage;

  delay(500);
  // enable motor
  enable();
  delay(500);
  
}

/*
	initialization function
*/
void BLDCMotor::initFOC() {
  // encoder alignment
  delay(500);
  alignEncoder();
  delay(500);
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
void BLDCMotor::alignEncoder() {
  setPwm(pwmA,12);
  setPwm(pwmB,0);
  setPwm(pwmC,0);
  delay(1000);
  encoder->setCounterZero();
  delay(500);
  setPhaseVoltage(0,0);
  delay(200);
  indexSearch();
  delay(500);
}
/*
	Encoder alignment to electrical 0 angle
*/
void BLDCMotor::indexSearch() {
  // if no index return
  if(!encoder->hasIndex()) return;
  // search the index with small speed
  while(!encoder->indexFound() && shaft_angle < 2 * M_PI){
    voltage_q = 3;
    loopFOC();    
  }
  setPhaseVoltage(0,0);

  encoder->setIndexZero();  
  zero_electric_angle = electricAngle(encoder->getIndexAngle());
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
  float angle = angle_el + M_PI/2.0 + zero_electric_angle;
  // Uq sign compensation
  angle = normalizeAngle(Uq > 0 ? angle :  angle + M_PI );
  // Park transform
  Ualpha = abs(Uq) * cos(angle);
  Ubeta = abs(Uq) * sin(angle);
  
  switch (driver) {
    case DriverType::full_bridge :
      // full Clarke transform
      Ua = Ualpha;
      Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta;
      Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta;
      break;
    case DriverType::half_bridge :
      // HMBGC & L6234
      // Unipolar Clarke transform
      // determine the segment I, II, III
      if ((angle >= 0) && (angle <= _120_D2R)) {
        // section I
        Ua = Ualpha + _1_SQRT3 * Ubeta;
        Ub = _2_SQRT3 * Ubeta;
        Uc = 0;
      } else if ((angle > _120_D2R) && (angle <= (2 * _120_D2R))) {
        // section III
        Ua = 0;
        Ub = _1_SQRT3 * Ubeta - Ualpha;
        Uc = -_1_SQRT3 * Ubeta - Ualpha;
      } else if ((angle > (2 * _120_D2R)) && (angle <= (3 * _120_D2R))) {
        // section II
        Ua = Ualpha - _1_SQRT3 * Ubeta;
        Ub = 0;
        Uc = - _2_SQRT3 * Ubeta;
      }
      break;
  }
  
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
  int U_max = power_supply_voltage;
  // uniploar or bipolar FOC
  switch (driver) {
    case DriverType::full_bridge :
      // sets the voltage [-U_max,U_max] to pwm [0,255]
      U_pwm = ((float)U + (float)U_max) / (2.0 * (float)U_max) * 255.0;
      break;
    case DriverType::half_bridge :
      // HMBGC
      // sets the voltage [0,12V(U_max)] to pwm [0,255]
      // - support for HMBGC controller
      // - support for L6234 drive
      U_pwm = (int)(255.0 * (float)U / (float)U_max);
      break;
  }
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
  float Ts = (micros() - PI_velocity.timestamp) * 1e-6;

  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = PI_velocity.uk_1;
  uk += PI_velocity.K * (Ts / (2.0 * PI_velocity.Ti) + 1.0) * ek + PI_velocity.K * (Ts / (2.0 * PI_velocity.Ti) - 1.0) * PI_velocity.ek_1;
  if (abs(uk) > PI_velocity.u_limit) uk = uk > 0 ? PI_velocity.u_limit : -PI_velocity.u_limit;

  PI_velocity.uk_1 = uk;
  PI_velocity.ek_1 = ek;
  PI_velocity.timestamp = micros();
  return uk;
}

// PI controller for ultra slow velocity control
float BLDCMotor::velocityUltraSlowPI(float vel) {
  float Ts = (micros() - PI_velocity_ultra_slow.timestamp) * 1e-6;
  static float angle;

  angle += vel * Ts;
  float ek = angle - shaft_angle;

  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = PI_velocity_ultra_slow.uk_1;
  uk += PI_velocity_ultra_slow.K * (Ts / (2 * PI_velocity_ultra_slow.Ti) + 1) * ek + PI_velocity_ultra_slow.K * (Ts / (2 * PI_velocity_ultra_slow.Ti) - 1) * PI_velocity_ultra_slow.ek_1;
  if (abs(uk) > PI_velocity_ultra_slow.u_limit) uk = uk > 0 ? PI_velocity_ultra_slow.u_limit : -PI_velocity_ultra_slow.u_limit;

  PI_velocity_ultra_slow.uk_1 = uk;
  PI_velocity_ultra_slow.ek_1 = ek;
  PI_velocity_ultra_slow.timestamp = micros();
  return uk;
}
// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.K * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
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





