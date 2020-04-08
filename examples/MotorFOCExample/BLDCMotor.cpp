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

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.K = 0.4;
  PI_velocity.Ti = 0.005;
  PI_velocity.timestamp = micros();

  // Ultra slow velocity
  // PI contoroller
  PI_velocity_ultra_slow.K = 120.0;
  PI_velocity_ultra_slow.Ti = 100.0;
  PI_velocity_ultra_slow.timestamp = micros();

  // position loop config
  // P controller constant
  P_angle.K = 15;

  // Power supply woltage
  U_max = 12;
  // maximum angular velocity to be used for positioning 
  velocity_max = 20;
}

// init hardware pins
void BLDCMotor::init(DriverType type) {
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

  driver_type = type;

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
  setPhaseVoltage(12, M_PI / 2);
  delay(1000);
  encoder->setCounterZero();
  setPhaseVoltage(0, M_PI / 2);
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
	Update motor angles and set thr Uq voltage
*/
void BLDCMotor::loopFOC() {
  // voltage open loop loop
  shaft_angle = shaftAngle();
  setPhaseVoltage(voltage_q, electricAngle(shaft_angle));
}

/*
  Update motor angles and velocities
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
void BLDCMotor::setPhaseVoltageUnipolar(double Uq, double angle_el) {

  // Uq sign compensation
  float angle = Uq > 0 ? angle_el : normalizeAngle( angle_el + M_PI );
  // Park transform
  Ualpha = abs(Uq) * cos(angle);
  Ubeta = abs(Uq) * sin(angle);

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

  // set phase voltages
  setPwm(pwmA, Ua);
  setPwm(pwmB, Ub);
  setPwm(pwmC, Uc);
}
/*
  Method using FOC to set Uq to the motor at the optimal angle
*/
void BLDCMotor::setPhaseVoltageBipolar(double Uq, double angle_el) {

  // Uq sign compensation
  float angle;// = angle + shaft_velocity*0.0003;
  angle = Uq > 0 ? angle_el : normalizeAngle( angle_el + M_PI );

  // Park transform
  Ualpha = abs(Uq) * cos(angle);
  Ubeta = abs(Uq) * sin(angle);

  // determine the segment I, II, III
  // section I
  Ua = Ualpha;
  Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta;
  Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta;

  // set phase voltages
  setPwm(pwmA, Ua);
  setPwm(pwmB, Ub);
  setPwm(pwmC, Uc);
}

/*
  Method using FOC to set Uq to the motor at the optimal angle
*/
void BLDCMotor::setPhaseVoltage(double Uq, double angle_el) {
  switch (driver_type) {
    case DriverType::bipolar :
      // L6234
      setPhaseVoltageBipolar(Uq, angle_el);
      break;
    case DriverType::unipolar :
      // HMBGC
      setPhaseVoltageUnipolar(Uq, angle_el);
      break;
  }
}

/*
	Set voltage to the pwm pin
*/
void BLDCMotor::setPwm(int pinPwm, float U) {
  int U_pwm = 0;
  // uniploar or bipolar FOC
  switch (driver_type) {
    case DriverType::bipolar :
      // sets the voltage [-U_max,U_max] to pwm [0,255]
      // - U_max you can set in header file - default 12V
      // - support for L6234 drive
      U_pwm = ((float)U + (float)U_max) / (2.0 * (float)U_max) * 255.0;
      break;
    case DriverType::unipolar :
      // HMBGC
      // sets the voltage [0,12V(U_max)] to pwm [0,255]
      // - U_max you can set in header file - default 12V
      // - support for HMBGC controller
      U_pwm = 255.0 * (float)U / (float)U_max;
      break;
  }
  // limit the values between 0 and 255;
  U_pwm = U_pwm < 0 ? 0 : U_pwm;
  U_pwm = U_pwm > 255 ? 255 : U_pwm;

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
/*
	Reference low pass filter
  used to filter set point signal - to remove sharp changes
*/
float BLDCMotor::filterLP(float u) {
  static float Ts, yk_1;
  float M_Tau = 0.01;
  Ts = (micros() - Ts) * 1e-6;
  float y_k = Ts / (M_Tau + Ts) * u + (1 - Ts / (M_Tau + Ts)) * yk_1;
  Ts = micros();
  yk_1 = y_k;
  return y_k;
}




/**
	Motor `ntrol functions
*/
float BLDCMotor::velocityPI(float ek) {
  float Ts = (micros() - PI_velocity.timestamp) * 1e-6;

  // u(s) = Kr(1 + 1/(Ti.s))
  float uk = PI_velocity.uk_1;
  uk += PI_velocity.K * (Ts / (2 * PI_velocity.Ti) + 1) * ek + PI_velocity.K * (Ts / (2 * PI_velocity.Ti) - 1) * PI_velocity.ek_1;
  if (abs(uk) > U_max) uk = uk > 0 ? U_max : -U_max;

  PI_velocity.uk_1 = uk;
  PI_velocity.ek_1 = ek;
  PI_velocity.timestamp = micros();
  return -uk;
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
  if (abs(uk) > U_max) uk = uk > 0 ? U_max : -U_max;

  PI_velocity_ultra_slow.uk_1 = uk;
  PI_velocity_ultra_slow.ek_1 = ek;
  PI_velocity_ultra_slow.timestamp = micros();
  return -uk;
}
// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.K * ek;
  if (abs(velk) > velocity_max) velk = velk > 0 ? velocity_max : -velocity_max;
  return velk;
}

// voltage control loop api
void BLDCMotor::setVoltage(float Uq) {
  voltage_q = Uq;
}
// shaft velocity loop api
void BLDCMotor::setVelocity(float vel) {
  shaft_velocity_sp = vel;
}

// utra slow shaft velocity loop api
void BLDCMotor::setVelocityUltraSlow(float vel) {
  shaft_velocity_sp = vel;

  shaft_velocity = shaftVelocity();
  static long timestamp;
  static float angle_1;
  if (!timestamp) {
    // init
    timestamp = micros();
    angle_1 = shaft_angle;
  }
  float dt = (micros() - timestamp) / 1e6;
  angle_1 += vel * dt;
  voltage_q = velocityUltraSlowPI(angle_1 - shaft_angle);
  timestamp = micros();
}

// postion control loop
void BLDCMotor::setPosition(float pos) {
  shaft_angle_sp = pos;
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





