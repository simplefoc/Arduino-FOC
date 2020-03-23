#include "BLDCMotor.h"

/*
  BLDCMotorEncoder(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr)
  - phA, phB, phC - motor A,B,C phase pwm pins
  - *counter      - encoder counter variable
  - encA, encB    - encoder A and B pins
  - pp            - pole pair number
  - cpr           - counts per rotation number (cpm=ppm*4)
  - enable_pin pin    - (optional input)
*/
BLDCMotorEncoder::BLDCMotorEncoder(int phA, int phB, int phC, int pp, int encA, int encB , int cpr)
{
  // set enable_pin pin to zero
  BLDCMotorEncoder(phA, phB, phC, pp, encA, encB,  cpr, 0);
}
BLDCMotorEncoder::BLDCMotorEncoder(int phA, int phB, int phC, int pp, int encA, int encB , int cpr, int en)
{
  // Pin intialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin = en;

  // Encoder measurement structure init
  // hardware pins
  encoder.pinA = encA;
  encoder.pinB = encB;
  // counter setup
  encoder.pulse_counter = 0;
  encoder.pulse_timestamp = 0;
  encoder.cpr = cpr;
  encoder.A_active = 0;
  encoder.B_active = 0;


}
/*
	initialization function
*/
void BLDCMotorEncoder::init() {
  // enable_pin motor
  enable_pinMotor();
  // encoder alignment
  delay(500);
  alignEncoder();
  delay(500);
}

/*
	disable motor
*/
void BLDCMotorEncoder::disableMotor()
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
void BLDCMotorEncoder::enable_pinMotor()
{
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
  // enable_pin the driver - if enable_pin pin available
  if (enable_pin) digitalWrite(enable_pin, HIGH);
}




/*
	Encoder alignment to electrical 0 angle
*/
void BLDCMotorEncoder::alignEncoder() {
  setPhaseVoltage(10, M_PI / 2);
  delay(500);
  encoder.pulse_counter = 0;
  disableMotor();
}

/**
	State calculation methods
*/
/*
	Shaft angle calculation
*/
float BLDCMotorEncoder::shaftAngle() {
  return  (encoder.pulse_counter) / ((float)encoder.cpr) * (2.0 * M_PI);
}


/*
  Shaft velocity calculation
  funciton using mixed time and frequency measurement technique
*/
float BLDCMotorEncoder::shaftVelocity() {
  static float Th_1, pps;
  static long N, prev_timestamp_us;

  // timestamp
  long timestamp_us = micros();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6;
  // time from last impulse
  float Th = (timestamp_us - encoder.pulse_timestamp) * 1e-6;
  long dN = encoder.pulse_counter - N;

  // Pulse per second calculation (Eq.3.)
  // dN - impulses received
  // Ts - sampling time - time in between function calls
  // Th - time from last impulse
  // Th_1 - time form last impulse of the previous call
  // only increment if some impulses received
  pps = dN != 0 ? dN / (Ts + Th_1 - Th) : pps;

  // velocity calculation
  float velocity = pps / ((float)encoder.cpr) * (2.0 * M_PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  Th_1 = Th;
  N = encoder.pulse_counter;
  return velocity;
}

/*
	Electrical angle calculation
*/
float BLDCMotorEncoder::electricAngle(float shaftAngle) {
  return normalizeAngle(shaftAngle * pole_pairs);
}
/*
	Update motor angles and velocities
*/
void BLDCMotorEncoder::updateStates() {
  shaft_velocity = shaftVelocity();
  shaft_angle = shaftAngle();
}


/**
	FOC methods
*/
/*
	Method using FOC to set Uq to the motor at the optimal angle
*/
void BLDCMotorEncoder::setVoltage(float Uq) {
  setPhaseVoltage(Uq, electricAngle(shaft_angle));
}

void BLDCMotorEncoder::setPhaseVoltage(double Uq, double angle_el) {

  // Park transform
  Ualpha = Uq * cos(angle_el);
  Ubeta = Uq * sin(angle_el);
  // negative Uq compensation
  float angle = Uq > 0 ? angle_el : normalizeAngle( angle_el + M_PI );

  // determine the segment I, II, III
  if ((angle >= 0) && (angle <= _120_D2R)) {
    // section I
    Ua = Ualpha + _1_SQRT3 * abs(Ubeta);
    Ub = _2_SQRT3 * abs(Ubeta);
    Uc = 0;

  } else if ((angle > _120_D2R) && (angle <= (2 * _120_D2R))) {
    // section III
    Ua = 0;
    Ub = _1_SQRT3 * Ubeta + abs(Ualpha);
    Uc = -_1_SQRT3 * Ubeta + abs(Ualpha);

  } else if ((angle > (2 * _120_D2R)) && (angle <= (3 * _120_D2R))) {
    // section II
    Ua = Ualpha + _1_SQRT3 * abs(Ubeta);
    Ub = 0;
    Uc = _2_SQRT3 * abs(Ubeta);
  }

  // set phase voltages
  setPwm(pwmA, Ua);
  setPwm(pwmB, Ub);
  setPwm(pwmC, Uc);
}
/*
	Set voltage to the pwm pin
*/
void BLDCMotorEncoder::setPwm(int pinPwm, float U) {
  // dead-zone compensation
  float U_comp = U==0 ? 0 : sign(U)*U_DEAD + U;
  
  // sets the voltage [0,12V(U_MAX)] to pwm [0,255]
  // - U_MAX you can set in header file - default 12V
  // - support for HMBGC controller
  // int U_pwm =   255.0*U_comp/U_MAX;

  // sets the voltage [-U_MAX,U_MAX] to pwm [0,255]
  // - U_MAX you can set in header file - default 12V
  // - support for L6234 drive
  int U_pwm = (U_comp + U_MAX) / (2 * U_MAX) * 255;
  
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
double BLDCMotorEncoder::normalizeAngle(double angle)
{
  double a = fmod(angle, 2 * M_PI);
  return a >= 0 ? a : (a + 2 * M_PI);
}
/*
	Reference low pass filter
  used to filter set point signal - to remove sharp changes
*/
float BLDCMotorEncoder::filterLP(float u) {
  static float Ts, yk_1;
  float M_Tau = 0.01;
  Ts = (micros() - Ts) * 1e-6;
  float y_k = Ts / (M_Tau + Ts) * u + (1 - Ts / (M_Tau + Ts)) * yk_1;
  Ts = micros();
  yk_1 = y_k;
  return y_k;
}




/**
	Motor control functions
*/
float BLDCMotorEncoder::velocityPI(float ek) {
  static float ek_1, uk_1;
  static float dT;
  dT = (micros() - dT) * 1e-6;

  float uk = uk_1 + M_Kr * (dT / (2 * M_Ti) + 1) * ek + M_Kr * (dT / (2 * M_Ti) - 1) * ek_1;
  if (abs(uk) > U_MAX) uk = uk > 0 ? U_MAX : -U_MAX;

  uk_1 = uk;
  ek_1 = ek;
  dT = micros();
  return uk;
}

float BLDCMotorEncoder::positionP(float ek) {
  float uk = M_P * ek;
  if (abs(uk) > W_MAX) uk = uk > 0 ? W_MAX : -W_MAX;
  return uk;
}


void BLDCMotorEncoder::setVelocity(float vel) {
  setVoltage(velocityPI(vel - shaft_velocity));
}

void BLDCMotorEncoder::setPosition(float pos) {
  setVoltage(velocityPI(-positionP( filterLP(pos) - shaft_angle ) - shaft_velocity));
}




//  Encoder interrupt callback functions
//  enabling CPR=4xPPR behaviour
// A channel
void BLDCMotorEncoder::handleEncoderA() {
  int A = digitalRead(encoder.pinA);
  if ( A != encoder.A_active ) {
    encoder.pulse_counter += (encoder.A_active == encoder.B_active) ? 1 : -1;
    encoder.pulse_timestamp = micros();
    encoder.A_active = A;
  }
}
// B channel
void BLDCMotorEncoder::handleEncoderB() {
  int B = digitalRead(encoder.pinA);
  if ( B != encoder.B_active ) {
    encoder.pulse_counter += (encoder.A_active != encoder.B_active) ? 1 : -1;
    encoder.pulse_timestamp = micros();
    encoder.B_active = B;
  }
}

