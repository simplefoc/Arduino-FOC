#include "BLDCMotor.h"

/*
BLDCMotorint(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr)
- phA, phB, phC - motor A,B,C phase pwm pins
- *counter      - encoder counter variable
- encA, encB    - encoder A and B pins
- pp            - pole pair number
- cpr           - counts per rotation number (cpm=ppm*4)
- enable pin    - (optional input)
*/
BLDCMotor::BLDCMotor(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr)
{
  // set enable pin to zero
	BLDCMotor(phA, phB, phC, counter, encA, encB, pp, cpr, 0);
}
BLDCMotor::BLDCMotor(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr, int en)
{
  // Pin intialization
  _phA = phA;
  _phB = phB;
  _phC = phC;

  // enable pin
  _en = en;

  // encoder pins
  _encA = encA;
  _encB = encB;
  
  // Encoder value
  _encoderPosition = counter;
  _cpr = cpr;
  _pp = pp;

}
/*
	initialization function
*/
void BLDCMotor::init(){
  // enable motor
  enableMotor();
	// encoder alignment
	delay(500);
	alignEncoder();
	delay(500);
}

/*
	disable motor
*/
void BLDCMotor::disableMotor()
{
  // disable the driver - if enable pin available
  if(_en) digitalWrite(_en, LOW);
  // set zero to PWM
  setPwm(_phA,0);
  setPwm(_phB,0);
  setPwm(_phC,0);  
}
/*
 disable motor
*/
void BLDCMotor::enableMotor()
{ 
  // set zero to PWM
  setPwm(_phA,0);
  setPwm(_phB,0);
  setPwm(_phC,0);  
  // enable the driver - if enable pin available
  if(_en) digitalWrite(_en, HIGH);
}




/*
	Encoder alignment to electrical 0 angle
*/
void BLDCMotor::alignEncoder(){
  setPhaseVoltage(10,M_PI/2);
  delay(500);
  *_encoderPosition = 0;
  disableMotor();
}

/**
	State calculation methods
*/
/*
	Shaft angle calculation
*/
float BLDCMotor::ShaftAngle(){
	return (*_encoderPosition)/((float)_cpr)*(2.0*M_PI);
}
/*
	Shaft velocity calculation
*/
float BLDCMotor::ShaftVelocity(float shaftAngle){
	static float dt; 
	static float shaftAngle_1; 
  
	dt = (micros() - dt)*1e-6;
	float d_angle = -(shaftAngle - shaftAngle_1)/(dt);
	dt = micros();
	shaftAngle_1 = shaftAngle;
	return d_angle;
}
/*
	Electrical angle calculation
*/
float BLDCMotor::ElectricAngle(float shaftAngle){
	return normalizeAngle(shaftAngle*_pp);
}
/*
	Update motor angles and velocities
*/
void BLDCMotor::updateStates(){
	_shaftAngle = ShaftAngle();
	_angleElec = ElectricAngle(_shaftAngle);
	_shaftVel = ShaftVelocity(_shaftAngle);
}


/**
	FOC methods 
*/
/*
	Method using FOC to set Uq to the motor at the optimal angle
*/
void BLDCMotor::setVoltage(float Uq){
	updateStates();
	setPhaseVoltage(Uq, _angleElec);
}
void BLDCMotor::setPhaseVoltage(double Uq, double angle_el){

	// Park transform
	Ualpha = Uq*cos(angle_el);
	Ubeta = Uq*sin(angle_el);
	// negative Uq compensation
	float angle = Uq > 0 ? angle_el : normalizeAngle( angle_el + M_PI );

	// determine the segment I, II, III
	if((angle >= 0)&&(angle <= _120_D2R)){ 
		// section I
		Ua = Ualpha + _1_SQRT3*abs(Ubeta);
		Ub = _2_SQRT3*abs(Ubeta);
		Uc = 0;

	}else if((angle > _120_D2R)&&(angle <= (2*_120_D2R))){ 
		// section III
		Ua = 0;
		Ub = _1_SQRT3*Ubeta + abs(Ualpha);
		Uc = -_1_SQRT3*Ubeta + abs(Ualpha);

	}else if((angle > (2*_120_D2R))&&(angle <= (3*_120_D2R))){ 
		// section II
		Ua = Ualpha + _1_SQRT3*abs(Ubeta);
		Ub = 0;
		Uc = _2_SQRT3*abs(Ubeta);
	}

	// set phase voltages
	setPwm(_phA,Ua);
	setPwm(_phB,Ub);
	setPwm(_phC,Uc);  
}
/*
	Set voltage to the pwm pin
*/
void BLDCMotor::setPwm(int pinPwm, float U){
	int U_pwm =   U <= U_MAX ? 255.0*U/U_MAX : 255;
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
*/
float BLDCMotor::filterLP(float u){
  static float dt,yk_1; 
  float M_Tau = 0.01;
  dt = (micros() - dt)*1e-6;
  float y_k = dt/(M_Tau+dt)*u +(1-dt/(M_Tau+dt))*yk_1;
  dt = micros();
  yk_1 = y_k;
  return y_k;
}




/**
	Motor control functions
*/
float BLDCMotor::velocityPI(float ek){
  static float ek_1, uk_1;
  static float dT;
  dT = (micros() - dT)*1e-6;
  
  float uk = uk_1 + M_Kr*(dT/(2*M_Ti)+1)*ek + M_Kr*(dT/(2*M_Ti)-1)*ek_1;
  if(abs(uk) > U_MAX) uk = uk > 0 ? U_MAX : -U_MAX;
  
  uk_1 = uk;
  ek_1 = ek;
  dT = micros();
  return uk;
}

float BLDCMotor::positionP(float ek){
  float uk = M_P * ek;
  if(abs(uk) > W_MAX) uk = uk > 0 ? W_MAX : -W_MAX;
  return uk;
}


void BLDCMotor::setVelocity(float vel){
  updateStates();
  setVoltage(velocityPI(vel-_shaftVel));
}

void BLDCMotor::setPosition(float pos){
  updateStates();
  setVoltage(velocityPI(-positionP( filterLP(pos) - _shaftAngle ) - _shaftVel));
}



