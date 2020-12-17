#include "MagneticSensorAnalog.h"

/** MagneticSensorAnalog(uint8_t _pinAnalog, int _min, int _max)
 * @param _pinAnalog  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * @param _max_raw_count  the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note: For ESP32 (with 12bit ADC the value will be nearer 4096)
 */
MagneticSensorAnalog::MagneticSensorAnalog(uint8_t _pinAnalog, int _min_raw_count, int _max_raw_count){
  
  pinAnalog = _pinAnalog;

  cpr = _max_raw_count - _min_raw_count;
  min_raw_count = _min_raw_count;
  max_raw_count = _max_raw_count;

  if(pullup == Pullup::INTERN){
    pinMode(pinAnalog, INPUT_PULLUP);
  }else{
    pinMode(pinAnalog, INPUT);
  }

}


void MagneticSensorAnalog::init(){

	// velocity calculation init
	angle_prev = 0;
	velocity_calc_timestamp = _micros(); 

	// full rotations tracking number
	full_rotation_offset = 0;
	raw_count_prev = getRawCount();  
	zero_offset = 0;
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorAnalog::getAngle(){
  // raw data from the sensor
  raw_count = getRawCount(); 

  int delta = raw_count - raw_count_prev;
  // if overflow happened track it as full rotation
  if(abs(delta) > (0.8*cpr) ) full_rotation_offset += delta > 0 ? -_2PI : _2PI; 

  float angle = natural_direction * (full_rotation_offset + ( (float) (raw_count - zero_offset) / (float)cpr) * _2PI);

  // calculate velocity here 
  long now = _micros();
  float Ts = ( now - velocity_calc_timestamp)*1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 
  velocity = (angle - angle_prev)/Ts;

  // save variables for future pass
  raw_count_prev = raw_count;
  angle_prev = angle;
  velocity_calc_timestamp = now;

  return angle;
}

// Shaft velocity calculation
float MagneticSensorAnalog::getVelocity(){
  // TODO: Refactor?: to avoid angle being called twice, velocity is pre-calculted during getAngle
  return velocity;
}

// set current angle as zero angle 
// return the angle [rad] difference
float MagneticSensorAnalog::initRelativeZero(){
  
  float angle_offset = -getAngle();
  zero_offset = natural_direction * getRawCount();

  // angle tracking variables
  full_rotation_offset = 0;
  return angle_offset;
}
// set absolute zero angle as zero angle
// return the angle [rad] difference
float MagneticSensorAnalog::initAbsoluteZero(){
  float rotation = -(int)zero_offset;
  // init absolute zero
  zero_offset = 0;

  // angle tracking variables
  full_rotation_offset = 0;
  // return offset in radians
  return rotation / (float)cpr * _2PI;
}
// returns 0 if it has no absolute 0 measurement
// 0 - incremental encoder without index
// 1 - encoder with index & magnetic sensors
int MagneticSensorAnalog::hasAbsoluteZero(){
  return 1;
}
// returns 0 if it does need search for absolute zero
// 0 - magnetic sensor 
// 1 - ecoder with index
int MagneticSensorAnalog::needsAbsoluteZeroSearch(){
  return 0;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorAnalog::getRawCount(){
	return analogRead(pinAnalog);
}
