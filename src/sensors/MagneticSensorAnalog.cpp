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
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorAnalog::getAngle(){
  // raw data from the sensor
  raw_count = getRawCount(); 

  int delta = raw_count - raw_count_prev;
  // if overflow happened track it as full rotation
  if(abs(delta) > (0.8*cpr) ) full_rotation_offset += delta > 0 ? -_2PI : _2PI; 
  
  float angle = full_rotation_offset + ( (float) (raw_count) / (float)cpr) * _2PI;

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

// function reading the raw counter of the magnetic sensor
int MagneticSensorAnalog::getRawCount(){
	return analogRead(pinAnalog);
}
