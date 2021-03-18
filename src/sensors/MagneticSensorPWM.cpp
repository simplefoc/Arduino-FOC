#include "MagneticSensorPWM.h"

/** MagneticSensorPWM(uint8_t _pinPWM, int _min, int _max)
 * @param _pinPWM  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading
 * @param _max_raw_count  the largest expected reading
 */
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pinPWM, int _min_raw_count, int _max_raw_count){

    pinPWM = _pinPWM;

    cpr = _max_raw_count - _min_raw_count;
    min_raw_count = _min_raw_count;
    max_raw_count = _max_raw_count;
    
    pinMode(pinPWM, INPUT);
}


void MagneticSensorPWM::init(){
    
    // velocity calculation init
    angle_prev = 0;
    velocity_calc_timestamp = _micros();

    // full rotations tracking number
    full_rotation_offset = 0;
    raw_count_prev = getRawCount();
}

// get current angle (rad) 
float MagneticSensorPWM::getAngle(){

    // raw data from sensor
    raw_count = getRawCount();
    
    int delta = raw_count - raw_count_prev;
    // if overflow happened track it as full rotation
    if(abs(delta) > (0.8*cpr) ) full_rotation_offset += delta > 0 ? -_2PI : _2PI;

    float angle = full_rotation_offset + ( (float) (raw_count) / (float)cpr) * _2PI;

    // calculate velocity here
    long now = _micros();
    float Ts = (now - velocity_calc_timestamp)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;
    velocity = (angle - angle_prev)/Ts;

    // save variables for future pass
    raw_count_prev = raw_count;
    angle_prev = angle;
    velocity_calc_timestamp = now;

    return angle;
}

//  get velocity (rad/s)
float MagneticSensorPWM::getVelocity(){
  return velocity;
}

// read the raw counter of the magnetic sensor
int MagneticSensorPWM::getRawCount(){
	return pulseIn(pinPWM,HIGH);
}