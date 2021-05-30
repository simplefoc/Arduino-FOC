#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"



float Sensor::updateSensor() {
    float val = getSensorAngle();
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return getAngle();
}


 /** get current angular velocity (rad/s)*/
float Sensor::getVelocity() {
    // calculate sample time
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;
    // velocity calculation
    float vel = (angle_prev - vel_angle_prev)/Ts;    
    // save variables for future pass
    vel_angle_prev = angle_prev;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}


float Sensor::getShaftAngle() {
    return angle_prev;
}



float Sensor::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}



double Sensor::getPreciseAngle() {
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
}



int32_t Sensor::getFullRotations() {
    return full_rotations;
}



int Sensor::needsSearch() {
    return 0; // default false
}