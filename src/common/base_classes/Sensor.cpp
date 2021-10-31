#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"

// TODO add an init method to make the startup smoother by initializing internal variables to current values rather than 0

void Sensor::update() {
    float val = getSensorAngle();
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}


 /** get current angular velocity (rad/s) */
float Sensor::getVelocity() {
    // calculate sample time
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0) Ts = 1e-3f;
    // velocity calculation
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // save variables for future pass
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}

void Sensor::init() {
    // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
    getSensorAngle(); // call once
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); // call again
    vel_angle_prev_ts = _micros();
    delay(1);
    getSensorAngle(); // call once
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); // call again
    angle_prev_ts = _micros();
}


float Sensor::getMechanicalAngle() {
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
