#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"



void Sensor::update() {
    float val = getSensorAngle();
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
        return; // TODO signal error, e.g. via a flag and counter
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}


 /** get current angular velocity (rad/s) */
float Sensor::getVelocity() {
    // calculate sample time
    // if timestamps were unsigned, we could get rid of this section, unsigned overflow handles it correctly
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6f;
    if (Ts < 0.0f) {    // handle micros() overflow - we need to reset vel_angle_prev_ts
        vel_angle_prev = angle_prev;
        vel_full_rotations = full_rotations;
        vel_angle_prev_ts = angle_prev_ts;
        return velocity;
    }
    if (Ts < min_elapsed_time) return velocity; // don't update velocity if deltaT is too small

    float current_angle = 0.0f;
    float prev_angle = 0.0f;
    // Avoid floating point precision loss for large full_rotations
    // this is likely optional
    if (full_rotations == vel_full_rotations) {
        current_angle = angle_prev;
        prev_angle = vel_angle_prev;
    } else {
        current_angle = (float) full_rotations * _2PI + angle_prev;
        prev_angle = (float) vel_full_rotations * _2PI + vel_angle_prev;
    }
    const float delta_angle = current_angle - prev_angle;

    // floating point equality checks are bad, so instead we check that the angle change is very small
    if (fabsf(delta_angle) > 1e-8f) {
        velocity = delta_angle / Ts;

        vel_angle_prev = angle_prev;
        vel_full_rotations = full_rotations;
        vel_angle_prev_ts = angle_prev_ts;
    }
    
    return velocity;
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
