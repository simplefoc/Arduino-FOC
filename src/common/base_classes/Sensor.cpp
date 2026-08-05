#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"



void Sensor::update() {
    angle_type val = getSensorAngle();
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
        return; // TODO signal error, e.g. via a flag and counter
    angle_prev_ts = _micros();
    setAngleContinuous(val);
}


 /** get current angular velocity (rad/s) */
float Sensor::getVelocity() {
    // calculate sample time
    // if timestamps were unsigned, we could get rid of this section, unsigned overflow handles it correctly
    #ifdef INTEGER_ANGLE
    angle_type Ts = angle_prev_ts - vel_angle_prev_ts;
    #else
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6f;
    #endif
    #if 0
    if (Ts < 0.0f) {    // handle micros() overflow - we need to reset vel_angle_prev_ts
        vel_angle_prev = angle_prev;
        #ifndef INTEGER_ANGLE
        vel_full_rotations = full_rotations;
        #endif
        vel_angle_prev_ts = angle_prev_ts;
        return velocity;
    }
    #endif
    if (Ts < min_elapsed_time) return velocity; // don't update velocity if deltaT is too small

    // Calculate change in angle. Handles `full_rotations` integer wrap-arounds,
    // and avoids float precision loss issues by keeping numbers small.
    angle_type delta_angle = angle_prev - vel_angle_prev;
    #ifndef INTEGER_ANGLE
    const int32_t delta_full_rotations = full_rotations - vel_full_rotations;
    if (delta_full_rotations) {
        delta_angle += delta_full_rotations * _2PI;
    }
    #endif

    #ifdef INTEGER_ANGLE
    if (abs(delta_angle) > 0)
    {
        velocity = delta_angle * 1000000 * _2PI / (Ts * steps_per_revolution);
    }
    else
    {
        velocity = 0.0f;
    }
    
    #else
    // floating point equality checks are bad, so instead we check that the angle change is very small
    if (fabsf(delta_angle) > 1e-8f) {
        velocity = delta_angle / Ts;
    } else {
        velocity = 0.0f;
    }
    #endif

    // Always advance the velocity reference sample to avoid stale deltas/time windows.
    vel_angle_prev = angle_prev;
    #ifndef INTEGER_ANGLE
    vel_full_rotations = full_rotations;
    #endif
    vel_angle_prev_ts = angle_prev_ts;
    
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


angle_type Sensor::getMechanicalAngle() {
    return angle_prev;
}



float Sensor::getAngle(){
    #ifdef INTEGER_ANGLE
    return angle_prev / (float)steps_per_revolution * _2PI;
    #else
    return (float)full_rotations * _2PI + angle_prev;
    #endif
}



double Sensor::getPreciseAngle() {
    #ifdef INTEGER_ANGLE
    return angle_prev / (double)steps_per_revolution * _2PI;
    #else
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
    #endif
}



int32_t Sensor::getFullRotations() {
    #ifdef INTEGER_ANGLE
    return angle_prev/steps_per_revolution;
    #else
    return full_rotations;
    #endif
}



int Sensor::needsSearch() {
    return 0; // default false
}

void Sensor::setAngleContinuous(angle_type sensor_angle)
{
    #ifdef INTEGER_ANGLE
    angle_type d_angle = sensor_angle - sensor_angle_prev;
    if(abs(d_angle) > (steps_per_revolution/2) ) 
    {
        d_angle += d_angle > 0 ? -steps_per_revolution : steps_per_revolution;
    }
    angle_prev += d_angle;
    sensor_angle_prev = sensor_angle;
    #else
    angle_type d_angle = sensor_angle - angle_prev;
    if(abs(d_angle) > (0.8f*_2PI) ) 
    {
        full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    }
    angle_prev = sensor_angle;
    #endif

}