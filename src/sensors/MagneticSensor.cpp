#include "MagneticSensor.h"
#include <limits.h>
#include "../common/hardware_specific/samd_mcu.h"
MagneticSensor::MagneticSensor(uint32_t cpr) 
: cpr(cpr)
{

}


void MagneticSensor::init(){
    
    // velocity calculation init
    angle_prev = 0;
    // full rotations tracking number
    full_rotation_count = 0;
}

void MagneticSensor::updateFullRotationCount()
{
    int delta = raw_count - raw_count_prev;
    // if overflow happened track it as full rotation
    if( abs(delta) > (cpr*4/5) ) 
        full_rotation_count += delta > 0 ? -cpr : cpr;

}

// get current angle (rad) 
float MagneticSensor::getAngle(){

    // raw data from sensor
    raw_count_prev = raw_count;
    uint64_t timestamp_tmp;
    raw_count = getRawCount(timestamp_tmp);
    
    if(timestamp_tmp == timestamp_us) //no new measurement
        return angle_prev;
        
    timestamp_us_prev = timestamp_us;
    timestamp_us = timestamp_tmp;

    updateFullRotationCount();

    angle_prev = angle;
    angle = ( (float) (full_rotation_count + (int)raw_count) / (float)cpr) * _2PI;

    return angle;
}

//  get velocity (rad/s)
float MagneticSensor::getVelocity()
{
    if(timestamp_us == timestamp_us_prev)
        return 0.0f;

    unsigned long delta = (timestamp_us < timestamp_us_prev) ? ULONG_MAX - timestamp_us_prev + timestamp_us : timestamp_us - timestamp_us_prev;
    return (angle - angle_prev)/(delta * 1e-6f);
}
