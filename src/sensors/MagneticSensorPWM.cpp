#include "MagneticSensorPWM.h"
#include "Arduino.h"

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

    // define if the sensor uses interrupts
    is_interrupt_based = false;

    // define as not set
    last_call_us = _micros();
}


void MagneticSensorPWM::init(){

    // initial hardware
    pinMode(pinPWM, INPUT);
    raw_count = getRawCount();
    
    this->Sensor::init(); // call base class init
}

// get current angle (rad)
float MagneticSensorPWM::getSensorAngle(){
    // raw data from sensor
    raw_count = getRawCount();
    return( (float) (raw_count) / (float)cpr) * _2PI;
}


// read the raw counter of the magnetic sensor
int MagneticSensorPWM::getRawCount(){
    if (!is_interrupt_based){ // if it's not interrupt based read the value in a blocking way
        pulse_length_us = pulseIn(pinPWM, HIGH);
    }
    return pulse_length_us;
}


void MagneticSensorPWM::handlePWM() {
    //  unsigned long now_us = ticks();
    unsigned long now_us = _micros();

    // if falling edge, calculate the pulse length
    if (!digitalRead(pinPWM)) pulse_length_us = now_us - last_call_us;

    // save the currrent timestamp for the next call
    last_call_us = now_us;
    is_interrupt_based = true; // set the flag to true
}

// function enabling hardware interrupts of the for the callback provided
// if callback is not provided then the interrupt is not enabled
void MagneticSensorPWM::enableInterrupt(void (*doPWM)()){
    // declare it's interrupt based
    is_interrupt_based  = true;

    // enable interrupts on pwm input pin
    attachInterrupt(digitalPinToInterrupt(pinPWM), doPWM, CHANGE);
}