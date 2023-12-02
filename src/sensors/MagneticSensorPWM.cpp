#include "MagneticSensorPWM.h"
#include "Arduino.h"

/** MagneticSensorPWM(uint8_t _pinPWM, int _min, int _max)
 * @param _pinPWM  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading
 * @param _max_raw_count  the largest expected reading
 */
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pinPWM, int _min_raw_count, int _max_raw_count){

    pinPWM = _pinPWM;

    cpr = _max_raw_count - _min_raw_count + 1;
    min_raw_count = _min_raw_count;
    max_raw_count = _max_raw_count;

    // define if the sensor uses interrupts
    is_interrupt_based = false;

    // define as not set
    last_call_us = _micros();
}


/** MagneticSensorPWM(uint8_t _pinPWM, int freqHz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks)
 * 
 * Constructor that computes the min and max raw counts based on the PWM frequency and the number of PWM clocks in one period
 * 
 * @param _pinPWM  the pin that is reading the pwm from magnetic sensor
 * @param freqHz  the frequency of the PWM signal, in Hz, e.g. 115, 230, 460 or 920 for the AS5600, depending on the PWM frequency setting
 * @param _total_pwm_clocks  the total number of PWM clocks in one period, e.g. 4351 for the AS5600
 * @param _min_pwm_clocks  the 0 value returned by the sensor, in PWM clocks, e.g. 128 for the AS5600
 * @param _max_pwm_clocks  the largest value returned by the sensor, in PWM clocks, e.g. 4223 for the AS5600
 */
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pinPWM, int freqHz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks){

    pinPWM = _pinPWM;

    min_raw_count = lroundf(1000000.0f/freqHz/_total_pwm_clocks*_min_pwm_clocks);
    max_raw_count = lroundf(1000000.0f/freqHz/_total_pwm_clocks*_max_pwm_clocks);
    cpr = max_raw_count - min_raw_count + 1;

    // define if the sensor uses interrupts
    is_interrupt_based = false;

    min_elapsed_time = 1.0f/freqHz; // set the minimum time between two readings

    // define as not set
    last_call_us = _micros();
}



void MagneticSensorPWM::init(){

    // initial hardware
    pinMode(pinPWM, INPUT);
    raw_count = getRawCount();
    pulse_timestamp = _micros();
    
    this->Sensor::init(); // call base class init
}

// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void MagneticSensorPWM::update() {
  if (is_interrupt_based)
    noInterrupts();
  Sensor::update();
  angle_prev_ts = pulse_timestamp; // Timestamp of actual sample, before the time-consuming PWM communication
  if (is_interrupt_based)
    interrupts();
}

// get current angle (rad)
float MagneticSensorPWM::getSensorAngle(){
    // raw data from sensor
    raw_count = getRawCount();
    if (raw_count > max_raw_count) raw_count = max_raw_count;
    if (raw_count < min_raw_count) raw_count = min_raw_count;
    return( (float) (raw_count - min_raw_count) / (float)cpr) * _2PI;
}


// read the raw counter of the magnetic sensor
int MagneticSensorPWM::getRawCount(){
    if (!is_interrupt_based){ // if it's not interrupt based read the value in a blocking way
        pulse_timestamp = _micros(); // ideally this should be done right at the rising edge of the pulse
        pulse_length_us = pulseIn(pinPWM, HIGH, timeout_us); // 1200us timeout, should this be configurable?
    }
    return pulse_length_us;
}


void MagneticSensorPWM::handlePWM() {
    //  unsigned long now_us = ticks();
    unsigned long now_us = _micros();

    // if falling edge, calculate the pulse length
    if (!digitalRead(pinPWM)) {
        pulse_length_us = now_us - last_call_us;
        pulse_timestamp = last_call_us; // angle was sampled at the rising edge of the pulse, so use that timestamp
    }

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