#ifndef MAGNETICSENSORPWM_LIB_H
#define MAGNETICSENSORPWM_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// This sensor has been tested with AS5048a running in PWM mode.

class MagneticSensorPWM: public Sensor{
 public:
   /** MagneticSensorPWM(uint8_t _pinPWM, int _min, int _max)
    * @param _pinPWM  the pin that is reading the pwm from magnetic sensor
    * @param _min_raw_count  the smallest expected reading
    * @param _max_raw_count  the largest expected reading
    */
    MagneticSensorPWM(uint8_t _pinPWM,int _min = 0, int _max = 0);
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
    MagneticSensorPWM(uint8_t _pinPWM, int freqHz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks);

    // initialize the sensor hardware
    void init();

    int pinPWM;
    
    // Interrupt-safe update
    void update() override;

    // get current angle (rad)
    float getSensorAngle() override;
  
    // pwm handler
    void handlePWM();
    void enableInterrupt(void (*doPWM)());
    unsigned long pulse_length_us;

    unsigned int timeout_us = 1200;

  private:
    // raw count (typically in range of 0-1023)
    int raw_count;
    int min_raw_count;
    int max_raw_count;
    int cpr;

    // flag saying if the readings are interrupt based or not
    bool is_interrupt_based;

    int read();

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // time tracking variables
    unsigned long last_call_us;
    // unsigned long pulse_length_us;
    unsigned long pulse_timestamp;
    

};

#endif
