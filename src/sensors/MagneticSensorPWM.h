#ifndef MAGNETICSENSORPWM_LIB_H
#define MAGNETICSENSORPWM_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// This sensor has been tested with AS5048a running in PWM mode.

class MagneticSensorPWM: public Sensor{
 public:
  /**
    * MagneticSensorPWM class constructor
    * @param _pinPWM the pin to read the PWM sensor input signal
    */
    MagneticSensorPWM(uint8_t _pinPWM,int _min = 0, int _max = 0);


    // initialize the sensor hardware
    void init();

    int pinPWM;

    // get current angle (rad)
    float getSensorAngle() override;
  
    // pwm handler
    void handlePWM();
    void enableInterrupt(void (*doPWM)());
    unsigned long pulse_length_us;

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
    

};

#endif
