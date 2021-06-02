#ifndef MAGNETICSENSORANALOG_LIB_H
#define MAGNETICSENSORANALOG_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/**
 * This sensor has been tested with AS5600 running in 'analog mode'.  This is where output pin of AS6000 is connected to an analog pin on your microcontroller.
 * This approach is very simple but you may more accurate results with MagneticSensorI2C if that is also supported as its skips the DAC -> ADC conversion (ADC supports MagneticSensorI2C)
 */
class MagneticSensorAnalog: public Sensor{
 public:
    /**
     * MagneticSensorAnalog class constructor
     * @param _pinAnalog  the pin to read the PWM signal
     */
    MagneticSensorAnalog(uint8_t _pinAnalog, int _min = 0, int _max = 0);
    

    /** sensor initialise pins */
    void init();

    int pinAnalog; //!< encoder hardware pin A
    
    // Encoder configuration
    Pullup pullup;
    
    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;
 
    /** raw count (typically in range of 0-1023), useful for debugging resolution issues */
    int raw_count;

  private:
    int min_raw_count;
    int max_raw_count;
    int cpr;
    int read();

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    int raw_count_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev; //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp
    float velocity;
    

};


#endif
