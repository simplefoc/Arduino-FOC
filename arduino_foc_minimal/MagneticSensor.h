#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#include "Arduino.h"
#include <SPI.h>
#include "FOCutils.h"
#include "Sensor.h"

#define DEF_ANGLE_REGISTAR 0x3FFF

class MagneticSensor: public Sensor{
 public:
    

    // MagneticSensor(int cs, float _cpr, int _angle_register)
    //  cs              - SPI chip select pin 
    //  _cpr            - counts per revolution 
    // _angle_register  - (optional) angle read register - default 0x3FFF
    MagneticSensor(int cs, float _cpr, int angle_register = 0);
    
    // SPI angle register to read
    int angle_register;
    // encoder initialise pins
    void init();

    // implementation of abstract functions of the Sensor class
    // get current angle (rad)
    float getAngle();
    // get current angular velocity (rad/s)
    float getVelocity();
    // set current agle as zero angle 
    // return the angle [rad] difference
    float initRelativeZero();
    // set absoule zero angle as zero angle
    // return the angle [rad] difference
    float initAbsoluteZero();
    // returns 1 because it is the absolute sensor
    int hasAbsoluteZero();
    // returns 0  maning it doesn't need search for absolute zero
    int needsAbsoluteZeroSearch();
    

  private:
    // spi variables
    int chip_select_pin;
	  bool errorFlag;
	  SPISettings settings;
    // spi funcitons
    void close();
    word read(word angle_register);
    byte spiCalcEvenParity(word value);


    // zero offset
    word zero_offset;
    // funciton getting current register value
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset;
    float angle_data_prev;
    // impulse cpr
    float cpr;                 
    // velocity calculation variables
    float angle_prev;
    long velocity_calc_timestamp;

};


#endif
