#ifndef MAGNETICSENSORSPI_LIB_H
#define MAGNETICSENSORSPI_LIB_H

#include "Arduino.h"
#include <SPI.h>
#include "FOCutils.h"
#include "Sensor.h"

#define DEF_ANGLE_REGISTAR 0x3FFF

class MagneticSensorSPI: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     * @param cs  SPI chip select pin 
     * @param bit_resolution   sensor resolution bit number
     * @param angle_register  (optional) angle read register - default 0x3FFF
     */
    MagneticSensorSPI(int cs, float bit_resolution, int angle_register = 0);
    

    /** sensor initialise pins */
    void init();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle();
    /** get current angular velocity (rad/s) **/
    float getVelocity();
    /**
     *  set current angle as zero angle 
     * return the angle [rad] difference
     */
    float initRelativeZero();
    /**
     *  set absolute zero angle as zero angle
     * return the angle [rad] difference
     */
    float initAbsoluteZero();
    /** returns 1 because it is the absolute sensor */
    int hasAbsoluteZero();
    /** returns 0  maning it doesn't need search for absolute zero */
    int needsAbsoluteZeroSearch();
    

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    int angle_register; //!< SPI angle register to read
    int chip_select_pin; //!< SPI chip select pin
	  SPISettings settings; //!< SPI settings variable
    // spi functions
    /** Stop SPI communication */
    void close(); 
    /** Read one SPI register value */
    word read(word angle_register);
    /** Calculate parity value  */
    byte spiCalcEvenParity(word value);


    word zero_offset; //!< user defined zero offset
    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    float angle_data_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev; //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp

};


#endif
