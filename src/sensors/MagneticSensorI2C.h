#ifndef MAGNETICSENSORI2C_LIB_H
#define MAGNETICSENSORI2C_LIB_H

#include "Arduino.h"
#include <Wire.h>
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

struct MagneticSensorI2CConfig_s  {
  uint8_t chip_address;
  uint8_t bit_resolution;
  uint8_t angle_register;
  uint8_t msb_mask;
  uint8_t msb_shift;
  uint8_t lsb_mask;
  uint8_t lsb_shift;
};

// some predefined structures
extern MagneticSensorI2CConfig_s AS5600_I2C, AS5048_I2C, MT6701_I2C;

#if defined(TARGET_RP2040)
#define SDA I2C_SDA
#define SCL I2C_SCL
#endif


class MagneticSensorI2C: public Sensor{
 public:
    /**
     * MagneticSensorI2C class constructor
     * @param chip_address  I2C chip address
     * @param bits number of bits of the sensor resolution 
     * @param angle_register_msb  angle read register msb
     * @param _bits_used_msb number of used bits in msb
     */
    MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used, bool lsb_right_aligned = true);

    /**
     * MagneticSensorI2C class constructor
     * @param config  I2C config
     */
    MagneticSensorI2C(MagneticSensorI2CConfig_s config);

    static MagneticSensorI2C AS5600();
        
    /** sensor initialise pins */
    void init(TwoWire* _wire = &Wire);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    /** experimental function to check and fix SDA locked LOW issues */
    int checkBus(byte sda_pin , byte scl_pin );

    /** current error code from Wire endTransmission() call **/
    uint8_t currWireError = 0;

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    MagneticSensorI2CConfig_s _conf;

    // I2C functions
    /** Read one I2C register value */
    int read(uint8_t angle_register_msb);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
    
    /* the two wire instance for this sensor */
    TwoWire* wire;
};


#endif
