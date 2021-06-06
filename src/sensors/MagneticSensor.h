#pragma once
#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"


class MagneticSensor: public Sensor
{
  public:
    MagneticSensor(uint32_t cpr);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;

    virtual void init();

    virtual uint32_t getRawCount(uint64_t & timestamp_us) = 0;

  private:

    virtual void updateFullRotationCount();

    uint32_t cpr; //!< Maximum range of the magnetic sensor

    // total angle tracking variables
    int32_t full_rotation_count; //!<number of full rotations made
    uint32_t raw_count, raw_count_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle, angle_prev; //!< angle in previous velocity calculation step
    uint64_t timestamp_us, timestamp_us_prev; //!< last velocity calculation timestamp
    
};
