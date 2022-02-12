#ifndef GENERIC_SENSOR_LIB_H
#define GENERIC_SENSOR_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/base_classes/Sensor.h"


class GenericSensor: public Sensor{
 public:
    /**
    GenericSensor class constructor
     * @param readCallback pointer to the function reading the sensor angle
     * @param initCallback pointer to the function initialising the sensor
    */
    GenericSensor(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr);
  
    float (*readCallback)() = nullptr; //!< function pointer to sensor reading
    void (*initCallback)() = nullptr; //!< function pointer to sensor initialisation

    void init() override;

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getSensorAngle() override;

};


#endif
