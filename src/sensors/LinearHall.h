#ifndef LINEAR_HALL_SENSOR_LIB_H
#define LINEAR_HALL_SENSOR_LIB_H

#include <SimpleFOC.h>

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
void ReadLinearHalls(int hallA, int hallB, int *a, int *b);

/**
 * This sensor class is for two linear hall effect sensors such as 49E, which are
 * positioned 90 electrical degrees apart (if one is centered on a rotor magnet,
 * the other is half way between rotor magnets).
 * It can also be used for a single magnet mounted to the motor shaft (set pp to 1).
 *
 * For more information, see this forum thread and PDF
 * https://community.simplefoc.com/t/40-cent-magnetic-angle-sensing-technique/1959
 * https://gist.github.com/nanoparticle/00030ea27c59649edbed84f0a957ebe1
 */
class LinearHall: public Sensor{
  public:
    LinearHall(int hallA, int hallB, int pp);

    void init(int centerA, int centerB); // Initialize without moving motor
    void init(class FOCMotor *motor); // Move motor to find center values

    // Get current shaft angle from the sensor hardware, and
    // return it as a float in radians, in the range 0 to 2PI.
    //  - This method is pure virtual and must be implemented in subclasses.
    //    Calling this method directly does not update the base-class internal fields.
    //    Use update() when calling from outside code.
    float getSensorAngle() override;

    int centerA;
    int centerB;
    int lastA, lastB;

  private:
    int pinA;
    int pinB;
    int pp;
    int electrical_rev;
    float prev_reading;
};

#endif
