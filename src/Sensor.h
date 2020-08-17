#ifndef SENSOR_H
#define SENSOR_H

/**
 *  Direction structure
 */
enum Direction{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
};

/**
 *  Pullup configuration structure
 */
enum Pullup{
    INTERN, //!< Use internal pullups
    EXTERN //!< Use external pullups
};

/**
 *  Sensor abstract class defintion
 * Each sensor needs to have these functions implemented
 */
class Sensor{
    public:
        /** get current angle (rad) */
        virtual float getAngle();
        /** get current angular velocity (rad/s)*/
        virtual float getVelocity();
        /**
         *  set current angle as zero angle 
         *  return the angle [rad] difference
         */
        virtual float initRelativeZero();
        /**
         * set absolute zero angle as zero angle
         * return the angle [rad] difference
         */
        virtual float initAbsoluteZero();

        // if natural_direction == Direction::CCW then direction will be flipped to CW
        int natural_direction = Direction::CW;

        /** 
         * returns 0 if it has no absolute 0 measurement
         * 0 - incremental encoder without index
         * 1 - encoder with index & magnetic sensors
         */
        virtual int hasAbsoluteZero();
        /** 
         * returns 0 if it does need search for absolute zero
         * 0 - magnetic sensor (& encoder with index which is found)
         * 1 - ecoder with index (with index not found yet)
         */
        virtual int needsAbsoluteZeroSearch();
};

#endif