#ifndef SENSOR_H
#define SENSOR_H

#include <inttypes.h>

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
    USE_INTERN, //!< Use internal pullups
    USE_EXTERN //!< Use external pullups
};

/**
 * Sensor abstract class defintion
 * 
 * This class is purposefully kept simple, as a base for all kinds of sensors. Currently we have
 * Encoders, Magnetic Encoders and Hall Sensor implementations. This base class extracts the
 * most basic common features so that a FOC driver can obtain the data it needs for operation.
 * 
 * To implement your own sensors, create a sub-class of this class, and implement the getAngle()
 * method. getAngle() returns a float value, in radians, representing the current shaft angle in the
 * range 0 to 2*PI (one full turn). 
 * 
 * To function correctly, the sensor class getAngle() method has to be called sufficiently quickly. Normally,
 * the BLDCMotor's loopFOC() function calls it once per iteration, so you must ensure to call loopFOC() quickly
 * enough, both for correct motor and sensor operation.
 * 
 * The Sensor base class provides an implementation of getVelocity(), and takes care of counting full
 * revolutions in a precise way, but if you wish you can additionally override these methods to provide more
 * optimal implementations for your hardware.
 * 
 */
class Sensor{
    public:
        /** 
         * Get current shaft angle, as a float in radians, in the range 0 to 2PI.
         */
        virtual float getAngle();
        /** 
         * Get current shaft angle, as a float in radians, in the range 0 to 2PI.
         * This method is pure virtual and must be implemented in subclasses.
         * Calling this method directly does not update the base-class internal fields.
         * Use getAngle() when calling from other code.
         */
        virtual float getShaftAngle()=0;
        /** 
         * Get current angular velocity (rad/s)
         * Can be overridden in subclasses. Base implementation uses the values 
         * previously returned by getShaftAngle().
         */
        virtual float getVelocity();

        /**
         * Get current position (in rad) including full rotations and shaft angle
         * Note that this value has limited precision as the number of rotations increases,
         * as the limited precision of float can't capture the large angle of the full rotations
         * and the small angle of the shaft angle at the same time.
         */
        virtual float getPosition();

        /** 
         * On architectures supporting it, this will return a double precision position value
         */
        virtual double getPrecisePosition();

        /**
         * Get the number of full rotations
         */
        virtual int32_t getFullRotations();

        /** 
         * returns 0 if it does need search for absolute zero
         * 0 - magnetic sensor (& encoder with index which is found)
         * 1 - ecoder with index (with index not found yet)
         */
        virtual int needsSearch();
    protected:
        // velocity calculation variables
        float angle_prev=0; // result of last call to getAngle, used for full rotations and velocity
        long angle_prev_ts=0; // timestamp of last call to getAngle, used for velocity
        float vel_angle_prev=0; // angle at last call to getVelocity, used for velocity
        long vel_angle_prev_ts=0; //!< last velocity calculation timestamp
        int32_t full_rotations=0; // full rotation tracking
};

#endif
