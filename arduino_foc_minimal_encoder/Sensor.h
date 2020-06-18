#ifndef SENSOR_H
#define SENSOR_H

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