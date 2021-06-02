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
    USE_INTERN, //!< Use internal pullups
    USE_EXTERN //!< Use external pullups
};

/**
 *  Sensor abstract class defintion
 * Each sensor needs to have these functions implemented
 */
class Sensor{
    public:

        /** get current angle (rad) */
        virtual float getAngle()=0;
        /** get current angular velocity (rad/s)*/
        virtual float getVelocity();

        /** 
         * returns 0 if it does need search for absolute zero
         * 0 - magnetic sensor (& encoder with index which is found)
         * 1 - ecoder with index (with index not found yet)
         */
        virtual int needsSearch();
    private:
        // velocity calculation variables
        float angle_prev=0; //!< angle in previous velocity calculation step
        long velocity_calc_timestamp=0; //!< last velocity calculation timestamp
};

#endif
