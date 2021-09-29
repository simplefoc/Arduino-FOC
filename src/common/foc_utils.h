#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"

// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

#define NOT_SET -12345.0
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1

// dq current structure
struct DQCurrent_s
{
    float d;
    float q;
};
// phase current structure
struct PhaseCurrent_s
{
    float a;
    float b;
    float c;
};
// dq voltage structs
struct DQVoltage_s
{
    float d;
    float q;
};


/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _sin(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _cos(float a);

/**
 * normalizing radian angle to [0,2PI]
 * @param angle - angle to be normalized
 */
float _normalizeAngle(float angle);


/**
 * Electrical angle calculation
 *
 * @param shaft_angle - shaft angle of the motor
 * @param pole_pairs - number of pole pairs
 */
float _electricalAngle(float shaft_angle, int pole_pairs);

/**
 * Function approximating square root function
 *  - using fast inverse square root
 *
 * @param value - number
 */
float _sqrtApprox(float value);

#endif