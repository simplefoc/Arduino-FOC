#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"

template<typename T>
constexpr inline int _sign(T val) {
  return __builtin_signbit(val);
}

#ifndef __AVR__
#include <type_traits>

#ifndef _round
// Use enable_if to select the roundf function for single precision floats.
// This improves performance when -ffast-math is not set.
template<typename T>
constexpr inline typename std::enable_if<std::is_same<T, float>::value, long>::type _round(T x) {
  return __builtin_roundf(x);
}
template<typename T>
constexpr inline typename std::enable_if<std::is_same<T, double>::value, long>::type _round(T x) {
  return __builtin_round(x);
}
#endif

// Use enable_if to select the fastest implementation according to the amt type.
// Using __builtin_fXf is measurably faster than using the ternary approach.
template<typename T, typename L, typename H>
constexpr inline typename std::enable_if<std::is_integral<T>::value, T>::type _constrain(T amt, L low, H high) {
  return (amt < low) ? low : (amt > high) ? high : amt;
}
template<typename T, typename L, typename H>
constexpr inline typename std::enable_if<std::is_same<T, float>::value, T>::type _constrain(T amt, L low, H high) {
  return __builtin_fmaxf(low, __builtin_fminf(high, amt));
}
template<typename T, typename L, typename H>
constexpr inline typename std::enable_if<std::is_same<T, double>::value, T>::type _constrain(T amt, L low, H high) {
  return __builtin_fmax(low, __builtin_fmin(high, amt));
}
#else  // __AVR__
// AVR compiler lacks type_traits, so we are forced to use the slower non-type inferenced
// version. That's okay, right? If you wanted to go fast you would not be on AVR.
template<typename T>
constexpr long _round(T x) {
  return __builtin_round(x);
}
template<typename T, typename L, typename H>
constexpr T _constrain(T amt, L low, H high) {
  return (amt < low) ? low : (amt > high) ? high : amt;
}
#endif

#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))

#define _swap(a, b) { auto temp = a; a = b; b = temp; }

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
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC ((int) NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)

// dq variables
struct DQ_s
{
    float d;
    float q;
};

// dq voltage structs
typedef DQ_s DQVoltage_s;
// dq current structs
typedef DQ_s DQCurrent_s;

// alpha-beta variables    
struct AB_s
{
    float alpha;
    float beta;
};
typedef AB_s ABVoltage_s;  // NOT USED
typedef AB_s ABCurrent_s;

// phase structs
struct Phase_s
{
    float a;
    float b;
    float c;
};
typedef Phase_s PhaseVoltage_s; // NOT USED
typedef Phase_s PhaseCurrent_s;


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
 * Function returning both sine and cosine of the angle in one call.
 * Internally it uses the _sin and _cos functions, but you may wish to
 * provide your own implementation which is more optimized.
 */
void _sincos(float a, float* s, float* c);

/**
 * Function approximating atan2 
 * 
 */
float _atan2(float y, float x);

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
