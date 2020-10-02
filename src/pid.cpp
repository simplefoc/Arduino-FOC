#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)  // output supply limit     [volts]
    , integral(0.0)
    , err_prev(0.0)
    , y_prev(0.0)
{
    t_prev = _micros()*1e-6;
}


inline void
PIDController::constrain(float& y)
{
    if (y > limit)
        y = limit;
    if (y < -limit)
        y = -limit;
}


float
PIDController::operator() (float t, float err)
{
    float dt = t - t_prev;
    float y_P = P*err;
    integral += I*dt*(err + err_prev)/2.0; // trapezoidal rule
    constrain(integral);
    float y_D = D*(err - err_prev)/dt;

    float y = y_P + integral + y_D;
    constrain(y);

    float actual_rate = (y - y_prev)/dt;
    if (actual_rate > ramp)
        y = ramp*dt;

    if (actual_rate < -ramp)
        y = -ramp*dt;

    err_prev = err;
    t_prev   = t;
    y_prev   = y;

    return y;
}
