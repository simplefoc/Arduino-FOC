#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant, float sampling_time)
    : Tf(time_constant)
    , Ts(sampling_time)
    , y_prev(0.0f)
{
    timestamp_prev = _micros();
}


float LowPassFilter::operator() (float x)
{
    // initalise the elapsed time with the fixed sampling tims Ts
    float dt = Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(!_isset(dt)){
        unsigned long timestamp = _micros();
        float dt = (timestamp - timestamp_prev)*1e-6f;

        if (dt < 0.0f ) dt = 1e-3f;
        else if(dt > 0.3f) {
            y_prev = x;
            timestamp_prev = timestamp;
            return x;
        }
        timestamp_prev = timestamp;
    }

    // calculate the first order filer
    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;

    // save the variables for the future steps
    y_prev = y;
    return y;
}
