#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  Low pass filter class
 */
class LowPassFilter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; //!< Low pass filter time constant
    float y_prev;
    float timestamp_prev;

};

#endif // LOWPASS_FILTER_H