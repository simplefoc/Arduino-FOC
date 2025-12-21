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
     * @param Ts - Filter sampling time
     * 
     * @note If sampling time Ts is not set the filter will measure the 
     *       elapsed time between each call. 
     * @note Ts can be changed dynamically as well by modifying the 
     *       variable in runtime.
     */
    LowPassFilter(float Tf, float Ts = NOT_SET);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; //!< Low pass filter time constant
    float Ts; //!< Fixed sampling time (optional default NOT_SET)

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};

#endif // LOWPASS_FILTER_H