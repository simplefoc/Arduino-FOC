#ifndef PID_H
#define PID_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  PID controller class
 */
class PIDController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     * @param sampling_time - sampling time
     * 
     * @note If sampling time is not set the filter will measure the 
     *       elapsed time between each call. If yes it will consider it fixed. 
     * @note Sampling time can be changed dynamically as well by modifying the 
     *       variable Ts in runtime.
     */
    PIDController(float P, float I, float D, float ramp, float limit, float sampling_time = NOT_SET);
    ~PIDController() = default;

    float operator() (float error);
    void reset();

    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value
    float Ts; //!< Fixed sampling time (optional default NOT_SET)

protected:
    float error_prev; //!< last tracking error value
    float output_prev;  //!< last pid output value
    float integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
};

#endif // PID_H