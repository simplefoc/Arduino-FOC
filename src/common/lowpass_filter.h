#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"
#include "base_classes/CommunicationNode.h"

/**
 *  Low pass filter class
 */
class LowPassFilter: public CommunicationNode
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; //!< Low pass filter time constant

    String communicate(String user_command) override;

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};

#endif // LOWPASS_FILTER_H