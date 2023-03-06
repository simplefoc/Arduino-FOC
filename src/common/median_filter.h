#ifndef MEIDAN_FILTER_H
#define MEDIAN_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"
#include "filter.h"
#include <deque>
#include <algorithm>

/**
 *  Low pass filter class
 */
class MedianFilter : public Filter
{
public:
    /**
     * @param samples - Sample size for moving median
     */
    MedianFilter(int samples);
    ~MedianFilter() = default;

    float operator() (float x);
private:
    int samples; // Sample size for moving median
    std::deque<float> sample_history;
};

#endif // LOWPASS_FILTER_H