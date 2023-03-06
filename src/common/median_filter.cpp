#include "median_filter.h"

MedianFilter::MedianFilter(int samples)
{
    this->samples = samples;
    y_prev = 0.0f;
    timestamp_prev = _micros();
}

float MedianFilter::operator() (float x)
{
    sample_history.push_back(x);
    if(sample_history.size()>samples){
        sample_history.pop_front();
    }
    // calculate median over sample history
    std::deque<float> copydeque = sample_history;

    //sort vector in ascending order
    sort(copydeque.begin(),copydeque.end());

    float median;
    int size = copydeque.size();
    if (size%2==0)
    { //even number of elements
        median = (copydeque[size/2-1] + copydeque[size / 2]) / 2.0;
    } else { //odd number of elements
        median = copydeque[size/2];
    }

}