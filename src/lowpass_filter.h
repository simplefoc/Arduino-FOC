#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

class ExponentialMovingAverage
{
public:
    ExponentialMovingAverage(float Tf);
    ~ExponentialMovingAverage() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif // LOWPASS_FILTER_H
