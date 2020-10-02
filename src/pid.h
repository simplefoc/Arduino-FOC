#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator()(float time, float err);

    float P;
    float I;
    float D;
    float ramp;
    float limit;

protected:
    float integral;
    float err_prev;
    float t_prev;
    float y_prev;

    inline void constrain(float& y);
};

#endif // PID_H
