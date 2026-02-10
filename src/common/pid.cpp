#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit, float sampling_time)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [ex. volts/second]
    , limit(limit)         // output supply limit     [ex. volts]
    , Ts(sampling_time)    // sampling time [S]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = _micros();
}

// PID controller function
float PIDController::operator() (float error){
    // initalise the elapsed time with the fixed sampling tims Ts
    float dt = Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(!_isset(dt)){
        unsigned long timestamp_now = _micros();
        dt = (timestamp_now - timestamp_prev) * 1e-6f;
        // quick fix for strange cases (micros overflow)
        if(dt <= 0 || dt > 0.5f) dt = 1e-3f;
        timestamp_prev = timestamp_now;
    }

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*dt*0.5f*(error + error_prev);
    // antiwindup - limit the output
    if(_isset(limit)) integral = _constrain(integral, -limit, limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/dt;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    if(_isset(limit)) output = _constrain(output, -limit, limit);

    // if output ramp defined
    if(_isset(output_ramp) && output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/dt;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*dt;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*dt;
    }
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    return output;
}

void PIDController::reset(){
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}
