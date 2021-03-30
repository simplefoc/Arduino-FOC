#include "CurrentSense.h"


// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided
float CurrentSense::getDCCurrent(float motor_electrical_angle){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();
    // currnet sign - if motor angle not provided the magnitude is always positive
    float sign = 1;

    // calculate clarke transform
    float i_alpha, i_beta;
    if(!current.c){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else{
        i_alpha = 2*(current.a - (current.b - current.c))/3.0;    
        i_beta = _2_SQRT3 *( current.b  - current.c );
    }

    // if motor angle provided function returns signed value of the current
    // determine the sign of the current
    // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
    if(motor_electrical_angle) 
        sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
    // return current magnitude
    return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
}

// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents internally
DQCurrent_s CurrentSense::getFOCCurrents(float angle_el){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();

    // calculate clarke transform
    float i_alpha, i_beta;
    if(!current.c){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else{
        i_alpha = 0.6666667*(current.a - (current.b - current.c));    
        i_beta = _2_SQRT3 *( current.b  - current.c );
    }

    // calculate park transform
    float ct = _cos(angle_el);
    float st = _sin(angle_el);
    DQCurrent_s return_current;
    return_current.d = i_alpha * ct + i_beta * st;
    return_current.q = i_beta * ct - i_alpha * st;
    return return_current;
}