#include "CurrentSense.h"


// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided
float CurrentSense::getDCCurrent(float motor_electrical_angle){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();
    
    // calculate clarke transform
    ABCurrent_s ABcurrent = getABCurrents(current);

    // current sign - if motor angle not provided the magnitude is always positive
    float sign = 1;

    // if motor angle provided function returns signed value of the current
    // determine the sign of the current
    // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
    if(motor_electrical_angle) {
        float ct;
        float st;
        _sincos(motor_electrical_angle, &st, &ct);
        sign = (ABcurrent.beta*ct - ABcurrent.alpha*st) > 0 ? 1 : -1;  
    }
    // return current magnitude
    return sign*_sqrt(ABcurrent.alpha*ABcurrent.alpha + ABcurrent.beta*ABcurrent.beta);
}

// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents and getABCurrents internally
DQCurrent_s CurrentSense::getFOCCurrents(float angle_el){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();

    // calculate clarke transform
    ABCurrent_s ABcurrent = getABCurrents(current);
    
    // calculate park transform
    DQCurrent_s return_current = getDQCurrents(ABcurrent,angle_el);

    return return_current;
}

// function used with the foc algorihtm
//   calculating Alpha Beta currents from phase currents
//   - function calculating Clarke transform of the phase currents
ABCurrent_s CurrentSense::getABCurrents(PhaseCurrent_s current){

    // calculate clarke transform
    float i_alpha, i_beta;
    if(!current.c){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else if(!current.a){
        // if only two measured currents
        float a = -current.c - current.b;
        i_alpha = a;  
        i_beta = _1_SQRT3 * a + _2_SQRT3 * current.b;
    }else if(!current.b){
        // if only two measured currents
        float b = -current.a - current.c;
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * b;
    } else {
        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    ABCurrent_s return_ABcurrent;
    return_ABcurrent.alpha = i_alpha;
    return_ABcurrent.beta = i_beta;
    return return_ABcurrent;
}

// function used with the foc algorihtm
//   calculating D and Q currents from Alpha Beta currents and electrical angle
//   - function calculating Clarke transform of the phase currents
DQCurrent_s CurrentSense::getDQCurrents(ABCurrent_s current, float angle_el){
 // calculate park transform
    float ct;
    float st;
    _sincos(angle_el, &st, &ct);
    DQCurrent_s return_current;
    return_current.d = current.alpha * ct + current.beta * st;
    return_current.q = current.beta * ct - current.alpha * st;
    return return_current;
}

/**
	Driver linking to the current sense
*/
void CurrentSense::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}


void CurrentSense::enable(){
    // nothing is done here, but you can override this function
};

void CurrentSense::disable(){
    // nothing is done here, but you can override this function
};