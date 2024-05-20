#include "CurrentSense.h"
#include "../../communication/SimpleFOCDebug.h"


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

// function used with the foc algorithm
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

// function used with the foc algorithm
//   calculating Alpha Beta currents from phase currents
//   - function calculating Clarke transform of the phase currents
ABCurrent_s CurrentSense::getABCurrents(PhaseCurrent_s current){

    // check if driver is an instance of StepperDriver
    // if so there is no need to Clarke transform
    if (driver_type == DriverType::Stepper){
        ABCurrent_s return_ABcurrent;
        return_ABcurrent.alpha = current.a;
        return_ABcurrent.beta = current.b;
        return return_ABcurrent;
    }

    // otherwise it's a BLDC motor and 
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

// function used with the foc algorithm
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
void CurrentSense::linkDriver(FOCDriver* _driver) {
    driver = _driver;
    // save the driver type for easier access
    driver_type = driver->type();
}


void CurrentSense::enable(){
    // nothing is done here, but you can override this function
};

void CurrentSense::disable(){
    // nothing is done here, but you can override this function
};


// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
// IMPORTANT, this function can be overriden in the child class
int CurrentSense::driverAlign(float voltage){
        
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if (!initialized) return 0;

    // check if stepper or BLDC 
    if(driver_type == DriverType::Stepper)
        return alignStepperDriver(voltage, (StepperDriver*)driver);
    else
        return alignBLDCDriver(voltage, (BLDCDriver*)driver);
}



// Helper function to read and average phase currents
PhaseCurrent_s CurrentSense::readAverageCurrents(int N) {
    PhaseCurrent_s c = getPhaseCurrents();
    for (int i = 0; i < N; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a * 0.6f + 0.4f * c1.a;
        c.b = c.b * 0.6f + 0.4f * c1.b;
        c.c = c.c * 0.6f + 0.4f * c1.c;
        _delay(3);
    }
    return c;
};



// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int CurrentSense::alignBLDCDriver(float voltage, BLDCDriver* bldc_driver){
        
    int exit_flag = 1;
    if(_isset(pinA)){
        // set phase A active and phases B and C down
        bldc_driver->setPwm(voltage, 0, 0);
        _delay(500);
        PhaseCurrent_s c = readAverageCurrents();
        bldc_driver->setPwm(0, 0, 0);
        // align phase A
        float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
        float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
        if(_isset(pinB) && ab_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinC) && ac_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinB) && ab_ratio < 0.7f ){ // should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch A-B");
            // switch phase A and B
            _swap(pinA, pinB);
            _swap(offset_ia, offset_ib);
            _swap(gain_a, gain_b);
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch A-C");
            // switch phase A and C
            _swap(pinA, pinC);
            _swap(offset_ia, offset_ic);
            _swap(gain_a, gain_c);
            gain_a *= _sign(c.c);
            exit_flag = 2;// signal that pins have been switched
        }else{
            SIMPLEFOC_DEBUG("CS: Err read A");
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }
    
    if(_isset(pinB)){
        // set phase B active and phases A and C down
        bldc_driver->setPwm(0, voltage, 0);
        _delay(500);
        PhaseCurrent_s c = readAverageCurrents();
        bldc_driver->setPwm(0, 0, 0);
        float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
        float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
        if(_isset(pinA) && ba_ratio > 1.5f ){ // should be ~2);
            gain_b *= _sign(c.b);
        }else if(_isset(pinC) && bc_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pinA) && ba_ratio < 0.7f ){ // it should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch B-A");
            // switch phase A and B
            _swap(pinB, pinA);
            _swap(offset_ib, offset_ia);
            _swap(gain_b, gain_a);
            gain_b *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch B-C");
            _swap(pinB, pinC);
            _swap(offset_ib, offset_ic);
            _swap(gain_b, gain_c);
            gain_b *= _sign(c.c);
            exit_flag = 2; // signal that pins have been switched
        }else{
            SIMPLEFOC_DEBUG("CS: Error read B");
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    // if phase C measured
    if(_isset(pinC)){
        // set phase C active and phases A and B down
        bldc_driver->setPwm(0, 0, voltage);
        _delay(500);
        PhaseCurrent_s c = readAverageCurrents();
        bldc_driver->setPwm(0, 0, 0);
        float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
        float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
        if(_isset(pinA) && ca_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinB) && cb_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinA) && ca_ratio < 0.7f ){ // it should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch C-A");
            // switch phase A and C
            _swap(pinC, pinA);
            _swap(offset_ic, offset_ia);
            _swap(gain_c, gain_a);
            gain_c *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinB) && cb_ratio < 0.7f ){ // should be ~0.5
            SIMPLEFOC_DEBUG("CS: Switch C-B");
            _swap(pinC, pinB);
            _swap(offset_ic, offset_ib);
            _swap(gain_c, gain_b);
            gain_b *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else{
            SIMPLEFOC_DEBUG("CS: Err read C");
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }
    // add 2 if pin gains negative
    if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    return exit_flag;
}


// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int CurrentSense::alignStepperDriver(float voltage, StepperDriver* stepper_driver){
        
    int exit_flag = 1;

    if(_isset(pinA)){
        // set phase A active to high and B to low
        stepper_driver->setPwm(voltage, 0);
        _delay(500);
        PhaseCurrent_s c = readAverageCurrents();
        // disable the phases
        stepper_driver->setPwm(0, 0);
        if (fabs(c.a) < 0.1f && fabs(c.b) < 0.1f ){
            SIMPLEFOC_DEBUG("CS: Err too low current!");
            return 0; // measurement current too low
        }
        // align phase A
        // check if measured current a is positive and invert if not
        // check if current b is around zero and if its not 
        // check if current a is near zero and if it is invert them
        if (fabs(c.a) < fabs(c.b)){
            SIMPLEFOC_DEBUG("CS: Switch A-B");
            // switch phase A and B
            _swap(pinA, pinB);
            _swap(offset_ia, offset_ib);
            _swap(gain_a, gain_b);
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else if (c.a < 0){
            SIMPLEFOC_DEBUG("CS: Neg A");
            gain_a *= -1;
        }
    }

    if(_isset(pinB)){
        // set phase B active and phases A and C down
        stepper_driver->setPwm(voltage, 0);
        _delay(500);
        PhaseCurrent_s c = readAverageCurrents();
        stepper_driver->setPwm(0, 0);
        if (fabs(c.a) < 0.1f && fabs(c.b) < 0.1f ){
            SIMPLEFOC_DEBUG("CS: Err too low current!");
            return 0; // measurement current too low
        }
        // align phase A
        // check if measured current a is positive and invert if not
        if (c.b < 0){
            SIMPLEFOC_DEBUG("CS: Neg B");
            gain_b *= -1;
        }
    }

    // add 2 if pin gains negative
    if(gain_a < 0 || gain_b < 0 ) exit_flag +=2;
    return exit_flag;
}


