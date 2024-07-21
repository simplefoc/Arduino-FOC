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
int CurrentSense::driverAlign(float voltage, bool modulation_centered){
        
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if (!initialized) return 0;

    // check if stepper or BLDC 
    if(driver_type == DriverType::Stepper)
        return alignStepperDriver(voltage, (StepperDriver*)driver, modulation_centered);
    else
        return alignBLDCDriver(voltage, (BLDCDriver*)driver, modulation_centered);
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
int CurrentSense::alignBLDCDriver(float voltage, BLDCDriver* bldc_driver, bool modulation_centered){
        
    bool phases_switched = 0;
    bool phases_inverted = 0;
    
    float zero = 0;
    if(modulation_centered) zero = driver->voltage_limit/2.0;

    // set phase A active and phases B and C down
    // 300 ms of ramping
    for(int i=0; i < 100; i++){
        bldc_driver->setPwm(voltage/100.0*((float)i)+zero , zero, zero);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c_a = readAverageCurrents();
    bldc_driver->setPwm(zero, zero, zero);
    // check if currents are to low (lower than 100mA) 
    // TODO calculate the 100mA threshold from the ADC resolution
    // if yes throw an error and return 0
    // either the current sense is not connected or the current is 
    // too low for calibration purposes (one should raise the motor.voltage_sensor_align)
    if((fabs(c_a.a) < 0.1f) && (fabs(c_a.b) < 0.1f) && (fabs(c_a.c) < 0.1f)){
            SIMPLEFOC_DEBUG("CS: Err too low current, rise voltage!");
            return 0; // measurement current too low
    }

    
    // now we have to determine 
    // 1) which pin correspond to which phase of the bldc driver
    // 2) if the currents measured have good polarity
    // 
    // > when we apply a voltage to a phase A of the driver what we expect to measure is the current I on the phase A
    //   and -I/2 on the phase B and I/2 on the phase C

    // find the highest magnitude in c_a
    // and make sure it's around 2 (1.5 at least) times higher than the other two
    float ca[3] = {fabs(c_a.a), fabs(c_a.b), fabs(c_a.c)};
    uint8_t max_i = -1; // max index
    float max_c = 0; // max current
    float max_c_ratio = 0; // max current ratio
    for(int i = 0; i < 3; i++){
        if(!ca[i]) continue; // current not measured
        if(ca[i] > max_c){
            max_c = ca[i];
            max_i = i;
            for(int j = 0; j < 3; j++){
                if(i == j) continue;
                if(!ca[j]) continue; // current not measured
                float ratio = max_c / ca[j];
                if(ratio > max_c_ratio) max_c_ratio = ratio;
            }
        }
    }

    // check the current magnitude ratios
    // 1) if there is one current that is approximately 2 times higher than the other two
    //    this is the A current
    // 2) if the max current is not at least 1.5 times higher than the other two
    //    we have two cases:
    //    - either we only measure two currents and the third one is not measured - then phase A is not measured
    //    - or the current sense is not connected properly

    if(max_c_ratio >=1.5f){
        switch (max_i){
            case 1: // phase B is the max current
                SIMPLEFOC_DEBUG("CS: Switch A-B");
                // switch phase A and B
                _swap(pinA, pinB);
                _swap(offset_ia, offset_ib);
                _swap(gain_a, gain_b);
                _swap(c_a.b, c_a.b);
                phases_switched = true; // signal that pins have been switched
                break;
            case 2: // phase C is the max current
                SIMPLEFOC_DEBUG("CS: Switch A-C");
                // switch phase A and C
                _swap(pinA, pinC);
                _swap(offset_ia, offset_ic);
                _swap(gain_a, gain_c);
                _swap(c_a.a, c_a.c);
                phases_switched = true;// signal that pins have been switched
                break;
        }
        // check if the current is negative and invert the gain if so
        if( _sign(c_a.a) < 0 ){
            SIMPLEFOC_DEBUG("CS: Inv A");
            gain_a *= -1;
            phases_inverted = true; // signal that pins have been inverted
        }
    }else if(_isset(pinA) && _isset(pinB) && _isset(pinC)){
        // if all three currents are measured and none of them is significantly higher
        // we have a problem with the current sense
        SIMPLEFOC_DEBUG("CS: Err A - all currents same magnitude!");
        return 0;
    }else{ //phase A is not measured so put the _NC to the phase A
        if(_isset(pinA) && !_isset(pinB)){
            SIMPLEFOC_DEBUG("CS: Switch A-(B)NC");
            _swap(pinA, pinB);
            _swap(offset_ia, offset_ib);
            _swap(gain_a, gain_b);
            _swap(c_a.b, c_a.b);
            phases_switched = true; // signal that pins have been switched
        }else if(_isset(pinA) && !_isset(pinC)){
            SIMPLEFOC_DEBUG("CS: Switch A-(C)NC");
            _swap(pinA, pinC);
            _swap(offset_ia, offset_ic);
            _swap(gain_a, gain_c);
            _swap(c_a.b, c_a.c);
            phases_switched = true; // signal that pins have been switched
        }
    }
    // at this point the current sensing on phase A can be either:
    // - aligned with the driver phase A
    // - or the phase A is not measured and the _NC is connected to the phase A
    //
    // In either case A is done, now we have to check the phase B and C 

    
    // set phase B active and phases A and C down
    // 300 ms of ramping
    for(int i=0; i < 100; i++){
        bldc_driver->setPwm(zero, voltage/100.0*((float)i)+zero, zero);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c_b = readAverageCurrents();
    bldc_driver->setPwm(zero, zero, zero);

    // check the phase B
    // find the highest magnitude in c_b
    // and make sure it's around 2 (1.5 at least) times higher than the other two
    float cb[3] = {fabs(c_b.a), fabs(c_b.b), fabs(c_b.c)};
    max_i = -1; // max index
    max_c = 0; // max current
    max_c_ratio = 0; // max current ratio
    for(int i = 0; i < 3; i++){
        if(!cb[i]) continue; // current not measured
        if(cb[i] > max_c){
            max_c = cb[i];
            max_i = i;
            for(int j = 0; j < 3; j++){
                if(i == j) continue;
                if(!cb[j]) continue; // current not measured
                float ratio = max_c / cb[j];
                if(ratio > max_c_ratio) max_c_ratio = ratio;
            }
        }
    }
    if(max_c_ratio >= 1.5f){
        switch (max_i){
            case 0: // phase A is the max current
                // this is an error as phase A is already aligned
                SIMPLEFOC_DEBUG("CS: Err align B");
                return 0;
            case 2: // phase C is the max current
                SIMPLEFOC_DEBUG("CS: Switch B-C");
                _swap(pinB, pinC);
                _swap(offset_ib, offset_ic);
                _swap(gain_b, gain_c);
                _swap(c_b.b, c_b.c);
                phases_switched = true; // signal that pins have been switched
                break;
        }
        // check if the current is negative and invert the gain if so
        if( _sign(c_b.b) < 0 ){
            SIMPLEFOC_DEBUG("CS: Inv B");
            gain_b *= -1;
            phases_inverted = true; // signal that pins have been inverted
        }
    }else if(_isset(pinB) && _isset(pinC)){
        // if all three currents are measured and none of them is significantly higher
        // we have a problem with the current sense
        SIMPLEFOC_DEBUG("CS: Err B - all currents same magnitude!");
        return 0;
    }else{ //phase B is not measured so put the _NC to the phase B
        if(_isset(pinB) && !_isset(pinC)){
            SIMPLEFOC_DEBUG("CS: Switch B-(C)NC");
            _swap(pinB, pinC);
            _swap(offset_ib, offset_ic);
            _swap(gain_b, gain_c);
            _swap(c_b.b, c_b.c);
            phases_switched = true; // signal that pins have been switched
        }
    }
    // at this point the current sensing on phase A and B can be either:
    // - aligned with the driver phase A and B
    // - or the phase A and B are not measured and the _NC is connected to the phase A and B
    //
    // In either case A and B is done, now we have to check the phase C
    // phase C is also aligned if it is measured (not _NC)
    // we have to check if the current is negative and invert the gain if so
    if(_isset(pinC)){
        if( _sign(c_b.c) > 0 ){ // the expected current is -I/2 (if the phase A and B are aligned and C has correct polarity)
            SIMPLEFOC_DEBUG("CS: Inv C");
            gain_c *= -1;
            phases_inverted = true; // signal that pins have been inverted
        }
    }

    // construct the return flag
    // if the phases have been switched return 2
    // if the gains have been inverted return 3
    // if both return 4
    uint8_t exit_flag = 1;
    if(phases_switched) exit_flag += 1;
    if(phases_inverted) exit_flag += 2;
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
int CurrentSense::alignStepperDriver(float voltage, StepperDriver* stepper_driver, bool modulation_centered){
    
    _UNUSED(modulation_centered);

    bool phases_switched = 0;
    bool phases_inverted = 0;

    if(!_isset(pinA) || !_isset(pinB)){
        SIMPLEFOC_DEBUG("CS: Pins A & B not specified!");
        return 0;
    }

    // set phase A active and phases B down
    // ramp 300ms
    for(int i=0; i < 100; i++){
        stepper_driver->setPwm(voltage/100.0*((float)i), 0);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c = readAverageCurrents();
    // disable the phases
    stepper_driver->setPwm(0, 0);        
    if (fabs(c.a) < 0.1f && fabs(c.b) < 0.1f ){
        SIMPLEFOC_DEBUG("CS: Err too low current!");
        return 0; // measurement current too low
    }
    // align phase A
    // 1) only one phase can be measured so we first measure which ADC pin corresponds 
    // to the phase A by comparing the magnitude
    if (fabs(c.a) < fabs(c.b)){
        SIMPLEFOC_DEBUG("CS: Switch A-B");
        // switch phase A and B
        _swap(pinA, pinB);
        _swap(offset_ia, offset_ib);
        _swap(gain_a, gain_b);
        phases_switched = true; // signal that pins have been switched
    }
    // 2) check if measured current a is positive and invert if not
    if (c.a < 0){
        SIMPLEFOC_DEBUG("CS: Inv A");
        gain_a *= -1;
        phases_inverted = true; // signal that pins have been inverted
    }

    // at this point the driver's phase A is aligned with the ADC pinA
    // and the pin B should be the phase B

    // set phase B active and phases A down
    // ramp 300ms
    for(int i=0; i < 100; i++){
        stepper_driver->setPwm(0, voltage/100.0*((float)i));
        _delay(3);
    }
    _delay(500);
    c = readAverageCurrents();
    stepper_driver->setPwm(0, 0);

    // phase B should be aligned
    // 1) we just need to verify that it has been measured
    if (fabs(c.b) < 0.1f ){
        SIMPLEFOC_DEBUG("CS: Err too low current on B!");
        return 0; // measurement current too low
    }
    // 2) check if measured current a is positive and invert if not
    if (c.b < 0){
        SIMPLEFOC_DEBUG("CS: Inv B");
        gain_b *= -1;
        phases_inverted = true; // signal that pins have been inverted
    }

    // construct the return flag
    // if success and nothing changed return 1 
    // if the phases have been switched return 2
    // if the gains have been inverted return 3
    // if both return 4
    uint8_t exit_flag = 1;
    if(phases_switched) exit_flag += 1;
    if(phases_inverted) exit_flag += 2;
    return exit_flag;
}


