#include "InlineCurrentSense.h"
// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};


InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // mV to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};



// Inline sensor init function
int InlineCurrentSense::init(){
    // if no linked driver its fine in this case 
    // at least for init()
    void* drv_params = driver ? driver->params : nullptr;
    // configure ADC variables
    params = _configureADCInline(drv_params,pinA,pinB,pinC);
    // if init failed return fail
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // calibrate zero offsets
    calibrateOffsets();
    // set the initialized flag
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // return success
    return 1;
}
// Function finding zero offsets of the ADC
void InlineCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;
    
    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
        if(_isset(pinA)) offset_ia += _readADCVoltageInline(pinA, params);
        if(_isset(pinB)) offset_ib += _readADCVoltageInline(pinB, params);
        if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC, params);
        _delay(1);
    }
    // calculate the mean offsets
    if(_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
    if(_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageInline(pinA, params) - offset_ia)*gain_a;// amps
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageInline(pinB, params) - offset_ib)*gain_b;// amps
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC, params) - offset_ic)*gain_c; // amps
    return current;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int InlineCurrentSense::driverAlign(float voltage){
    
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if(_isset(pinA)){
        // set phase A active and phases B and C down
        driver->setPwm(voltage, 0, 0);
        _delay(2000);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 100 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6f + 0.4f*c1.a;
            c.b = c.b*0.6f + 0.4f*c1.b;
            c.c = c.c*0.6f + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        // align phase A
        float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
        float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
        if(_isset(pinB) && ab_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinC) && ac_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinB) && ab_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and B
            int tmp_pinA = pinA;
            pinA = pinB;
            pinB = tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ib;
            offset_ib = tmp_offsetA;
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinA = pinA;
            pinA = pinC;
            pinC= tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ic;
            offset_ic = tmp_offsetA;
            gain_a *= _sign(c.c);
            exit_flag = 2;// signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    if(_isset(pinB)){
        // set phase B active and phases A and C down
        driver->setPwm(0, voltage, 0);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 50 times
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
        float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
        if(_isset(pinA) && ba_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pinC) && bc_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pinA) && ba_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and B
            int tmp_pinB = pinB;
            pinB = pinA;
            pinA = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ia;
            offset_ia = tmp_offsetB;
            gain_b *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinB = pinB;
            pinB = pinC;
            pinC = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ic;
            offset_ic = tmp_offsetB;
            gain_b *= _sign(c.c);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    // if phase C measured
    if(_isset(pinC)){
        // set phase C active and phases A and B down
        driver->setPwm(0, 0, voltage);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
        float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
        if(_isset(pinA) && ca_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinB) && cb_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinA) && ca_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and C
            int tmp_pinC = pinC;
            pinC = pinA;
            pinA = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ia;
            offset_ia = tmp_offsetC;
            gain_c *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinB) && cb_ratio < 0.7f ){ // should be ~0.5
            // switch phase B and C
            int tmp_pinC = pinC;
            pinC = pinB;
            pinB = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ib;
            offset_ib = tmp_offsetC;
            gain_c *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted

    return exit_flag;
}
