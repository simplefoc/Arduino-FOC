#include "InlineCurrentSense.h"

InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0 /_shunt_resistor / _gain; // volts to amps
}

void InlineCurrentSense::init(){
    // calibrate zero offsets
    calibrateOffsets();
}

void InlineCurrentSense::calibrateOffsets(){
    // find adc offset = zero current voltage
    offset_ia =0;
    offset_ib= 0;
    offset_ic= 0;
    for (int i = 0; i < 500; i++) {
        offset_ia += _readADCVoltage(pinA);
        offset_ib += _readADCVoltage(pinB);
        if(pinC != NOT_SET) offset_ic += _readADCVoltage(pinC);
    }
    offset_ia = offset_ia / 500.0;
    offset_ib = offset_ib / 500.0;
    if(pinC != NOT_SET) offset_ic = offset_ic / 500.0;
}

DQCurrent_s InlineCurrentSense::getFOCCurrents(float angle_el){
    // read current phase currents
    PhaseCurrent_s current = getCurrents();

    // calculate clarke transform
    float i_alpha, i_beta;
    if(pinC == NOT_SET){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else{
        i_alpha = 2*(current.a - (current.b - current.c))/3.0;    
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

PhaseCurrent_s InlineCurrentSense::getCurrents(){
    PhaseCurrent_s current;
    current.a = (_readADCVoltage(pinA) - offset_ia)*volts_to_amps_ratio;// amps
    current.b = -(_readADCVoltage(pinB) - offset_ib)*volts_to_amps_ratio;// amps
    current.c = (pinC == NOT_SET) ? -(current.a + current.b) : (_readADCVoltage(pinC) - offset_ic)*volts_to_amps_ratio; // amps
    return current;
}
