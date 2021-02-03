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
    volts_to_amps_ratio = 1.0 /_shunt_resistor / _gain; // volts to amps
}

// Inline sensor init function
void InlineCurrentSense::init(){
    // calibrate zero offsets
    calibrateOffsets();
}
// Function finding zero offsets of the ADC
void InlineCurrentSense::calibrateOffsets(){
    // find adc offset = zero current voltage
    offset_ia =0;
    offset_ib= 0;
    offset_ic= 0;
    // read the adc voltage 500 times ( arbitrary number )
    for (int i = 0; i < 500; i++) {
        offset_ia += _readADCVoltage(pinA);
        offset_ib += _readADCVoltage(pinB);
        if(pinC != NOT_SET) offset_ic += _readADCVoltage(pinC);
    }
    // calculate the mean offsets
    offset_ia = offset_ia / 500.0;
    offset_ib = offset_ib / 500.0;
    if(pinC != NOT_SET) offset_ic = offset_ic / 500.0;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = gain_adjust_a*(_readADCVoltage(pinA) - offset_ia)*volts_to_amps_ratio;// amps
    current.b = gain_adjust_b*(_readADCVoltage(pinB) - offset_ib)*volts_to_amps_ratio;// amps
    current.c = (pinC == NOT_SET) ? 0 : gain_adjust_c*(_readADCVoltage(pinC) - offset_ic)*volts_to_amps_ratio; // amps
    return current;
}

// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided
float InlineCurrentSense::getCurrent(float motor_electrical_angle){
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
