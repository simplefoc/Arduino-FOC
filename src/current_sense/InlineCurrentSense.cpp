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
}

// Inline sensor init function
void InlineCurrentSense::init(){
    // configure ADC variables
    _configureADCInline(pinA,pinB,pinC);
    // calibrate zero offsets
    calibrateOffsets();
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
        offset_ia += _readADCVoltageInline(pinA);
        offset_ib += _readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC);
        _delay(1);
    }
    // calculate the mean offsets
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = (_readADCVoltageInline(pinA) - offset_ia)*gain_a;// amps
    current.b = (_readADCVoltageInline(pinB) - offset_ib)*gain_b;// amps
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
    return current;
}
// Function synchronizing current sense with motor driver.
// for in-line sensig no such thing is necessary
int InlineCurrentSense::driverSync(BLDCDriver *driver){
    return 1;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int InlineCurrentSense::driverAlign(BLDCDriver *driver, float voltage){
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    // set phase A active and phases B and C down
    driver->setPwm(voltage, 0, 0);
    _delay(200);
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
    float ab_ratio = fabs(c.a / c.b);
    float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
    if( ab_ratio > 1.5f ){ // should be ~2
        gain_a *= _sign(c.a);
    }else if( ab_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and B
        int tmp_pinA = pinA;
        pinA = pinB;
        pinB = tmp_pinA;
        gain_a *= _sign(c.b);
        exit_flag = 2; // signal that pins have been switched
    }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and C
        int tmp_pinA = pinA;
        pinA = pinC;
        pinC= tmp_pinA;
        gain_a *= _sign(c.c);
        exit_flag = 2;// signal that pins have been switched
    }else{
        // error in current sense - phase either not measured or bad connection
        return 0;
    }

    // set phase B active and phases A and C down
    driver->setPwm(0, voltage, 0);
    _delay(200);
    c = getPhaseCurrents();
    // read the current 50 times
    for (int i = 0; i < 100; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a*0.6f + 0.4f*c1.a;
        c.b = c.b*0.6f + 0.4f*c1.b;
        c.c = c.c*0.6f + 0.4f*c1.c;
        _delay(3);
    }
    driver->setPwm(0, 0, 0);
    float ba_ratio = fabs(c.b/c.a);
    float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
     if( ba_ratio > 1.5f ){ // should be ~2
        gain_b *= _sign(c.b);
    }else if( ba_ratio < 0.7f ){ // it should be ~0.5
        // switch phase A and B
        int tmp_pinB = pinB;
        pinB = pinA;
        pinA = tmp_pinB;
        gain_b *= _sign(c.a);
        exit_flag = 2; // signal that pins have been switched
    }else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and C
        int tmp_pinB = pinB;
        pinB = pinC;
        pinC = tmp_pinB;
        gain_b *= _sign(c.c);
        exit_flag = 2; // signal that pins have been switched
    }else{
        // error in current sense - phase either not measured or bad connection
        return 0;
    }

    // if phase C measured
    if(_isset(pinC)){
        // set phase B active and phases A and C down
        driver->setPwm(0, 0, voltage);
        _delay(200);
        c = getPhaseCurrents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 50; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.c = (c.c+c1.c)/50.0f;
        }
        driver->setPwm(0, 0, 0);
        gain_c *= _sign(c.c);
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
