#include "GenericCurrentSense.h"

// GenericCurrentSense constructor
GenericCurrentSense::GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)()){
  // if function provided add it to the 
  if(readCallback != nullptr) this->readCallback = readCallback;
  if(initCallback != nullptr) this->initCallback = initCallback;
}

// Inline sensor init function
void GenericCurrentSense::init(){
    // configure ADC variables
    if(initCallback != nullptr) initCallback();
    // calibrate zero offsets
    calibrateOffsets();
}
// Function finding zero offsets of the ADC
void GenericCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;

    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
        PhaseCurrent_s current = readCallback();
        offset_ia += current.a;
        offset_ib += current.b;
        offset_ic += current.c;
        _delay(1);
    }
    // calculate the mean offsets
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s GenericCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current = readCallback();
    current.a = (current.a - offset_ia);// amps
    current.b = (current.a - offset_ib);// amps
    current.c = (current.a - offset_ic); // amps
    return current;
}

// Function synchronizing current sense with motor driver.
// for in-line sensig no such thing is necessary
int GenericCurrentSense::driverSync(BLDCDriver *driver){
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
int GenericCurrentSense::driverAlign(BLDCDriver *driver, float voltage){
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    // // set phase A active and phases B and C down
    // driver->setPwm(voltage, 0, 0);
    // _delay(200);
    // PhaseCurrent_s c = getPhaseCurrents();
    // // read the current 100 times ( arbitrary number )
    // for (int i = 0; i < 100; i++) {
    //     PhaseCurrent_s c1 = getPhaseCurrents();
    //     c.a = c.a*0.6f + 0.4f*c1.a;
    //     c.b = c.b*0.6f + 0.4f*c1.b;
    //     c.c = c.c*0.6f + 0.4f*c1.c;
    //     _delay(3);
    // }
    // driver->setPwm(0, 0, 0);
    // // align phase A
    // float ab_ratio = fabs(c.a / c.b);
    // float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
    // if( ab_ratio > 1.5f ){ // should be ~2
    //     gain_a *= _sign(c.a);
    // }else if( ab_ratio < 0.7f ){ // should be ~0.5
    //     // switch phase A and B
    //     int tmp_pinA = pinA;
    //     pinA = pinB;
    //     pinB = tmp_pinA;
    //     gain_a *= _sign(c.b);
    //     exit_flag = 2; // signal that pins have been switched
    // }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
    //     // switch phase A and C
    //     int tmp_pinA = pinA;
    //     pinA = pinC;
    //     pinC= tmp_pinA;
    //     gain_a *= _sign(c.c);
    //     exit_flag = 2;// signal that pins have been switched
    // }else{
    //     // error in current sense - phase either not measured or bad connection
    //     return 0;
    // }

    // // set phase B active and phases A and C down
    // driver->setPwm(0, voltage, 0);
    // _delay(200);
    // c = getPhaseCurrents();
    // // read the current 50 times
    // for (int i = 0; i < 100; i++) {
    //     PhaseCurrent_s c1 = getPhaseCurrents();
    //     c.a = c.a*0.6f + 0.4f*c1.a;
    //     c.b = c.b*0.6f + 0.4f*c1.b;
    //     c.c = c.c*0.6f + 0.4f*c1.c;
    //     _delay(3);
    // }
    // driver->setPwm(0, 0, 0);
    // float ba_ratio = fabs(c.b/c.a);
    // float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
    //  if( ba_ratio > 1.5f ){ // should be ~2
    //     gain_b *= _sign(c.b);
    // }else if( ba_ratio < 0.7f ){ // it should be ~0.5
    //     // switch phase A and B
    //     int tmp_pinB = pinB;
    //     pinB = pinA;
    //     pinA = tmp_pinB;
    //     gain_b *= _sign(c.a);
    //     exit_flag = 2; // signal that pins have been switched
    // }else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
    //     // switch phase A and C
    //     int tmp_pinB = pinB;
    //     pinB = pinC;
    //     pinC = tmp_pinB;
    //     gain_b *= _sign(c.c);
    //     exit_flag = 2; // signal that pins have been switched
    // }else{
    //     // error in current sense - phase either not measured or bad connection
    //     return 0;
    // }

    // // if phase C measured
    // if(_isset(pinC)){
    //     // set phase B active and phases A and C down
    //     driver->setPwm(0, 0, voltage);
    //     _delay(200);
    //     c = getPhaseCurrents();
    //     // read the adc voltage 500 times ( arbitrary number )
    //     for (int i = 0; i < 50; i++) {
    //         PhaseCurrent_s c1 = getPhaseCurrents();
    //         c.c = (c.c+c1.c)/50.0f;
    //     }
    //     driver->setPwm(0, 0, 0);
    //     gain_c *= _sign(c.c);
    // }

    // if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted
    return exit_flag;
}
