#include "GenericCurrentSense.h"

// GenericCurrentSense constructor
GenericCurrentSense::GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)()){
  // if function provided add it to the 
  if(readCallback != nullptr) this->readCallback = readCallback;
  if(initCallback != nullptr) this->initCallback = initCallback;
}

// Inline sensor init function
int GenericCurrentSense::init(){
    // configure ADC variables
    if(initCallback != nullptr) initCallback();
    // calibrate zero offsets
    calibrateOffsets();
    // set the initialized flag
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // return success
    return 1;
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
    current.a = (current.a - offset_ia); // amps
    current.b = (current.b - offset_ib); // amps
    current.c = (current.c - offset_ic); // amps
    return current;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
int GenericCurrentSense::driverAlign(float voltage, bool modulation_centered){
    _UNUSED(voltage) ; // remove unused parameter warning
    int exit_flag = 1;
    if(skip_align) return exit_flag;
    if (!initialized) return 0;
    return exit_flag;
}
