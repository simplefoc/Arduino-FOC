#include "HallSensor.h"


/*
  HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp           - pole pairs
*/
HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp){

  // hardware pins
  pinA = _hallA;
  pinB = _hallB;
  pinC = _hallC;

  // hall has 6 segments per electrical revolution
  cpr = _pp * 6;

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
}

//  HallSensor interrupt callback functions
// A channel
void HallSensor::handleA() {
  A_active= digitalRead(pinA);
  updateState();
}
// B channel
void HallSensor::handleB() {
  B_active = digitalRead(pinB);
  updateState();
}

// C channel
void HallSensor::handleC() {
  C_active = digitalRead(pinC);
  updateState();
}

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor::updateState() {
  long new_pulse_timestamp = _micros();

  int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);

  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (new_hall_state == hall_state) {
    return;
  }
  hall_state = new_hall_state;

  int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
  static Direction old_direction;
  if (new_electric_sector - electric_sector > 3) {
    //underflow
    direction = Direction::CCW;
    electric_rotations += direction;
  } else if (new_electric_sector - electric_sector < (-3)) {
    //overflow
    direction = Direction::CW;
    electric_rotations += direction;
  } else {
    direction = (new_electric_sector > electric_sector)? Direction::CW : Direction::CCW;
  }
  electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if (direction == old_direction) {
    // not oscilating or just changed direction
    pulse_diff = new_pulse_timestamp - pulse_timestamp;
  } else {
    pulse_diff = 0;
  }

  pulse_timestamp = new_pulse_timestamp;
  total_interrupts++;
  old_direction = direction;
  if (onSectorChange != nullptr) onSectorChange(electric_sector);
}

/**
 * Optionally set a function callback to be fired when sector changes
 * void onSectorChange(int sector) {
 *  ... // for debug or call driver directly?
 * }
 * sensor.attachSectorCallback(onSectorChange);
 */
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
  onSectorChange = _onSectorChange;
}



float HallSensor::getSensorAngle() {
  return getAngle();
}



/*
	Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor::getMechanicalAngle() {
  return ((float)((electric_rotations * 6 + electric_sector) % cpr) / (float)cpr) * _2PI ;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity(){
  if (pulse_diff == 0 || ((long)(_micros() - pulse_timestamp) > pulse_diff) ) { // last velocity isn't accurate if too old
    return 0;
  } else {
    return direction * (_2PI / (float)cpr) / (pulse_diff / 1000000.0f);
  }

}



float HallSensor::getAngle() {
  return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI ;
}


double HallSensor::getPreciseAngle() {
  return ((double)(electric_rotations * 6 + electric_sector) / (double)cpr) * (double)_2PI ;
}


int32_t HallSensor::getFullRotations() {
  return (int32_t)((electric_rotations * 6 + electric_sector) / cpr);
}





// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init(){
  // initialise the electrical rotations to 0
  electric_rotations = 0;

  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::USE_INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }

    // init hall_state
  A_active= digitalRead(pinA);
  B_active = digitalRead(pinB);
  C_active = digitalRead(pinC);
  updateState();

  pulse_timestamp = _micros();

  // we don't call Sensor::init() here because init is handled in HallSensor class.
}

// function enabling hardware interrupts for the callback provided
// if callback is not provided then the interrupt is not enabled
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()){
  // attach interrupt if functions provided

  // A, B and C callback
  if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
  if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
  if(doC != nullptr) attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE);
}
