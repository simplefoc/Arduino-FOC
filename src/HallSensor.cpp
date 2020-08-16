#include "HallSensor.h"


/*
  HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp           - pole pairs
*/

HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp){
  
  // HallSensor measurement structure init
  // hardware pins
  pinA = _hallA;
  pinB = _hallB;
  pinC = _hallC;
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _pp * 6; // hall has 6 segments per electrical revolution
  A_active = 0;
  B_active = 0;
  C_active = 0;

  // velocity calculation variables

  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // extern pullup as default
  pullup = Pullup::EXTERN;
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

void HallSensor::updateState() {
  int newState = C_active + (B_active << 1) + (A_active << 2);
  Direction direction = decodeDirection(state, newState); 
  state = newState;
  pulse_counter += direction; 
  pulse_timestamp = _micros();
}

// determines whether the hallsensr state transition means that it has moved one step CW (+1), CCW (-1) or state transition is invalid (0)
// states are 3bits, one for each hall sensor
Direction HallSensor::decodeDirection(int oldState, int newState)
{
  // here are expected state transitions (oldState > newState).
  // CW state transitions are:  ( 6 > 2 > 3 > 1 > 5 > 4 > 6 )
  // CCW state transitions are: ( 6 > 4 > 5 > 1 > 3 > 2 > 6 )
  // invalid state transitions are oldState == newState or if newState or oldState == 0 | 7 as hallsensors can't be all on or all off

  int rawDirection;
  
  if (
      (oldState == 6 && newState == 2) || \
      (oldState == 2 && newState == 3) || \
      (oldState == 3 && newState == 1) || \
      (oldState == 1 && newState == 5) || \
      (oldState == 5 && newState == 4) || \
      (oldState == 4 && newState == 6) 
    ) {
    rawDirection = Direction::CW;
  } else if(
      (oldState == 6 && newState == 4) || \
      (oldState == 4 && newState == 5) || \
      (oldState == 5 && newState == 1) || \
      (oldState == 1 && newState == 3) || \
      (oldState == 3 && newState == 2) || \
      (oldState == 2 && newState == 6) 
  ) {
    rawDirection = Direction::CCW;
  } else {
    rawDirection = Direction::UNKNOWN;
  }

  // setting sensor.reverse in setup() will reverse direction
  direction = static_cast<Direction>(rawDirection * reverse);
  return direction; // * goofy;
}

/*
	Shaft angle calculation
*/
float HallSensor::getAngle(){
  
  long dN = pulse_counter - prev_pulse_counter;

  if (dN != 0)
  {
    
    // time from last impulse
    float Th = (pulse_timestamp - prev_timestamp_us) * 1e-6;
    if (Th <= 0 || Th > 0.5)
      Th = 1e-3;
    // save variables for next pass
    prev_timestamp_us = pulse_timestamp;
    prev_pulse_counter = pulse_counter;
    velocity = (float) _2PI * dN / (cpr * Th);
  }
  angle = (float) _2PI * pulse_counter /  cpr;

  return angle;
}
/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity(){
  // this is calculated during the last call to getAngle();
  return velocity;
}

// getter for index pin
// return -1 if no index
int HallSensor::needsAbsoluteZeroSearch(){
  return 0;
}
// getter for index pin
int HallSensor::hasAbsoluteZero(){
  return 0;
}
// initialize counter to zero
float HallSensor::initRelativeZero(){
  return 0.0;
}
// initialize index to zero
float HallSensor::initAbsoluteZero(){
  return 0.0;
}

// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init(){
  
  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }
  
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = _micros();
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

}

// function enabling hardware interrupts of the for the callback provided
// if callback is not provided then the interrupt is not enabled
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()){
  // attach interrupt if functions provided

  // A, B and C callback
  if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
  if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
  if(doC != nullptr) attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE);
}

