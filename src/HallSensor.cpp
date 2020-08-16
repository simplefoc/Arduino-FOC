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
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // extern pullup as default
  pullup = Pullup::EXTERN;

}

//  HallSensor interrupt callback functions
// A channel
void HallSensor::handleA() {
  int A = digitalRead(pinA);

  if ( A != A_active ) {
    pulse_counter += (A_active == B_active) ? 1 : -1;
    pulse_timestamp = _micros();
    A_active = A;
  }
}
// B channel
void HallSensor::handleB() {
  int B = digitalRead(pinB);
 
  if ( B != B_active ) {
    pulse_counter += (A_active != B_active) ? 1 : -1;
    pulse_timestamp = _micros();
    B_active = B;
  }
 
}

// C channel
void HallSensor::handleC() {
  int C = digitalRead(pinB);
 
  if ( C != C_active ) {
    pulse_counter += (A_active != C_active) ? 1 : -1;
    pulse_timestamp = _micros();
    C_active = C;
  }
 
}

/*
	Shaft angle calculation
*/
float HallSensor::getAngle(){
  return  _2PI * (pulse_counter) / ((float)cpr);
}
/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity(){
  // timestamp
  long timestamp_us = _micros();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 
  
  // time from last impulse
  float Th = (timestamp_us - pulse_timestamp) * 1e-6;
  long dN = pulse_counter - prev_pulse_counter;

  // Pulse per second calculation (Eq.3.)
  // dN - impulses received
  // Ts - sampling time - time in between function calls
  // Th - time from last impulse
  // Th_1 - time form last impulse of the previous call
  // only increment if some impulses received
  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;
  
  // if more than 0.05 passed in between impulses
  if ( Th > 0.1) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (_2PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return (velocity);
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
// private function used to determine if HallSensor has index
int HallSensor::hasIndex(){
  return 0;
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
  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
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

