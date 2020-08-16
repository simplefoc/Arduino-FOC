#include "HallSensor.h"


/*
  HallSensor(int encA, int encB , int cpr, int index)
  - encA, encB    - HallSensor A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
  - index pin     - (optional input)
*/

HallSensor::HallSensor(int _encA, int _encB , float _ppr, int _index){
  
  // HallSensor measurement structure init
  // hardware pins
  pinA = _encA;
  pinB = _encB;
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _ppr;
  A_active = 0;
  B_active = 0;
  I_active = 0;
  // index pin
  index_pin = _index; // its 0 if not used
  index_pulse_counter = 0;

  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // extern pullup as default
  pullup = Pullup::EXTERN;
  // enable quadrature HallSensor by default

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
  return index_pulse_counter == 0;
}
// getter for index pin
int HallSensor::hasAbsoluteZero(){
  return hasIndex();
}
// initialize counter to zero
float HallSensor::initRelativeZero(){
  long angle_offset = -pulse_counter;
  pulse_counter = 0;
  pulse_timestamp = _micros();
  return _2PI * (angle_offset) / ((float)cpr);
}
// initialize index to zero
float HallSensor::initAbsoluteZero(){
  pulse_counter -= index_pulse_counter;
  prev_pulse_counter = pulse_counter;
  return (index_pulse_counter) / ((float)cpr) * (_2PI);
}
// private function used to determine if HallSensor has index
int HallSensor::hasIndex(){
  return index_pin != 0;
}


// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init(){
  
  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    if(hasIndex()) pinMode(index_pin,INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    if(hasIndex()) pinMode(index_pin,INPUT);
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
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doIndex)()){
  // attach interrupt if functions provided

  // A callback and B callback
  if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
  if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
}

