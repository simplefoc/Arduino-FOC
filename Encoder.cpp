#include "Encoder.h"


/*
  Encoder(int encA, int encB , int cpr, int index)
  - encA, encB    - encoder A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
  - index pin     - (optional input)
*/

Encoder::Encoder(int _encA, int _encB , float _cpr, int _index){
  
  // Encoder measurement structure init
  // hardware pins
  pinA = _encA;
  pinB = _encB;
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = 0;
  cpr = _cpr;
  A_active = 0;
  B_active = 0;
  I_active = 0;
  // index pin
  index = _index; // its 0 if not used

  // velocity calculation varibles
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = micros();

  // extern pullup as default
  pullup = Pullup::EXTERN;
}

//  Encoder interrupt callback functions
//  enabling CPR=4xPPR behaviour
// A channel
void Encoder::handleA() {
  int A = digitalRead(pinA);
  if ( A != A_active ) {
    pulse_counter += (A_active == B_active) ? 1 : -1;
    pulse_timestamp = micros();
    A_active = A;
  }
}
// B channel
void Encoder::handleB() {
  int B = digitalRead(pinB);
  if ( B != B_active ) {
    pulse_counter += (A_active != B_active) ? 1 : -1;
    pulse_timestamp = micros();
    B_active = B;
  }
}

/*
	Shaft angle calculation
*/
float Encoder::getAngle(){
  return  (pulse_counter) / ((float)cpr) * (2.0 * M_PI);
}
/*
  Shaft velocity calculation
  funciton using mixed time and frequency measurement technique
*/
float Encoder::getVelocity(){
  // timestamp
  long timestamp_us = micros();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6;
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
  
  // if more than 0.3 passed in between impulses
  if ( Th > 0.1) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (2.0 * M_PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return (velocity);
}

// intialise counter to zero
void Encoder::setCounterZero(){
  pulse_counter = 0;
}


void Encoder::init(){
  
  // Encoder - check if pullup needed for your encoder
  if(pullup == INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
  }
  // if index used intialise it
  if(index) pinMode(index,INPUT);

  // counter setup
  pulse_counter = 0;
  pulse_timestamp = micros();
  // velocity calculation varibles
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = micros();

 
}


void Encoder::enableInterrupt(){
   // interupt intitialisation
  // A callback and B callback
  attachInterrupt(digitalPinToInterrupt(pinA), []() {
    encoder.handleA();
  }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), []() {
    encoder.handleB();
  }, CHANGE);
}