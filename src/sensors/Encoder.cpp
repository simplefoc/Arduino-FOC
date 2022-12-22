#include "Encoder.h"


/*
  Encoder(int encA, int encB , int cpr, int index)
  - encA, encB    - encoder A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
  - index pin     - (optional input)
*/

Encoder::Encoder(int _encA, int _encB , float _ppr, int _index){

  // Encoder measurement structure init
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

  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
  // enable quadrature encoder by default
  quadrature = Quadrature::ON;
}

//  Encoder interrupt callback functions
// A channel
void Encoder::handleA() {
  bool A = digitalRead(pinA);
  switch (quadrature){
    case Quadrature::ON:
      // CPR = 4xPPR
      if ( A != A_active ) {
        pulse_counter += (A_active == B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        A_active = A;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(A && !digitalRead(pinB)){
        pulse_counter++;
        pulse_timestamp = _micros();
      }
      break;
  }
}
// B channel
void Encoder::handleB() {
  bool B = digitalRead(pinB);
  switch (quadrature){
    case Quadrature::ON:
  //     // CPR = 4xPPR
      if ( B != B_active ) {
        pulse_counter += (A_active != B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        B_active = B;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(B && !digitalRead(pinA)){
        pulse_counter--;
        pulse_timestamp = _micros();
      }
      break;
  }
}

// Index channel
void Encoder::handleIndex() {
  if(hasIndex()){
    bool I = digitalRead(index_pin);
    if(I && !I_active){
      index_found = true;
      // align encoder on each index
      long tmp = pulse_counter;
      // corrent the counter value
      pulse_counter = round((double)pulse_counter/(double)cpr)*cpr;
      // preserve relative speed
      prev_pulse_counter += pulse_counter - tmp;
    }
    I_active = I;
  }
}


void Encoder::update() {
    // do nothing for Encoder
}

/*
	Shaft angle calculation
*/
float Encoder::getSensorAngle(){
  return getAngle();
}
// TODO: numerical precision issue here if the pulse_counter overflows the angle will be lost
float Encoder::getMechanicalAngle(){
  return  _2PI * ((pulse_counter) % ((int)cpr)) / ((float)cpr);
}

float Encoder::getAngle(){
  return  _2PI * (pulse_counter) / ((float)cpr);
}
double Encoder::getPreciseAngle(){
  return  _2PI * (pulse_counter) / ((double)cpr);
}
int32_t Encoder::getFullRotations(){
  return  pulse_counter / (int)cpr;
}



/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float Encoder::getVelocity(){
  // timestamp
  long timestamp_us = _micros();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // time from last impulse
  float Th = (timestamp_us - pulse_timestamp) * 1e-6f;
  long dN = pulse_counter - prev_pulse_counter;

  // Pulse per second calculation (Eq.3.)
  // dN - impulses received
  // Ts - sampling time - time in between function calls
  // Th - time from last impulse
  // Th_1 - time form last impulse of the previous call
  // only increment if some impulses received
  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;

  // if more than 0.05f passed in between impulses
  if ( Th > 0.1f) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (_2PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return velocity;
}

// getter for index pin
// return -1 if no index
int Encoder::needsSearch(){
  return hasIndex() && !index_found;
}

// private function used to determine if encoder has index
int Encoder::hasIndex(){
  return index_pin != 0;
}


// encoder initialisation of the hardware pins
// and calculation variables
void Encoder::init(){

  // Encoder - check if pullup needed for your encoder
  if(pullup == Pullup::USE_INTERN){
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

  // initial cpr = PPR
  // change it if the mode is quadrature
  if(quadrature == Quadrature::ON) cpr = 4*cpr;

  // we don't call Sensor::init() here because init is handled in Encoder class.
}

// function enabling hardware interrupts of the for the callback provided
// if callback is not provided then the interrupt is not enabled
void Encoder::enableInterrupts(void (*doA)(), void(*doB)(), void(*doIndex)()){
  // attach interrupt if functions provided
  switch(quadrature){
    case Quadrature::ON:
      // A callback and B callback
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
      break;
    case Quadrature::OFF:
      // A callback and B callback
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, RISING);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, RISING);
      break;
  }

  // if index used initialize the index interrupt
  if(hasIndex() && doIndex != nullptr) attachInterrupt(digitalPinToInterrupt(index_pin), doIndex, CHANGE);
}
