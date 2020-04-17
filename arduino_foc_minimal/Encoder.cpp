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
  index_pulse_counter = 0;

  // velocity calculation varibles
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // extern pullup as default
  pullup = Pullup::EXTERN;
  // enable quadrature encoder by default
  quadrature = Quadrature::ENABLE;
}

//  Encoder interrupt callback functions
//  enabling CPR=4xPPR behaviour
// A channel
void Encoder::handleA() {
  int A = digitalRead(pinA);
  switch (quadrature){
    case Quadrature::ENABLE:
      // CPR = 4xPPR
      if ( A != A_active ) {
        pulse_counter += (A_active == B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        A_active = A;
      }
      break;
    case Quadrature::DISABLE:
      // CPR = PPR
      if(A && !digitalRead(pinB)){
        pulse_counter++;
        pulse_timestamp = _micros();
      }
      break;
  }
 if(hasIndex()){
    int I = digitalRead(index_pin);
    if(I && !I_active){
      // aling encoder on each index 
      if(index_pulse_counter){
        long tmp = pulse_counter;
        pulse_counter = round((float)pulse_counter/(float)cpr)*cpr;
        prev_pulse_counter = pulse_counter + (tmp-prev_pulse_counter);
      } 
      // initial offset 
      if(!index_pulse_counter) index_pulse_counter = pulse_counter;
    }
    I_active = I;
  }
}
// B channel
void Encoder::handleB() {
  int B = digitalRead(pinB);
  switch (quadrature){
    case Quadrature::ENABLE:
      // CPR = 4xPPR
      if ( B != B_active ) {
        pulse_counter += (A_active != B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        B_active = B;
      }
      break;
    case Quadrature::DISABLE:
      // CPR = PPR
      if(B && !digitalRead(pinA)){
        pulse_counter--;
        pulse_timestamp = _micros();
      }
      break;
  }
 if(hasIndex()){
    int I = digitalRead(index_pin);
    if(I && !I_active){
      // aling encoder on each index 
      if(index_pulse_counter){
        long tmp = pulse_counter;
        pulse_counter = round((float)pulse_counter/(float)cpr)*cpr;
        prev_pulse_counter = pulse_counter + (tmp-prev_pulse_counter);
      } 
      // initial offset 
      if(!index_pulse_counter) index_pulse_counter = pulse_counter;
    }
    I_active = I;
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
  long timestamp_us = _micros();
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
  if ( Th > 0.15) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (2.0 * M_PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return (velocity);
}

// getter for index pin
// return -1 if no index
int Encoder::indexFound(){
  return index_pulse_counter != 0;
}
// getter for index pin
int Encoder::hasIndex(){
  return index_pin != 0;
}
// getter for Index angle
float Encoder::getIndexAngle(){
  return  (index_pulse_counter) / ((float)cpr) * (2.0 * M_PI);
}


// intialise counter to zero
void Encoder::setCounterZero(){
  pulse_counter = 0;
  pulse_timestamp = _micros();
}
// intialise index to zero
void Encoder::setIndexZero(){
  pulse_counter -= index_pulse_counter;
  prev_pulse_counter = pulse_counter;
}



void Encoder::init(void (*doA)(), void(*doB)()){
  
  // Encoder - check if pullup needed for your encoder
  if(pullup == Pullup::INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
  }
  // if index used intialise it
  if(hasIndex()) pinMode(index_pin,INPUT);

  // counter setup
  pulse_counter = 0;
  pulse_timestamp = _micros();
  // velocity calculation varibles
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();


  // attach interrupt if functions provided
  if(doA != nullptr){
  switch(quadrature){
    case Quadrature::ENABLE:
      cpr = 4*cpr;
      // CPR = 4xPPR
      attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
      break;
    case Quadrature::DISABLE:
      // CPR = PPR
      attachInterrupt(digitalPinToInterrupt(pinA), doA, RISING);
      attachInterrupt(digitalPinToInterrupt(pinB), doB, RISING);
      break;
  }
    // // A callback and B callback
  }
}


long _micros(){
  return micros();
}
