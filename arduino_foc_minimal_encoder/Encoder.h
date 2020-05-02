#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"
#include "FOCutils.h"
#include "Sensor.h"


// Pullup configuation structure
enum Pullup{
    INTERN,
    EXTERN
};

enum Quadrature{
  ENABLE, // CPR = 4xPPR
  DISABLE // CPR = PPR
};

class Encoder: public Sensor{
 public:
    /*
    Encoder(int encA, int encB , int cpr, int index)
    - encA, encB    - encoder A and B pins
    - ppr           - impulses per rotation  (cpr=ppr*4)
    - index pin     - (optional input)
    */
    Encoder(int encA, int encB , float ppr, int index = 0);

    // encoder initialise pins
    void init();
    // funciton enabling hardware interrupts of the for the callback provided
    // if callback is not provided then the interrupt is not enabled
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doIndex)() = nullptr);
    
    //  Encoder interrupt callback functions
    //  enabling CPR=4xPPR behaviour
    // A channel
    void handleA();
    // B channel
    void handleB();
    // index handle
    void handleIndex();
    
    
    // pins A and B
    int pinA, pinB;           // encoder hardware pins
    // index pin
    int index_pin;
    // encoder pullup type
    Pullup pullup;
    // use 4xppr or not
    Quadrature quadrature;

    // implementation of abstract functions of the Sensor class
    // get current angle (rad)
    float getAngle();
    // get current angular velocity (rad/s)
    float getVelocity();
    // set current agle as zero angle 
    // return the angle [rad] difference
    float initRelativeZero();
    // set index angle as zero angle
    // return the angle [rad] difference
    float initAbsoluteZero();
    // returns 0 if it has no index 
    // 0 - encoder without index
    // 1 - encoder with index 
    int hasAbsoluteZero();
    // returns 0 if it does need search for absolute zero
    // 0 - encoder without index 
    // 1 - ecoder with index
    int needsAbsoluteZeroSearch();

  private:
    int hasIndex();

    volatile long pulse_counter;        // current pulse counter
    volatile long pulse_timestamp;      // last impulse timestamp in us
    float cpr;                 // impulse cpr
    volatile int A_active, B_active;    // current active states of A and B line
    volatile int I_active;              // index active
    volatile long index_pulse_counter;  // pulse counter of the index

    // velocity calculation varibles
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;

};


#endif
