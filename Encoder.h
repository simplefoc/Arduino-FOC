#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"


// Pullup configuation structure
enum Pullup{
    INTERN,
    EXTERN
};

class Encoder{
 public:
    /*
    Encoder(int encA, int encB , int cpr, int index)
    - encA, encB    - encoder A and B pins
    - cpr           - counts per rotation number (cpm=ppm*4)
    - index pin     - (optional input)
    */
    Encoder(int encA, int encB , float cpr, int index = 0);

    // encoder initialise pins
    void init(void (*doA)() = nullptr, void(*doB)() = nullptr);

    //  Encoder interrupt callback functions
    //  enabling CPR=4xPPR behaviour
    // A channel
    void handleA();
    // B channel
    void handleB();
    
    // encoder getters
    // shaft velocity getter
    float getVelocity();
    float getAngle();

    // setter for counter to zero
    void setCounterZero();
    
    // pins A and B
    int pinA, pinB;           // encoder hardware pins
    // index pin
    int index;
    // encoder pullup type
    Pullup pullup;


  private:
    long pulse_counter;       // current pulse counter
    long pulse_timestamp;     // last impulse timestamp in us
    float cpr;                  // impulse cpr
    int A_active, B_active;   // current active states of A and B line
    int I_active;  // index active

    // velocity calculation varibles
    float prev_Th, pulse_per_second;
    long prev_pulse_counter, prev_timestamp_us;

};


#endif
