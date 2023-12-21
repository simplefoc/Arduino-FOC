#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/base_classes/Sensor.h"


/**
 *  Quadrature mode configuration structure
 */
enum Quadrature : uint8_t {
  ON    = 0x00, //!<  Enable quadrature mode CPR = 4xPPR
  OFF   = 0x01  //!<  Disable quadrature mode / CPR = PPR
};

class Encoder: public Sensor{
 public:
    /**
    Encoder class constructor
    @param encA  encoder B pin
    @param encB  encoder B pin
    @param ppr  impulses per rotation  (cpr=ppr*4)
    @param index index pin number (optional input)
    */
    Encoder(int encA, int encB , float ppr, int index = 0);

    /** encoder initialise pins */
    void init() override;
    /**
     *  function enabling hardware interrupts for the encoder channels with provided callback functions
     *  if callback is not provided then the interrupt is not enabled
     * 
     * @param doA pointer to the A channel interrupt handler function
     * @param doB pointer to the B channel interrupt handler function
     * @param doIndex pointer to the Index channel interrupt handler function
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doIndex)() = nullptr);
    
    //  Encoder interrupt callback functions
    /** A channel callback function */
    void handleA();
    /** B channel callback function */
    void handleB();
    /** Index channel callback function */
    void handleIndex();
    
    
    // pins A and B
    int pinA; //!< encoder hardware pin A
    int pinB; //!< encoder hardware pin B
    int index_pin; //!< index pin

    // Encoder configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    Quadrature quadrature;//!< Configuration parameter enable or disable quadrature mode
    float cpr;//!< encoder cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getSensorAngle() override;
    /**  get current angular velocity (rad/s) */
    float getVelocity() override;
    virtual void update() override;

    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index 
     * 1 - ecoder with index
     */
    int needsSearch() override;

  private:
    int hasIndex(); //!< function returning 1 if encoder has index pin and 0 if not.

    volatile long pulse_counter;//!< current pulse counter
    volatile long pulse_timestamp;//!< last impulse timestamp in us
    volatile int A_active; //!< current active states of A channel
    volatile int B_active; //!< current active states of B channel
    volatile int I_active; //!< current active states of Index channel
    volatile bool index_found = false; //!< flag stating that the index has been found

    // velocity calculation variables
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
};


#endif
