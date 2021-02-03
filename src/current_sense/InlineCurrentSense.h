#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/base_classes/CurrentSense.h"
#include "hardware_api.h"


class InlineCurrentSense: public CurrentSense{
  public:
    /**
      InlineCurrentSense class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);

    // CurrentSense interface implementing functions 
    void init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    float getCurrent( float angle_el  = 0) override;


    // ADC measuremnet gain adjustment for each phase
    // if (shunt+amp combination) measures inverse current you can set it to -1 for example
    // this should be automated later
  	int gain_adjust_a = 1; //!< phase A gain adjust
  	int gain_adjust_b = 1; //!< phase B gain adjust
  	int gain_adjust_c = 1; //!< phase C gain adjust

  private:

    // hardware variables
  	int pinA; //!< pin A analog pin for current measurement
  	int pinB; //!< pin B analog pin for current measurement
  	int pinC; //!< pin C analog pin for current measurement

    // gain variables
    double shunt_resistor; //!< Shunt resistor value 
    double amp_gain; //!< amp gain value 
    double volts_to_amps_ratio; //!< Volts to amps ratio
    
    /**
     *  Function finding zero offsets of the ADC
     */
    void calibrateOffsets();
    double offset_ia; //!< zero current A voltage value (center of the adc reading)
    double offset_ib; //!< zero current B voltage value (center of the adc reading)
    double offset_ic; //!< zero current C voltage value (center of the adc reading)

};

#endif