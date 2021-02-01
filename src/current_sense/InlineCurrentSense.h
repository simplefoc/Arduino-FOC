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
      @param phA A phase pwm pin
      @param phB B phase pwm pin
      @param phC C phase pwm pin (optional)
    */
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);

    void init() override;

    PhaseCurrent_s getCurrents() override;
    DQCurrent_s getFOCCurrents(float angle_el) override;


    // hardware variables
  	int pinA; //!< pin A analog pin for current measurement
  	int pinB; //!< pin B analog pin for current measurement
  	int pinC; //!< pin C analog pin for current measurement

  private:


    double shunt_resistor; //!< Shunt resistor value 
    double amp_gain; //!< amp gain value 
    double volts_to_amps_ratio; //!< Volts to amps ratio
    
    // function finding the zero offsets
    void calibrateOffsets();
    double offset_ia; //!< zero current A voltage value (center of the adc reading)
    double offset_ib; //!< zero current B voltage value (center of the adc reading)
    double offset_ic; //!< zero current C voltage value (center of the adc reading)

};

#endif