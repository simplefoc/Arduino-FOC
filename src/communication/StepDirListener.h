#ifndef STEPDIR_H
#define STEPDIR_H

#include "Arduino.h"
#include "../common/foc_utils.h"


#if defined(_STM32_DEF_) || defined(ESP_H) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_SAM_DUE) || defined(CORE_TEENSY) || defined(NRF52_SERIES)
#define PinStatus int
#endif


/**
 * Step/Dir listenner class for easier interraction with this communication interface.
 */
class StepDirListener
{
  public:

    /**
     * Constructor for step/direction interface
     *  @param step              - pin
     *  @param direction         - pin
     *  @param counter_to_value  - step counter to value
     */
    StepDirListener(int pinStep, int pinDir, float counter_to_value = 1);
    /**
     * Start listenning for step commands
     * 
     * @param handleStep - on step received handler
     */
    void enableInterrupt(void (*handleStep)());

    /**
     * Initialise dir and step commands
     */
    void init();
    /**
     * step handler
     */
    void handle();
    /**
     * Get so far received valued
     */
    float getValue();
    /**
     * Attach the value to be updated on each step receive 
     * - no need to call getValue function
     */
    void attach(float* variable);

    // variables
    int pin_step; //!< step pin
    int pin_dir; //!< direction pin
    long count; //!< current counter value - should be set to 0 for homing
    PinStatus polarity = RISING; //!< polarity of the step pin

  private:
    float* attached_variable = nullptr; //!< pointer to the attached variable 
    float counter_to_value; //!< step counter to value 
    //bool step_active = 0; //!< current step pin status (HIGH/LOW) - debouncing variable

};

#endif