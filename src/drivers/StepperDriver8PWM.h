#ifndef STEPPER_DRIVER_8PWM_h
#define STEPPER_DRIVER_8PWM_h

#include "../common/base_classes/StepperDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 8 pwm stepper driver class
*/
class StepperDriver8PWM : public StepperDriver {
  public:
    /**
      StepperMotor class constructor
      @param ph1A 1A phase pwm pin
      @param ph1B 1B phase pwm pin
      @param ph2A 2A phase pwm pin
      @param ph2B 2B phase pwm pin
      @param ph3A 3A phase pwm pin
      @param ph3B 3B phase pwm pin
      @param ph4A 4A phase pwm pin
      @param ph4B 4B phase pwm pin
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
      @param en3 enable pin phase 3 (optional input)
      @param en4 enable pin phase 4 (optional input)
    */
    StepperDriver8PWM(int ph1A, int ph1B, int ph2A, int ph2B, int ph3A, int ph3B, int ph4A, int ph4B, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET, int en4 = NOT_SET);
    
    /**  Motor hardware init function */
    int init() override;
    /** Motor disable function */
    void disable() override;
    /** Motor enable function */
    void enable() override;

    float dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]

    PhaseState phase_state[4]; //!< phase state (active / disabled)

    // hardware variables
    int pwm1A, pwm1B;  //!< phase 1A pwm pin number
    int pwm2A, pwm2B; 
    int pwm3A, pwm3B; 
    int pwm4A, pwm4B; 
    int enable_pin1; //!< enable pin number phase 1
    int enable_pin2; //!< enable pin number phase 2
    int enable_pin3; //!< enable pin number phase 3
    int enable_pin4; //!< enable pin number phase 4

      /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua phase A voltage
     * @param Ub phase B voltage
    */
    void setPwm(float Ua, float Ub) override;


  private:
        
};



#endif
