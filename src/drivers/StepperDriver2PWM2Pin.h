#ifndef STEPPER_DRIVER_2PWM_2PIN_h
#define STEPPER_DRIVER_2PWM_2PIN_h

#include "../common/base_classes/StepperDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 2 pwm stepper driver class
*/
class StepperDriver2PWM2Pin: public StepperDriver
{
  public:
    /**
      StepperMotor class constructor
      @param ph1PWM  PWM1 phase pwm pin
      @param ph1Dir  DIR1 phase dir pin
      @param ph2PWM  PWM2 phase pwm pin
      @param ph2Dir  DIR2 phase dir pin
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
    */
    StepperDriver2PWM2Pin(int ph1PWM, int ph1Dir,int ph2PWM, int ph2Dir, int en1 = NOT_SET, int en2 = NOT_SET);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
    int pwm1; //!< phase 1 pwm pin number
    int dir1; //!< phase 1 dir pin number
    int pwm2; //!< phase 2 pwm pin number
    int dir2; //!< phase 2 dir pin number
    int enable_pin1; //!< enable pin number phase 1
    int enable_pin2; //!< enable pin number phase 2

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
