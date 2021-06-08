#ifndef STEPPER_DRIVER_2PWM_h
#define STEPPER_DRIVER_2PWM_h

#include "../common/base_classes/StepperDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 2 pwm stepper driver class
*/
class StepperDriver2PWM: public StepperDriver
{
  public:
    /**
      StepperMotor class constructor
      @param pwm1  PWM1 phase pwm pin
      @param in1   IN1A phase dir pin
      @param pwm2  PWM2 phase pwm pin
      @param in2   IN2A phase dir 
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
    */
    StepperDriver2PWM(int pwm1, int* in1, int pwm2, int* in2, int en1 = NOT_SET, int en2 = NOT_SET);
    
    /**
      StepperMotor class constructor
      @param pwm1  PWM1 phase pwm pin
      @param dir1  DIR1 phase dir pin
      @param pwm2  PWM2 phase pwm pin
      @param dir2  DIR2 phase dir pin
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
    */
    StepperDriver2PWM(int pwm1, int dir1, int pwm2, int dir2, int en1 = NOT_SET, int en2 = NOT_SET);

    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
    int pwm1; //!< phase 1 pwm pin number
    int dir1a; //!< phase 1 INA pin number
    int dir1b; //!< phase 1 INB pin number
    int pwm2; //!< phase 2 pwm pin number
    int dir2a; //!< phase 2 INA pin number
    int dir2b; //!< phase 2 INB pin number
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
