#ifndef STEPPER_DRIVER_4PWM_h
#define STEPPER_DRIVER_4PWM_h

#include "../common/base_classes/StepperDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 4 pwm stepper driver class
*/
class StepperDriver4PWM: public StepperDriver
{
  public:
    /**
      StepperMotor class constructor
      @param ph1A 1A phase pwm pin
      @param ph1B 1B phase pwm pin
      @param ph2A 2A phase pwm pin
      @param ph2B 2B phase pwm pin
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
    */
    StepperDriver4PWM(int ph1A,int ph1B,int ph2A,int ph2B, int en1 = NOT_SET, int en2 = NOT_SET);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwm1A; //!< phase 1A pwm pin number
  	int pwm1B; //!< phase 1B pwm pin number
  	int pwm2A; //!< phase 2A pwm pin number
    int pwm2B; //!< phase 2B pwm pin number
    int enable_pin1; //!< enable pin number phase 1
    int enable_pin2; //!< enable pin number phase 2

    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua phase A voltage
     * @param Ub phase B voltage
    */
    void setPwm(float Ua, float Ub) override;


    /** 
     * Set phase voltages to the hardware. 
     * > Only possible is the driver has separate enable pins for both phases! 
     * 
     * @param sa phase A state : active / disabled ( high impedance )
     * @param sb phase B state : active / disabled ( high impedance )
    */
    virtual void setPhaseState(PhaseState sa, PhaseState sb) override;

  private:
        
};


#endif
