#ifndef BLDCDriver6PWM_h
#define BLDCDriver6PWM_h

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 6 pwm bldc driver class
*/
class BLDCDriver6PWM: public BLDCDriver
{
  public:
    /**
      BLDCDriver class constructor
      @param phA_h A phase pwm pin
      @param phA_l A phase pwm pin
      @param phB_h B phase pwm pin
      @param phB_l A phase pwm pin
      @param phC_h C phase pwm pin
      @param phC_l A phase pwm pin
      @param en enable pin (optional input)
    */
    BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en = NOT_SET);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwmA_h,pwmA_l; //!< phase A pwm pin number
  	int pwmB_h,pwmB_l; //!< phase B pwm pin number
  	int pwmC_h,pwmC_l; //!< phase C pwm pin number
    int enable_pin; //!< enable pin number

    float dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]

    PhaseState phase_state[3]; //!< phase state (active / disabled)


    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc) override;

    /** 
     * Set phase voltages to the harware 
     * 
     * @param sc - phase A state : active / disabled ( high impedance )
     * @param sb - phase B state : active / disabled ( high impedance )
     * @param sa - phase C state : active / disabled ( high impedance )
    */
    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

  private:
        
};


#endif
