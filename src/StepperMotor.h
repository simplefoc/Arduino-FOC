/**
 *  @file StepperMotor.h
 * 
 */

#ifndef StepperMotor_h
#define StepperMotor_h

#include "Arduino.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/StepperDriver.h"
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"

/**
 Stepper Motor class
*/
class StepperMotor: public FOCMotor
{
  public:
    /**
      StepperMotor class constructor
      @param pp  pole pair number 
     @param R  motor phase resistance - [Ohm]
     @param KV  motor KV rating (1/K_bemf) - rpm/V
     @param L  motor phase inductance - [H]
    */
    StepperMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);

    /**
     * Function linking a motor and a foc driver 
     * 
     * @param driver StepperDriver class implementing all the hardware specific functions necessary PWM setting
     */
    void linkDriver(StepperDriver* driver);

    /** 
      * StepperDriver link:
      * - 4PWM  - L298N for example
    */
    StepperDriver* driver; 


    // Methods implementing the FOCMotor interface

    /**  Motor hardware init function */
    int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;
    
    /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    void setPhaseVoltage(float Uq, float Ud, float angle_el) override;

    /**
     * Method estimating the Back EMF voltage based
     * based on the current velocity and KV rating
     * 
     * @param velocity Current motor velocity
     */
    float estimateBEMF(float velocity) override;

    // Methods overriding the FOCMotor default behavior
    
    /**
     * Measure resistance and inductance of a StepperMotor and print results to debug.
     * If a sensor is available, an estimate of zero electric angle will be reported too.
     * TODO: determine the correction factor
     * @param voltage The voltage applied to the motor
     * @returns 0 for success, >0 for failure
     */
    int characteriseMotor(float voltage){
      return FOCMotor::characteriseMotor(voltage, 1.0f);
    }

};


#endif
