/**
 *  @file HybridStepperMotor.h
 *
 */

#ifndef HybridStepperMotor_h
#define HybridStepperMotor_h

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
class HybridStepperMotor : public FOCMotor
{
public:
    /**
    * HybridStepperMotor class constructor
    *  @param pp  pole pair number
    *  @param R  motor phase resistance - [Ohm]
    *  @param KV  motor KV rating (1/K_bemf) - rpm/V
    *  @param Lq  motor q-axis inductance - [H]
    *  @param Ld  motor d-axis inductance - [H]
    */
    HybridStepperMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L_q = NOT_SET, float L_d = NOT_SET);

    /**
     * Function linking a motor and a foc driver
     *
     * @param driver BLDCDriver handle for hardware peripheral control
     */
    void linkDriver(BLDCDriver *driver);

    BLDCDriver* driver; //!< BLDCDriver instance

    float Ua, Ub, Uc; //!< Phase voltages used for inverse Park and Clarke transform

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
     * Measure resistance and inductance of a HybridStepperMotor and print results to debug.
     * If a sensor is available, an estimate of zero electric angle will be reported too.
     * TODO: determine the correction factor
     * @param voltage The voltage applied to the motor
     * @returns 0 for success, >0 for failure
     */
    int characteriseMotor(float voltage){
      // correction factor is not correct here, 
      // we need to add the driver reistance to compensate for the voltage drop on it, but it is a good starting point for now
      return FOCMotor::characteriseMotor(voltage, 1.0f); 
    };
    
    /**
     * Link the currentsense
     */
    void linkCurrentSense(CurrentSense* current_sense);

    
};

#endif
