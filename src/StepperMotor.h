/**
 *  @file StepperMotor.h
 * 
 */

#ifndef StepperMotor_h
#define StepperMotor_h

#include "Arduino.h"
#include "common/FOCMotor.h"
#include "common/foc_utils.h"
#include "common/hardware_utils.h"
#include "common/Sensor.h"
#include "common/defaults.h"

/**
 Stepper Motor class
*/
class StepperMotor: public FOCMotor
{
  public:
    /**
      StepperMotor class constructor
      @param ph1A 1A phase pwm pin
      @param ph1B 1B phase pwm pin
      @param ph2A 2A phase pwm pin
      @param ph2B 2B phase pwm pin
      @param pp  pole pair number - cpr counts per rotation number (cpm=ppm*4)
      @param en1 enable pin phase 1 (optional input)
      @param en2 enable pin phase 2 (optional input)
    */
    StepperMotor(int ph1A,int ph1B,int ph2A,int ph2B,int pp, int en1 = NOT_SET, int en2 = NOT_SET);
    
    /**  Motor hardware init function */
  	void init(long pwm_frequency = NOT_SET) override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     * 
     * - If zero_electric_offset parameter is set the alignment procedure is skipped
     * 
     * @param zero_electric_offset value of the sensors absolute position electrical offset in respect to motor's electrical 0 position.
     * @param sensor_direction  sensor natural direction - default is CW
     *
     */  
    int initFOC( float zero_electric_offset = NOT_SET , Direction sensor_direction = Direction::CW) override;
    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    void loopFOC() override;
    /**
     * Function executing the control loops set by the controller parameter of the BLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    void move(float target = NOT_SET) override;

    // hardware variables
  	int pwm1A; //!< phase 1A pwm pin number
  	int pwm1B; //!< phase 1B pwm pin number
  	int pwm2A; //!< phase 2A pwm pin number
    int pwm2B; //!< phase 2B pwm pin number
    int enable_pin1; //!< enable pin number phase 1
    int enable_pin2; //!< enable pin number phase 2

  private:
  
    // FOC methods 
    /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    void setPhaseVoltage(float Uq, float angle_el);

    /** Sensor alignment to electrical 0 angle of the motor */
    int alignSensor();
    /** Motor and sensor alignment to the sensors absolute 0 angle  */
    int absoluteZeroAlign();
    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua phase A voltage
     * @param Ub phase B voltage
     * @param Uc phase C voltage
    */
    void setPwm(float Ualpha, float Ubeta);
        
    // Open loop motion control    
    /**
     * Function (iterative) generating open loop movement for target velocity
     * it uses voltage_limit variable
     * 
     * @param target_velocity - rad/s
     */
    void velocityOpenloop(float target_velocity);
    /**
     * Function (iterative) generating open loop movement towards the target angle
     * it uses voltage_limit and velocity_limit variables
     * 
     * @param target_angle - rad
     */
    void angleOpenloop(float target_angle);
    // open loop variables
    long open_loop_timestamp;
};


#endif
