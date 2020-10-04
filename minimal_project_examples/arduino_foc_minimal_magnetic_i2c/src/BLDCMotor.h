/**
 *  @file BLDCMotor.h
 * 
 */

#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "common/FOCMotor.h"
#include "common/foc_utils.h"
#include "common/hardware_utils.h"
#include "common/Sensor.h"
#include "common/defaults.h"

/**
 BLDC motor class
*/
class BLDCMotor: public FOCMotor
{
  public:
    /**
      BLDCMotor class constructor
      @param phA A phase pwm pin
      @param phB B phase pwm pin
      @param phC C phase pwm pin
      @param pp  pole pair number
      @param cpr counts per rotation number (cpm=ppm*4)
      @param en enable pin (optional input)
    */
    BLDCMotor(int phA,int phB,int phC,int pp, int en = NOT_SET);
    
    /**  Motor hardware init function */
  	void init();
    /** Motor disable function */
  	void disable();
    /** Motor enable function */
    void enable();

    /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     */  
    int initFOC( float zero_electric_offset = NOT_SET , Direction sensor_direction = Direction::CW); 
    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    void loopFOC();

    /**
     * Function executing the control loops set by the controller parameter of the BLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    void move(float target = NOT_SET);

    // hardware variables
  	int pwmA; //!< phase A pwm pin number
  	int pwmB; //!< phase B pwm pin number
  	int pwmC; //!< phase C pwm pin number
    int enable_pin; //!< enable pin number


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
    void setPwm(float Ua, float Ub, float Uc);
        
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
