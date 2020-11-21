#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Arduino.h"
#include "Sensor.h"
#include "BLDCDriver.h"

#include "../time_utils.h"
#include "../foc_utils.h"
#include "../defaults.h"
#include "../pid.h"
#include "../lowpass_filter.h"


/**
 *  Motiron control type
 */
enum ControlType{
  voltage,//!< Torque control using voltage
  velocity,//!< Velocity motion control
  angle,//!< Position/angle motion control
  velocity_openloop,
  angle_openloop
};

/**
 *  FOC modulation type
 */
enum FOCModulationType{
  SinePWM, //!< Sinusoidal PWM modulation
  SpaceVectorPWM, //!< Space vector modulation method
  Trapezoid_120,
  Trapezoid_150
};

/**
 Generic motor class
*/
class FOCMotor
{
  public:
    /**
     * Default constructor - setting all variabels to default values
     */
    FOCMotor();

    /**  Motor hardware init function */
  	virtual void init()=0;
    /** Motor disable function */
  	virtual void disable()=0;
    /** Motor enable function */
    virtual void enable()=0;

    /**
     * Function linking a motor and a sensor 
     * 
     * @param sensor Sensor class  wrapper for the FOC algorihtm to read the motor angle and velocity
     */
    void linkSensor(Sensor* sensor);


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
    virtual int initFOC( float zero_electric_offset = NOT_SET , Direction sensor_direction = Direction::CW)=0; 
    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    virtual void loopFOC()=0;
    /**
     * Function executing the control loops set by the controller parameter of the BLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    virtual void move(float target = NOT_SET)=0;

    // State calculation methods 
    /** Shaft angle calculation in radians [rad] */
    float shaftAngle();
    /** 
     * Shaft angle calculation function in radian per second [rad/s]
     * It implements low pass filtering
     */
    float shaftVelocity();

    // state variables
    float target; //!< current target value - depends of the controller
  	float shaft_angle;//!< current motor angle
  	float shaft_velocity;//!< current motor velocity 
    float shaft_velocity_sp;//!< current target velocity
    float shaft_angle_sp;//!< current target angle
    float voltage_q;//!< current voltage u_q set
    float voltage_d;//!< current voltage u_d set

    // motor configuration parameters
    float voltage_sensor_align;//!< sensor and motor align voltage parameter
    float velocity_index_search;//!< target velocity for index search 
    int pole_pairs;//!< Motor pole pairs number

    // limiting variables
    float voltage_limit; //!< Voltage limitting variable - global limit
    float velocity_limit; //!< Velocity limitting variable - global limit

    float zero_electric_angle;//!<absolute zero electric angle - if available

    // configuration structures
    ControlType controller; //!< parameter determining the control loop to be used
    FOCModulationType foc_modulation;//!<  parameter derterniming modulation algorithm
    PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_U_RAMP,DEF_POWER_SUPPLY};//!< parameter determining the velocity PI configuration
    PIDController P_angle{DEF_P_ANGLE_P,0,0,1e10,DEF_VEL_LIM};	//!< parameter determining the position P configuration 
    LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};//!<  parameter determining the velocity Lpw pass filter configuration 

    /**
     * Function providing BLDCMotor class with the 
     * Serial interface and enabling monitoring mode
     * 
     * @param serial Monitoring Serial class reference
     */
    void useMonitoring(Print &serial);

    /**
     * Utility function intended to be used with serial plotter to monitor motor variables
     * significantly slowing the execution down!!!!
     */
    void monitor();

     /**
     * Function setting the configuration parameters  of the motor, target value of the control loop
     * and outputing them to the monitoring port( if available ) :
     * - configure PID controller constants
     * - change motion control loops
     * - monitor motor variabels
     * - set target values
     * - check all the configuration values 
     * 
     * To check the config value just enter the command letter.
     * For example: 
     * - to read velocity PI controller P gain run: P
     * - to set velocity PI controller P gain  to 1.2 run: P1.2
     * 
     * To change the target value just enter a number in the terminal:
     * For example: 
     * - to change the target value to -0.1453 enter: -0.1453
     * - to get the current target value enter: V3 
     * 
     * List of commands:
     *  - P: velocity PI controller P gain
     *  - I: velocity PI controller I gain
     *  - L: velocity PI controller voltage limit
     *  - R: velocity PI controller voltage ramp
     *  - F: velocity Low pass filter time constant
     *  - K: angle P controller P gain
     *  - N: angle P controller velocity limit
     *  - C: control loop 
     *    - 0: voltage 
     *    - 1: velocity 
     *    - 2: angle
     *  - V: get motor variables
     *    - 0: currently set voltage
     *    - 1: current velocity
     *    - 2: current angle
     *    - 3: current target value
     *
     * - Look into the documentation (docs.simplefoc.com) for more information.
     * 
     * @param command String containing the user command
     * 
     * returns 0 for error or 1 for executed command
     */
    int command(String command);
    
    /** 
      * Sensor link:
      * - Encoder 
      * - MagneticSensor*
      * - HallSensor
    */
    Sensor* sensor; 
    // monitoring functions
    Print* monitor_port; //!< Serial terminal variable if provided
};


#endif
