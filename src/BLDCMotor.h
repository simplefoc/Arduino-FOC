/**
 *  @file BLDCMotor.h
 * 
 */

#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "FOCutils.h"
#include "Sensor.h"
#include "defaults.h"
#include "pid.h"
#include "lowpass_filter.h"


#define NOT_SET -12345.0

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
  SpaceVectorPWM //!< Space vector modulation method
};

// P controller structure
struct P_s{
  float P; //!< Proportional gain 
  long timestamp; //!< Last execution timestamp
};


/**
 BLDC motor class
*/
class BLDCMotor
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
    float Ua,Ub,Uc;//!< Current phase voltages Ua,Ub and Uc set to motor

    // motor configuration parameters
    float voltage_power_supply;//!< Power supply voltage
    float voltage_sensor_align;//!< sensor and motor align voltage parameter
    float velocity_index_search;//!< target velocity for index search 
    int pole_pairs;//!< Motor pole pairs number

    // limiting variables
    float voltage_limit; //!< Voltage limitting variable - global limit
    float velocity_limit; //!< Velocity limitting variable - global limit

    // configuration structures
    ControlType controller; //!< parameter determining the control loop to be used
    FOCModulationType foc_modulation;//!<  parameter derterniming modulation algorithm
    PIDController PID_velocity;//!< parameter determining the velocity PI configuration
    P_s P_angle;	//!< parameter determining the position P configuration 
    ExponentialMovingAverage LPF_velocity;//!<  parameter determining the velocity Lpw pass filter configuration 

    /** 
      * Sensor link:
      * - Encoder 
      * - MagneticSensor
    */
    Sensor* sensor; 

    float zero_electric_angle;//!<absolute zero electric angle - if available

    // FOC methods 
    /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    void setPhaseVoltage(float Uq, float angle_el);

    // monitoring functions
    Print* monitor_port; //!< Serial terminal variable if provided
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
    

  private:
    /** Sensor alignment to electrical 0 angle of the motor */
    int alignSensor();
    /** Motor and sensor alignment to the sensors absolute 0 angle  */
    int absoluteZeroAlign();
    
    /** Electrical angle calculation  */
    float electricAngle(float shaftAngle);

    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua phase A voltage
     * @param Ub phase B voltage
     * @param Uc phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc);
        
    // Utility functions 
    /** normalizing radian angle to [0,2PI]  */
    float normalizeAngle(float angle);
    /** determining if the enable pin has been provided  */
    int hasEnable();
    
    // Motion control functions 
    /**  Position P controller implementation */
    float positionP(float ek);

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

    
    // phase voltages 
    float	Ualpha,Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform


    // open loop variables
    long open_loop_timestamp;
};


#endif
