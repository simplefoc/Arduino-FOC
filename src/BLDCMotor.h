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


#define NOT_SET -12345.0

/**
 *  Motiron control type
 */
enum ControlType{
  voltage,//!< Torque control using voltage
  velocity,//!< Velocity motion control
  angle//!< Position/angle motion control
};

/**
 *  FOC modulation type
 */
enum FOCModulationType{
  SinePWM, //!< Sinusoidal PWM modulation
  SpaceVectorPWM //!< Space vector modulation method
};

/**
 *  PI controller structure
 */
struct PI_s{
  float P; //!< Proportional gain 
  float I; //!< Integral gain 
  float voltage_limit; //!< Voltage limit of the controller output
  float voltage_ramp;  //!< Maximum speed of change of the output value 
  long timestamp;  //!< Last execution timestamp
  float voltage_prev;  //!< last controller output value 
  float tracking_error_prev;  //!< last tracking error value
};

// P controller structure
struct P_s{
  float P; //!< Proportional gain 
  long timestamp; //!< Last execution timestamp
  float velocity_limit; //!< Velocity limit of the controller output
};

/**
 *  Low pass filter structure
 */
struct LPF_s{
  float Tf; //!< Low pass filter time constant
  long timestamp; //!< Last execution timestamp
  float prev; //!< filtered value in previous execution step 
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
     */  
    int initFOC(); 
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

    // configuration structures
    ControlType controller; //!< parameter determining the control loop to be used
    FOCModulationType foc_modulation;//!<  parameter derterniming modulation algorithm
    PI_s PI_velocity;//!< parameter determining the velocity PI configuration
    P_s P_angle;	//!< parameter determining the position P configuration 
    LPF_s LPF_velocity;//!<  parameter determining the velocity Lpw pass filter configuration 

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
    /** Set phase voltaget to pwm output */
    void setPwm(int pinPwm, float U);
        
    // Utility functions 
    /** normalizing radian angle to [0,2PI]  */
    float normalizeAngle(float angle);
    /** determining if the enable pin has been provided  */
    int hasEnable();
    
    // Motion control functions 
    /**
     * Generic PI controller function executing one step of a controller 
     * receives tracking error and PI_s structure and outputs the control signal
     * 
     * @param tracking_error Current error in between target value and mesasured value
     * @param controller PI_s structure containing all the necessary PI controller config and variables
     */
    float controllerPI(float tracking_error, PI_s &controller);
    /** Velocity PI controller implementation */
    float velocityPI(float tracking_error);
    /**  Position P controller implementation */
    float positionP(float ek);
    
    // phase voltages 
    float	Ualpha,Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

};


#endif
