#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Arduino.h"
#include "Sensor.h"
#include "CurrentSense.h"

#include "../time_utils.h"
#include "../foc_utils.h"
#include "../defaults.h"
#include "../pid.h"
#include "../lowpass_filter.h"


// monitoring bitmap
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL    0b0000010 // monitor velocity value
#define _MON_ANGLE  0b0000001 // monitor angle value

/**
 *  Motiron control type
 */
enum MotionControlType : uint8_t {
  torque            = 0x00,     //!< Torque control
  velocity          = 0x01,     //!< Velocity motion control
  angle             = 0x02,     //!< Position/angle motion control
  velocity_openloop = 0x03,
  angle_openloop    = 0x04
};

/**
 *  Motiron control type
 */
enum TorqueControlType : uint8_t { 
  voltage            = 0x00,     //!< Torque control using voltage
  dc_current         = 0x01,     //!< Torque control using DC current (one current magnitude)
  foc_current        = 0x02,     //!< torque control using dq currents
};

/**
 *  FOC modulation type
 */
enum FOCModulationType : uint8_t {
  SinePWM            = 0x00,     //!< Sinusoidal PWM modulation
  SpaceVectorPWM     = 0x01,     //!< Space vector modulation method
  Trapezoid_120      = 0x02,     
  Trapezoid_150      = 0x03,     
};



enum FOCMotorStatus : uint8_t {
  motor_uninitialized = 0x00,     //!< Motor is not yet initialized
  motor_initializing  = 0x01,     //!< Motor intiialization is in progress
  motor_uncalibrated  = 0x02,     //!< Motor is initialized, but not calibrated (open loop possible)
  motor_calibrating   = 0x03,     //!< Motor calibration in progress
  motor_ready         = 0x04,     //!< Motor is initialized and calibrated (closed loop possible)
  motor_error         = 0x08,     //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
  motor_calib_failed  = 0x0E,     //!< Motor calibration failed (possibly recoverable)
  motor_init_failed   = 0x0F,     //!< Motor initialization failed (not recoverable)
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
     * Function linking a motor and current sensing 
     * 
     * @param current_sense CurrentSense class wrapper for the FOC algorihtm to read the motor current measurements
     */
    void linkCurrentSense(CurrentSense* current_sense);


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


    /** 
     * Electrical angle calculation  
     */
    float electricalAngle();

    // state variables
    float target; //!< current target value - depends of the controller
  	float shaft_angle;//!< current motor angle
  	float electrical_angle;//!< current electrical angle
  	float shaft_velocity;//!< current motor velocity 
    float current_sp;//!< target current ( q current )
    float shaft_velocity_sp;//!< current target velocity
    float shaft_angle_sp;//!< current target angle
    DQVoltage_s voltage;//!< current d and q voltage set to the motor
    DQCurrent_s current;//!< current d and q current measured

    // motor configuration parameters
    float voltage_sensor_align;//!< sensor and motor align voltage parameter
    float velocity_index_search;//!< target velocity for index search 
    
    // motor physical parameters
    float	phase_resistance; //!< motor phase resistance
    int pole_pairs;//!< motor pole pairs number

    // limiting variables
    float voltage_limit; //!< Voltage limitting variable - global limit
    float current_limit; //!< Current limitting variable - global limit
    float velocity_limit; //!< Velocity limitting variable - global limit

    // motor status vairables
    int8_t enabled = 0;//!< enabled or disabled motor flag
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; //!< motor status
    
    // pwm modulation related variables
    FOCModulationType foc_modulation;//!<  parameter derterniming modulation algorithm
    int8_t modulation_centered = 1;//!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0


    // configuration structures
    TorqueControlType torque_controller; //!< parameter determining the torque control type
    MotionControlType controller; //!< parameter determining the control loop to be used

    // controllers and low pass filters
    PIDController PID_current_q{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the q current PID config
    PIDController PID_current_d{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the d current PID config
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};//!<  parameter determining the current Low pass filter configuration 
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};//!<  parameter determining the current Low pass filter configuration 
    PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT};//!< parameter determining the velocity PID configuration
    PIDController P_angle{DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM};	//!< parameter determining the position PID configuration 
    LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};//!<  parameter determining the velocity Low pass filter configuration 
    LowPassFilter LPF_angle{0.0};//!<  parameter determining the angle low pass filter configuration 
    unsigned int motion_downsample = DEF_MOTION_DOWNSMAPLE; //!< parameter defining the ratio of downsampling for move commad
    unsigned int motion_cnt = 0; //!< counting variable for downsampling for move commad

    // sensor related variabels
    float sensor_offset; //!< user defined sensor zero offset
    float zero_electric_angle = NOT_SET;//!< absolute zero electric angle - if available
    int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW

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
    unsigned int monitor_downsample = DEF_MON_DOWNSMAPLE; //!< show monitor outputs each monitor_downsample calls 
    // initial monitoring will display target, voltage, velocity and angle
    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; //!< Bit array holding the map of variables the user wants to monitor
   
    /** 
      * Sensor link:
      * - Encoder 
      * - MagneticSensor*
      * - HallSensor
    */
    Sensor* sensor; 
    /** 
      * CurrentSense link
    */
    CurrentSense* current_sense; 

    // monitoring functions
    Print* monitor_port; //!< Serial terminal variable if provided
  private:
    // monitor counting variable
    unsigned int monitor_cnt = 0 ; //!< counting variable
};


#endif
