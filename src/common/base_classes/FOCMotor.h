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

#define MOT_ERR "ERR-MOT:"
#define MOT_WARN "WARN-MOT:"
#define MOT_DEBUG "MOT:"

#ifndef SIMPLEFOC_DISABLE_DEBUG
#define SIMPLEFOC_MOTOR_WARN(msg, ...)  \
      SimpleFOCDebug::print(MOT_WARN); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)

#define SIMPLEFOC_MOTOR_ERROR(msg, ...)  \
      SimpleFOCDebug::print(MOT_ERR); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)

#define SIMPLEFOC_MOTOR_DEBUG(msg, ...)  \
      SimpleFOCDebug::print(MOT_DEBUG); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)
      
#else
#define SIMPLEFOC_MOTOR_DEBUG(msg, ...)
#define SIMPLEFOC_MOTOR_ERROR(msg, ...)
#define SIMPLEFOC_MOTOR_WARN(msg, ...)
#endif

// monitoring bitmap
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL    0b0000010 // monitor velocity value
#define _MON_ANGLE  0b0000001 // monitor angle value

/**
 *  Motion control type
 */
enum MotionControlType : uint8_t {
  torque            = 0x00,     //!< Torque control
  velocity          = 0x01,     //!< Velocity motion control
  angle             = 0x02,     //!< Position/angle motion control
  velocity_openloop = 0x03,
  angle_openloop    = 0x04,
  angle_nocascade   = 0x05,     //!< Position/angle motion control without velocity cascade
  custom            = 0x06      //!< Custom control method - control method added by user
};

/**
 *  Motiron control type
 */
enum TorqueControlType : uint8_t { 
  voltage            = 0x00,     //!< Torque control using voltage
  dc_current         = 0x01,     //!< Torque control using DC current (one current magnitude)
  foc_current        = 0x02,     //!< torque control using dq currents
  estimated_current  = 0x03      //!< torque control using estimated current (provided motor parameters)
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


    // Methods that need to be implemented, defining the FOCMotor interface

    /**  Motor hardware init function */
  	virtual int init() = 0;
    /** Motor disable function */
  	virtual void disable()=0;
    /** Motor enable function */
    virtual void enable()=0;

    /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el)=0;
    
    /**
     * Estimation of the Back EMF voltage
     * 
     * @param velocity - current shaft velocity
     */
    virtual float estimateBEMF(float velocity){return 0.0f;};

    // Methods that have a default behavior but can be overriden if needed

    /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     * 
     * - If zero_electric_offset parameter is set the alignment procedure is skipped
     */  
    virtual int initFOC();

    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    virtual void loopFOC();

    /**
     * Function executing the control loops set by the controller.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    virtual void move(float target = NOT_SET);

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


    /**
     * Measure resistance and inductance of a motor and print results to debug.
     * If a sensor is available, an estimate of zero electric angle will be reported too.
     * @param voltage The voltage applied to the motor
     * @param correction_factor  Is 1.5 for 3 phase motors, because we measure over a series-parallel connection. TODO: what about 2 phase motors?
     * @returns 0 for success, >0 for failure
     */
    int characteriseMotor(float voltage, float correction_factor);

    /**
     * Auto-tune the current controller PID parameters based on desired bandwidth.
     * Uses a simple method that assumes a first order system and requires knowledge of
     * the motor phase resistance and inductance (if not set, the characteriseMotor function can be used).
     * 
     * @param bandwidth Desired closed-loop bandwidth in Hz.
     * @returns returns 0 for success, >0 for failure
     */
    int tuneCurrentController(float bandwidth);

    // state variables
    float target; //!< current target value - depends of the controller
    float feed_forward_velocity = 0.0f; //!< current feed forward velocity
  	float shaft_angle;//!< current motor angle
  	float electrical_angle;//!< current electrical angle
  	float shaft_velocity;//!< current motor velocity 
    float current_sp;//!< target current ( q current )
    float shaft_velocity_sp;//!< current target velocity
    float shaft_angle_sp;//!< current target angle
    DQVoltage_s voltage;//!< current d and q voltage set to the motor
    DQCurrent_s current;//!< current d and q current measured
    float voltage_bemf; //!< estimated backemf voltage (if provided KV constant)
    float	Ualpha, Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

    DQCurrent_s feed_forward_current;//!< current d and q current measured
    DQVoltage_s feed_forward_voltage;//!< current d and q voltage set to the motor

    // motor configuration parameters
    float voltage_sensor_align;//!< sensor and motor align voltage parameter
    float velocity_index_search;//!< target velocity for index search 
    
    // motor physical parameters
    float	phase_resistance; //!< motor phase resistance
    int pole_pairs;//!< motor pole pairs number
    float KV_rating; //!< motor KV rating
    float	phase_inductance; //!< motor phase inductance q axis - FOR BACKWARDS COMPATIBILITY
    DQ_s	axis_inductance{NOT_SET, NOT_SET}; //!< motor direct axis phase inductance

    // limiting variables
    float voltage_limit; //!< Voltage limiting variable - global limit
    float current_limit; //!< Current limiting variable - global limit
    float velocity_limit; //!< Velocity limiting variable - global limit

    // motor status vairables
    int8_t enabled = 0;//!< enabled or disabled motor flag
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; //!< motor status
    
    // pwm modulation related variables
    FOCModulationType foc_modulation;//!<  parameter determining modulation algorithm
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
    Direction sensor_direction = Direction::UNKNOWN; //!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped compared to CW. Set to UNKNOWN to set by calibration
    bool pp_check_result = false; //!< the result of the PP check, if run during loopFOC

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
    char monitor_start_char = '\0'; //!< monitor starting character 
    char monitor_end_char = '\0'; //!< monitor outputs ending character 
    char monitor_separator = '\t'; //!< monitor outputs separation character
    unsigned int  monitor_decimals = 4; //!< monitor outputs decimal places
    // initial monitoring will display target, voltage, velocity and angle
    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; //!< Bit array holding the map of variables the user wants to monitor
   
    /** 
      * Sensor link:
      * - Encoder 
      * - MagneticSensor*
      * - HallSensor
    */
    Sensor* sensor; 
    //!< CurrentSense link
    CurrentSense* current_sense; 

    // monitoring functions
    Print* monitor_port; //!< Serial terminal variable if provided

    //!< time between two loopFOC executions in microseconds
    uint32_t loopfoc_time_us = 0; //!< filtered loop times
    uint32_t move_time_us = 0; // filtered motion control times

    /**
     * Update limit values in controllers when changed
     * @param new_velocity_limit - new velocity limit value
     * 
     * @note Updates velocity limit in:
     *  - motor.velocity_limit
     *  - motor.P_angle.limit
     */
    void updateVelocityLimit(float new_velocity_limit);
    /**
     * Update limit values in controllers when changed
     * @param new_current_limit - new current limit value
     * 
     * @note Updates current limit in:
     *  - motor.current_limit
     *  - motor.PID_velocity.limit (if current control)
     */
    void updateCurrentLimit(float new_current_limit);
    /**
     * Update limit values in controllers when changed
     * @param new_voltage_limit - new voltage limit value
     * 
     * @note Updates voltage limit in:
     *  - motor.voltage_limit
     *  - motor.PID_current_q.limit
     *  - motor.PID_current_d.limit
     *  - motor.PID_velocity.limit (if voltage control)
     */
    void updateVoltageLimit(float new_voltage_limit);

    /**
     * Update torque control type and related controller limit values
     * @param new_torque_controller - new torque control type
     * 
     * @note Updates motor.torque_controller and motor.PID_velocity.limit
     */
    void updateTorqueControlType(TorqueControlType new_torque_controller);
    /**
     * Update motion control type and related target values
     * @param new_motion_controller - new motion control type
     * 
     * @note Updates the target value based on the new controller type
     * - if velocity control: target is set to 0rad/s
     * - if angle control: target is set to the current shaft_angle
     * - if torque control: target is set to 0V or 0A depending on torque control type
     */
    void updateMotionControlType(MotionControlType new_motion_controller);

    // Open loop motion control    
    /**
     * Function (iterative) generating open loop movement for target velocity
     * it uses voltage_limit variable
     * 
     * @param target_velocity - rad/s
     */
    float velocityOpenloop(float target_velocity);
    /**
     * Function (iterative) generating open loop movement towards the target angle
     * it uses voltage_limit and velocity_limit variables
     * 
     * @param target_angle - rad
     */
    float angleOpenloop(float target_angle);
  

    /**
     * Function setting a custom motion control method defined by the user
     * @note the custom control method has to be defined by the user and should follow the signature: float controlMethod(FOCMotor* motor)
     * @param controlMethod - pointer to the custom control method function defined by the user
     */
    void linkCustomMotionControl(float (*controlMethod)(FOCMotor* motor)){
      customMotionControlCallback = controlMethod;
    }

  protected:

    /**
     * Function udating loop time measurement
     * time between two loopFOC executions in microseconds
     * It filters the value using low pass filtering alpha = 0.1
     * @note - using _micros() function - be aware of its overflow every ~70 minutes
     */
    void updateLoopFOCTime(){
      updateTime(loopfoc_time_us, last_loopfoc_time_us, last_loopfoc_timestamp_us);
    }

    void updateMotionControlTime(){
      updateTime(move_time_us, last_move_time_us, last_move_timestamp_us);
    }

    /** Sensor alignment to electrical 0 angle of the motor */
    int alignSensor();
    /** Current sense and motor phase alignment */
    int alignCurrentSense();
    /** Motor and sensor alignment to the sensors absolute 0 angle  */
    int absoluteZeroSearch();
    
    uint32_t last_loopfoc_timestamp_us = 0; //!< timestamp of the last loopFOC execution in microseconds
    uint32_t last_loopfoc_time_us = 0; //!< last elapsed time of loopFOC in microseconds
    uint32_t last_move_timestamp_us = 0; //!< timestamp of the last move execution in microseconds
    uint32_t last_move_time_us = 0; //!< last elapsed time of move in microseconds
  private:
    // monitor counting variable
    unsigned int monitor_cnt = 0 ; //!< counting variable

    // time measuring function 
    // It filters the value using low pass filtering alpha = 0.1
    void updateTime(uint32_t& elapsed_time_filetered, uint32_t& elapsed_time, uint32_t& last_timestamp_us, float alpha = 0.1f){
      uint32_t now = _micros();
      elapsed_time = now - last_timestamp_us;
      elapsed_time_filetered = (1-alpha) * elapsed_time_filetered + alpha * elapsed_time;
      last_timestamp_us = now;
    }

    // open loop variables
    uint32_t open_loop_timestamp;
    
    // function pointer for custom control method
    float (*customMotionControlCallback)(FOCMotor* motor, float target) = nullptr;
    
};


#endif
