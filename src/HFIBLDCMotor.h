#ifndef HFIBLDCMotor_h
#define HFIBLDCMotor_h

#include "Arduino.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/Sensor.h"
#include "common/base_classes/BLDCDriver.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"

/**
 BLDC motor class
*/
class HFIBLDCMotor: public FOCMotor
{
  public:

    float Ld = 2200e-6f;
    float Lq = 3100e-6f;

    float hfi_gain1 = 750.0f * _2PI;
    float hfi_gain2 = 5.0f * _2PI;

    bool hfi_firstcycle = true;
    bool hfi_on = false;
    bool hfi_high = false;
    float hfi_v = 4.0f;
    uint8_t hfi_mode = 0;
    float hfi_curangleest = 0;
    float hfi_error = 0;
    float hfi_int;
    float hfi_out;
    float hfi_full_turns=0;
    float error_saturation_limit = 0.30f;

    float Ts = 1.0f/60000.0f;
    float Ts_L = Ts * ( 1 / Lq - 1 / Ld );
    float current_bandwidth = 300;

    DQCurrent_s current_meas;
    DQCurrent_s current_high;
    DQCurrent_s current_low;
    DQCurrent_s delta_current;
    DQCurrent_s current_setpoint;

    void process_hfi();

    /**
     BLDCMotor class constructor
     @param pp pole pairs number
     @param R  motor phase resistance - [Ohm]
     @param KV  motor KV rating (1/K_bemf) - rpm/V
     @param L  motor phase inductance - [H]
     */ 
    HFIBLDCMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
    
    /**
     * Function linking a motor and a foc driver 
     * 
     * @param driver BLDCDriver class implementing all the hardware specific functions necessary PWM setting
     */
    virtual void linkDriver(BLDCDriver* driver);

    /** 
      * BLDCDriver link:
      * - 3PWM 
      * - 6PWM 
    */
    BLDCDriver* driver; 
    
    /**  Motor hardware init function */
  	void init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     */  
    int initFOC() override;
    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    void loopFOC() override;

    /**
     * Function executing the control loops set by the controller parameter of the HFIBLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    void move(float target = NOT_SET) override;
    
    float Ua, Ub, Uc;//!< Current phase voltages Ua,Ub and Uc set to motor
    float	Ualpha, Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

  /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    void setPhaseVoltage(float Uq, float Ud, float angle_el) override;

  private:
    // FOC methods 

    /** Sensor alignment to electrical 0 angle of the motor */
    int alignSensor();
    /** Current sense and motor phase alignment */
    int alignCurrentSense();
    /** Motor and sensor alignment to the sensors absolute 0 angle  */
    int absoluteZeroSearch();

        
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
    // open loop variables
    long open_loop_timestamp;
};


#endif
