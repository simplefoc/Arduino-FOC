#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "Encoder.h"
#include "FOCutils.h"

// default configuration values
// power supply voltage
#define DEF_POWER_SUPPLY 12.0
// velocity PI controller params
#define DEF_PI_VEL_K 0.5
#define DEF_PI_VEL_TI 0.01
#define DEF_PI_VEL_U_RAMP 300
// ultra slow velocity PI params
#define DEF_PI_VEL_US_K 60.0
#define DEF_PI_VEL_US_TI 100.0
#define DEF_PI_VEL_US_U_RAMP 300
// angle P params
#define DEF_P_ANGLE_K 20
// angle velocity limit default
#define DEF_P_ANGLE_VEL_LIM 20
// index search velocity
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1
// velocity PI controller params for index search
#define DEF_PI_VEL_INDEX_K 0.5
#define DEF_PI_VEL_INDEX_TI 0.01
#define DEF_PI_VEL_INDEX_U_RAMP 100

// controller type configuration enum
enum ControlType{
  voltage,
  velocity,
  velocity_ultra_slow,
  angle
};

// PI controller strucutre
struct PI_s{
  float K;
  float Ti;
  long timestamp;
  float voltage_prev, tracking_error_prev;
  float voltage_limit;
  float voltage_ramp;
};

// P controller structure
struct P_s{
  float K;
  long timestamp;
  float voltage_prev, tracking_error_prev;
  float velocity_limit;
};

/**
 BLDC motor class
*/
class BLDCMotor
{
  public:
    BLDCMotor(int phA,int phB,int phC,int pp, int en = 0);
    // change driver state
  	void init();
  	void disable();
    void enable();
    // connect encoder
    void linkEncoder(Encoder* enc);

    //  initilise FOC  
    int initFOC();
    // iterative method updating motor angles and velocity measurement
    void loopFOC();
    // iterative control loop defined by controller 
    void move(float target);
    

    // hardware variables
  	int pwmA;
  	int pwmB;
  	int pwmC;
    int enable_pin;
    int pole_pairs;



    /** State calculation methods */
    //Shaft angle calculation
    float shaftAngle();
    //Shaft velocity calculation
    float shaftVelocity();

    // state variables
    // current elelctrical angle
  	float elctric_angle;
    // current motor angle
  	float shaft_angle;
    // current motor velocity 
  	float shaft_velocity;
    // current target velocity
    float shaft_velocity_sp;
    // current target angle
    float shaft_angle_sp;
    // current voltage u_q set
    float voltage_q;

    // Power supply woltage
    float voltage_power_supply;

    // configuraion structures
    ControlType controller;
    PI_s PI_velocity;
    PI_s PI_velocity_ultra_slow;
    P_s P_angle;
    PI_s PI_velocity_index_search;
  	
    // encoder link
    Encoder* encoder;
    // index electric angle - if available
    float index_electric_angle;
    // index search velocity
    float index_search_velocity;

    /** FOC methods */
    //Method using FOC to set Uq to the motor at the optimal angle
    void setPhaseVoltage(float Uq, float angle_el);


    // debugging 
    void useDebugging(Print &print);
    Print* debugger;

  private:
    //Encoder alignment to electrical 0 angle
    int alignEncoder();
    int indexSearch();
    
    //Electrical angle calculation
    float electricAngle(float shaftAngle);
    //Set phase voltaget to pwm output
    void setPwm(int pinPwm, float U);
        
    /** Utility funcitons */
    //normalizing radian angle to [0,2PI]
    float normalizeAngle(float angle);
    //Reference low pass filter 
    float filterLP(float u);
    
    /** Motor control functions */
    float controllerPI(float tracking_error, PI_s &controller);
    float velocityPI(float tracking_error);
    float velocityUltraSlowPI(float vel);
    float velocityIndexSearchPI(float tracking_error);
    float positionP(float ek);
    
    // phase voltages 
    float	Ualpha,Ubeta;
    float Ua,Ub,Uc;

    // velocity ultra slow angle 
    float ultraslow_estimated_angle;

};


#endif
