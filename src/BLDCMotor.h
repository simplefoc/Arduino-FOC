#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "Encoder.h"

// default configuration values
// power supply voltage
#define DEF_POWER_SUPPLY 12.0
// velocity PI controller params
#define DEF_PI_VEL_K 0.5
#define DEF_PI_VEL_TI 0.01
// ultra slow velocity PI params
#define DEF_PI_VEL_US_K 60.0
#define DEF_PI_VEL_US_TI 100.0
// angle P params
#define DEF_P_ANGLE_K 20
// angle velocity limit default
#define DEF_P_ANGLE_VEL_LIM 20
// index search velocity
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1
// velocity PI controller params for index search
#define DEF_PI_VEL_INDEX_K 0.5
#define DEF_PI_VEL_INDEX_TI 0.01


// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239

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
  float uk_1, ek_1;
  float u_limit;
};

// P controller structure
struct P_s{
  float K;
  long timestamp;
  float uk_1, ek_1;
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
    float power_supply_voltage;

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
    void setPhaseVoltage(double Uq, double angle_el);


    // debugging 
    void useDebugging(Print &print);
    Print* debugger;

  private:
    //Encoder alignment to electrical 0 angle
    int alignEncoder();
    int indexSearch();
    /** State calculation methods */
    //Shaft angle calculation
    float shaftAngle();
    //Shaft velocity calculation
    float shaftVelocity();
    
    //Electrical angle calculation
    float electricAngle(float shaftAngle);
    //Set phase voltaget to pwm output
    void setPwm(int pinPwm, float U);
        
    /** Utility funcitons */
    //normalizing radian angle to [0,2PI]
    double normalizeAngle(double angle);
    //Reference low pass filter 
    float filterLP(float u);
    
    /** Motor control functions */
    float controllerPI(float ek, PI_s &controller);
    float velocityPI(float ek);
    float velocityUltraSlowPI(float vel);
    float velocityIndexSearchPI(float ek);
    float positionP(float ek);
    
    // phase voltages 
    float	Ualpha,Ubeta;
    float Ua,Ub,Uc;

    // velocity ultra slow angle 
    float ultraslow_estimated_angle;

};


/*
  High PWM frequency
*/
void setPwmFrequency(int pin);
// funciton implementing arduino blocking delay funciton
// regular delay funciton doesn't work with interrupts
void _delay(uint64_t ms);

#endif
