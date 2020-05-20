#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "FOCutils.h"
#include "Sensor.h"

// default configuration values
// power supply voltage
#define DEF_POWER_SUPPLY 12.0
// velocity PI controller params
#define DEF_PI_VEL_P 0.5
#define DEF_PI_VEL_I 10
#define DEF_PI_VEL_U_RAMP 300
// angle P params
#define DEF_P_ANGLE_P 20
// angle velocity limit default
#define DEF_P_ANGLE_VEL_LIM 20
// index search velocity
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1
// velocity PI controller params for index search
#define DEF_PI_VEL_INDEX_P 1
#define DEF_PI_VEL_INDEX_I 10
#define DEF_PI_VEL_INDEX_U_RAMP 100
// velocity filter time constant
#define DEF_VEL_FILTER_Tf 0.005

// controller type configuration enum
enum ControlType{
  voltage,
  velocity,
  angle
};

// FOC Type
enum FOCModulationType{
  SinePWM,
  SpaceVectorPWM
};

// PI controller structure
struct PI_s{
  float P;
  float I;
  long timestamp;
  float voltage_prev, tracking_error_prev;
  float voltage_limit;
  float voltage_ramp;
};

// P controller structure
struct P_s{
  float P;
  long timestamp;
  float voltage_prev, tracking_error_prev;
  float velocity_limit;
};

// flow pass filter structure
struct LPF_s{
  float Tf;
  long timestamp;
  float prev;
};

/**
 BLDC motor class
*/
class BLDCMotor
{
  public:
    /*
      BLDCMotor( int phA, int phB, int phC, int pp , int cpr, int en)
      - phA, phB, phC - motor A,B,C phase pwm pins
      - pp            - pole pair number
      - cpr           - counts per rotation number (cpm=ppm*4)
      - enable pin    - (optional input)
    */
    BLDCMotor(int phA,int phB,int phC,int pp, int en = 0);
    // change driver state
  	void init();
  	void disable();
    void enable();
    // connect sensor
    void linkSensor(Sensor* _sensor);

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

    // Power supply voltage
    float voltage_power_supply;

    // configuration structures
    ControlType controller;
    FOCModulationType foc_modulation;
    PI_s PI_velocity;
    PI_s PI_velocity_index_search;  	
    P_s P_angle;	
    LPF_s LPF_velocity;

    // sensor link:
    // - Encoder 
    // - MagneticSensor
    Sensor* sensor;
    // absolute zero electric angle - if available
    float zero_electric_angle;
    // index search velocity
    float index_search_velocity;

    /** FOC methods */
    //Method using FOC to set Uq to the motor at the optimal angle
    void setPhaseVoltage(float Uq, float angle_el);

    // debugging 
    void useDebugging(Print &print);
    void monitor();
    Print* debugger;
    
    float Ua,Ub,Uc;

  private:
    //Sensor alignment to electrical 0 angle
    int alignSensor();
    //Motor and sensor alignment to the sensors absolute 0 angle
    int absoluteZeroAlign();
    
    //Electrical angle calculation
    float electricAngle(float shaftAngle);
    //Set phase voltaget to pwm output
    void setPwm(int pinPwm, float U);
        
    /** Utility functions */
    //normalizing radian angle to [0,2PI]
    float normalizeAngle(float angle);
    // determining if the enable pin has been provided
    int hasEnable();
    
    /** Motor control functions */
    float controllerPI(float tracking_error, PI_s &controller);
    float velocityPI(float tracking_error);
    float velocityIndexSearchPI(float tracking_error);
    float positionP(float ek);
    
    // phase voltages 
    float	Ualpha,Ubeta;

};


#endif
