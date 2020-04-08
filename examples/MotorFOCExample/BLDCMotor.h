#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "Encoder.h"

// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _120_D2R 2.09439510239

enum ControlType{
  voltage,
  velocity,
  velocity_ultra_slow,
  angle
};


enum DriverType{
  bipolar,    // L6234
  unipolar    // HMBGC
};

// PI strucutre
struct PI_s{
  float K;
  float Ti;
  long timestamp;
  float uk_1, ek_1;
};

/**
 BLDC motor class
*/
class BLDCMotor
{
  public:
    BLDCMotor(int phA,int phB,int phC,int pp, int en = 0);
  	void init(DriverType type = DriverType::bipolar);
  	void disable();
    void enable();

    void linkEncoder(Encoder* enc);
    
    // Set phase voltages using FOC
    void setVoltage(float Uq);
    // Set referent velocity PI
    void setVelocity(float vel);
    void setVelocityUltraSlow(float vel);
    // Set referent position P+PI
    void setPosition(float pos);
    
    void initFOC();
    // iterative method updating motor angles and velocity measurement
    void loopFOC();
    void move(float target);
    

    // variables
  	int pwmA;
  	int pwmB;
  	int pwmC;
    int enable_pin;

    int pole_pairs;

    ControlType controller;
  	float elctric_angle;
  	float shaft_velocity;
  	float shaft_angle;

    float shaft_velocity_sp;
    float shaft_angle_sp;
    float voltage_q;

    // Power supply woltage
    float U_max;
    // maximum angular velocity to be used for positioning 
    float velocity_max;

    PI_s PI_velocity;
    PI_s PI_velocity_ultra_slow;
    PI_s P_angle;
  	
    Encoder* encoder;
  	

  private:
    //Encoder alignment to electrical 0 angle
    void alignEncoder();
    /** State calculation methods */
    //Shaft angle calculation
    float shaftAngle();
    //Shaft velocity calculation
    float shaftVelocity();
    
    //Electrical angle calculation
    float electricAngle(float shaftAngle);
    //Set phase voltaget to pwm output
    void setPwm(int pinPwm, float U);
    
    /** FOC methods */
    //Method using FOC to set Uq to the motor at the optimal angle
    void setPhaseVoltage(double Uq, double angle_el);
    void setPhaseVoltageUnipolar(double Uq, double angle_el);
    void setPhaseVoltageBipolar(double Uq, double angle_el);
    
    /** Utility funcitons */
    //normalizing radian angle to [0,2PI]
    double normalizeAngle(double angle);
    //Reference low pass filter 
    float filterLP(float u);
    
    /** Motor control functions */
    float velocityPI(float ek);
    float velocityUltraSlowPI(float ek);
    float positionP(float ek);
    
    float Ua,Ub,Uc;
    float	Ualpha,Ubeta;
    DriverType driver_type;
};


/*
  High PWM frequency
*/
void setPwmFrequency(int pin);

#endif
