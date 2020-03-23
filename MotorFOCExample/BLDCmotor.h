
#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"

#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _120_D2R 2.09439510239

// Power supply woltage
#define U_MAX 12

// dead-zone compensation 
// minimal voltage to move the motor
#define U_DEAD 1.5

// Velocity loop config
// PI contoroller constant
#define M_Kr 0.25
#define M_Ti 0.1

// position loop config
// P controller constant
#define M_P 70
// maximum angular velocity to be used for positioning 
#define W_MAX 20

// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )


// encoder measurement structure
struct encoder_s{
  long pulse_counter;       // current pulse counter
  long pulse_timestamp;     // last impulse timestamp in us
  int cpr;                  // impulse cpr
  int pinA, pinB;           // encoder hardware pins
  int A_active, B_active;   // current active states of A and B line
};


/**
 BLDC motor class
*/
class BLDCMotorEncoder
{
  public:
    BLDCMotorEncoder(int phA,int phB,int phC,int pp, int encA, int encB , int cpr);
    BLDCMotorEncoder(int phA,int phB,int phC,int pp, int encA, int encB , int cpr, int en);
  	void init();
  	void disableMotor();
    void enable_pinMotor();
    // Set phase voltages using FOC
    void setVoltage(float Uq);
    // Set referent velocity PI
    void setVelocity(float vel);
    // Set referent position P+PI
    void setPosition(float pos);
    
    // iterative method updating motor angles and velocity measurement
    void updateStates();

    // encoder measurement structure
    encoder_s encoder;
    //  Encoder interrupt callback functions
    //  enabling CPR=4xPPR behaviour
    // A channel
    void handleEncoderA();
    // B channel
    void handleEncoderB();
    
  	int pwmA;
  	int pwmB;
  	int pwmC;
    int enable_pin;
    int pole_pairs;
  	
  	float elctric_angle;
  	float shaft_velocity;
  	float shaft_angle;
  	
  	

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
    
    /** Utility funcitons */
    //normalizing radian angle to [0,2PI]
    double normalizeAngle(double angle);
    //Reference low pass filter 
    float filterLP(float u);
    
    /** Motor control functions */
    float velocityPI(float ek);
    float positionP(float ek);
    
    float Ua,Ub,Uc;
    float	Ualpha,Ubeta;
};


#endif
