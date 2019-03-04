
#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "encoder.h"

#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _120_D2R 2.09439510239

#define U_MAX 12
#define W_MAX 20


#define M_Kr 0.25
#define M_Ti 0.01
#define M_P 70



/**
 BLDC motor class
*/
class BLDCMotor
{
  public:
    BLDCMotor(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr);
  	void init();
  	void disableMotor();
    // Set phase voltages using FOC
    void setVoltage(float Uq);
    // Set referent velocity PI
    void setVelocity(float vel);
    // Set referent position P+PI
    void setPosition(float pos);
  
    long* _encoderPosition;
    
  	int _phA;
  	int _phB;
  	int _phC;
    int _encoder;
    int _encA;
    int _encB;
  	
  	float _angleElec;
  	float _shaftVel;
  	float _shaftAngle;
  	
  	int _pp;
  	int _cpr;
  	

  private:
    //Encoder alignment to electrical 0 angle
    void alignEncoder();
    /** State calculation methods */
    //Shaft angle calculation
    float ShaftAngle();
    //Shaft velocity calculation
    float ShaftVelocity(float shaftAngle);
    //Electrical angle calculation
    float ElectricAngle(float shaftAngle);
    //Update motor angles and velocities
    void updateStates();
    
    /** FOC methods */
    //Method using FOC to set Uq to the motor at the optimal angle
    void setPhaseVoltage(double Uq, double angle_el);
    
    /** Utility funcitons */
    //normalizing radian angle to [0,2PI]
    double normalizeAngle(double angle);
    //Reference low pass filter 
    float filterLP(float u);
    //Pahse voltage to PWM calculation
    int voltage2pwm(float Uph);
    
    /** Motor control functions */
    float velocityPI(float ek);
    float positionP(float ek);
    
    float Ua,Ub,Uc;
    float	Ualpha,Ubeta;
};


#endif
