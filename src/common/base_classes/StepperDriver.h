#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

#include "drivers/hardware_api.h"

class StepperDriver{
    public:
       /** Initialise hardware */
virtual int init() = 0;
/** Enable hardware */
virtual void enable() = 0;
/** Disable hardware */
virtual void disable() = 0;

long pwm_frequency; //!< pwm frequency value in hertz
float voltage_power_supply; //!< power supply voltage 
float voltage_limit; //!< limiting voltage set to the motor
    
bool initialized = false; // true if driver was successfully initialized
void* params = 0; // pointer to hardware specific parameters of driver

float dc_1a; //!< currently set duty cycle on phaseA
float dc_1b; //!< currently set duty cycle on phaseB
float dc_1c; //!< currently set duty cycle on phaseC
float dc_1d; //!< currently set duty cycle on phaseC


/** 
 * Set phase voltages to the hardware 
 * 
 * @param Ua phase A voltage
 * @param Ub phase B voltage
 */
virtual void setPwm(float Ua, float Ub) = 0;



};

#endif