#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

#include "Arduino.h"
#include "FOCDriver.h"

class StepperDriver: public FOCDriver{
    public:
        
        /** 
         * Set phase voltages to the hardware 
         * 
         * @param Ua phase A voltage
         * @param Ub phase B voltage
        */
        virtual void setPwm(float Ua, float Ub) = 0;

        /**
         * Set phase state, enable/disable
         *
         * @param sc - phase A state : active / disabled ( high impedance )
         * @param sb - phase B state : active / disabled ( high impedance )
        */
        virtual void setPhaseState(PhaseState sa, PhaseState sb) = 0;
        
        /** driver type getter function */
        virtual DriverType type() override { return DriverType::Stepper; } ;
};

#endif