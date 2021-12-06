#ifndef BLDCDRIVER_H
#define BLDCDRIVER_H

#include "Arduino.h"

class BLDCDriver{
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


        float dc_a; //!< currently set duty cycle on phaseA
        float dc_b; //!< currently set duty cycle on phaseB
        float dc_c; //!< currently set duty cycle on phaseC

        bool initialized = false; // true if driver was successfully initialized

        /**
         * Set phase voltages to the harware
         *
         * @param Ua - phase A voltage
         * @param Ub - phase B voltage
         * @param Uc - phase C voltage
        */
        virtual void setPwm(float Ua, float Ub, float Uc) = 0;

        /**
         * Set phase state, enable/disable
         *
         * @param sc - phase A state : active / disabled ( high impedance )
         * @param sb - phase B state : active / disabled ( high impedance )
         * @param sa - phase C state : active / disabled ( high impedance )
        */
        virtual void setPhaseState(int sa, int sb, int sc) = 0;
};

#endif
