#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include "Arduino.h"


enum PhaseState : uint8_t {
  PHASE_OFF = 0, // both sides of the phase are off
  PHASE_ON = 1,  // both sides of the phase are driven with PWM, dead time is applied in 6-PWM mode
  PHASE_HI = 2,  // only the high side of the phase is driven with PWM (6-PWM mode only)
  PHASE_LO = 3,  // only the low side of the phase is driven with PWM (6-PWM mode only)
};


enum DriverType{
    Unknown=0,
    BLDC=1,
    Stepper=2
};

/**
 * FOC driver class
 */
class FOCDriver{
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

        bool initialized = false; //!< true if driver was successfully initialized
        void* params = 0; //!< pointer to hardware specific parameters of driver

        bool enable_active_high = true; //!< enable pin should be set to high to enable the driver (default is HIGH)

        /** get the driver type*/
        virtual DriverType type() = 0;
};

#endif
