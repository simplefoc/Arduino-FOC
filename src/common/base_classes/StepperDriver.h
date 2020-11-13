#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

class StepperDriver{
    public:
        
        /** Initialise hardware */
        virtual int init();
        /** Enable hardware */
        virtual void enable();
        /** Disable hardware */
        virtual void disable();

        long pwm_frequency; //!< pwm frequency value in hertz
        float voltage_power_supply; //!< power supply voltage 
        float voltage_limit; //!< limiting voltage set to the motor
            
        /** 
         * Set phase voltages to the harware 
         * 
         * @param Ua phase A voltage
         * @param Ub phase B voltage
        */
        virtual void setPwm(float Ua, float Ub);
};

#endif