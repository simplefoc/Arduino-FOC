#ifndef FOCDRIVER_H
#define FOCDRIVER_H

class FOCDriver{
    public:
        /** Initialise hardware */
        virtual void init();
        /** Enable hardware */
        virtual void enable();
        /** Disable hardware */
        virtual void disable();


        /** Set the pwm frequency */ 
        virtual void setPwmFrequency(long frequency);
};

#endif