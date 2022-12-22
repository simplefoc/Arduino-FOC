#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

#include "BLDCDriver.h"
#include "../foc_utils.h"

/**
 *  Current sensing abstract class defintion
 * Each current sensoring implementation needs to extend this interface
 */
class CurrentSense{
    public:

    /**
     *  Function intialising the CurrentSense class
     *   - All the necessary intialisations of adc and sync should be implemented here
     *   
     * @returns -  0 - for failure &  1 - for success 
     */
    virtual int init() = 0;
    
    /**
     * Linking the current sense with the motor driver
     * Only necessary if synchronisation in between the two is required
     */
    void linkDriver(BLDCDriver *driver);

    // variables
    bool skip_align = false; //!< variable signaling that the phase current direction should be verified during initFOC()
    
    BLDCDriver* driver = nullptr; //!< driver link
    bool initialized = false; // true if current sense was successfully initialized   
    void* params = 0; //!< pointer to hardware specific parameters of current sensing
    
    /**
     * Function intended to verify if:
     *   - phase current are oriented properly 
     *   - if their order is the same as driver phases
     * 
     * This function corrects the alignment errors if possible ans if no such thing is needed it can be left empty (return 1)
     * @returns -  0 - for failure &  positive number (with status) - for success 
     */
    virtual int driverAlign(float align_voltage) = 0;

    /**
     *  Function rading the phase currents a, b and c
     *   This function will be used with the foc control throught the function 
     *   CurrentSense::getFOCCurrents(electrical_angle)
     *   - it returns current c equal to 0 if only two phase measurements available
     * 
     *  @return PhaseCurrent_s current values
     */
    virtual PhaseCurrent_s getPhaseCurrents() = 0;
    /**
     * Function reading the magnitude of the current set to the motor
     *  It returns the abosolute or signed magnitude if possible
     *  It can receive the motor electrical angle to help with calculation
     *  This function is used with the current control  (not foc)
     *  
     * @param angle_el - electrical angle of the motor (optional) 
     */
    virtual float getDCCurrent(float angle_el = 0);

    /**
     * Function used for FOC contorl, it reads the DQ currents of the motor 
     *   It uses the function getPhaseCurrents internally
     * 
     * @param angle_el - motor electrical angle
     */
    DQCurrent_s getFOCCurrents(float angle_el);


};

#endif