#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H


#include "../foc_utils.h"

/**
 *  Current sensing abstract class defintion
 * Each current sensoring implementation needs to extend this interface
 */
class CurrentSense{
    public:

    virtual void init();
    virtual PhaseCurrent_s getCurrents();
    virtual DQCurrent_s getFOCCurrents(float angle_el);

};

#endif