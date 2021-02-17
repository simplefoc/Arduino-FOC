#ifndef COMNODE_h
#define COMNODE_h

#include "Arduino.h"

/**
 *  Interface class for communication
 */
class CommunicationNode{
    public:
        virtual String communicate(String use_command)=0;
};


#endif