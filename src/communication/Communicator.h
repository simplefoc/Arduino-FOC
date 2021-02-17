#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/CommunicationNode.h"

/**
 Serial communiactor
*/
class Communicator
{
  public:
    /**
     * Default constructor - setting all variabels to default values
     */
    Communicator(HardwareSerial &serial);

    void run();
    void addNode(CommunicationNode* node, char id = 0);

    // monitoring functions
    HardwareSerial* com_port; //!< Serial terminal variable if provided

  private:
    CommunicationNode* node_list[10]; //!< Dictionary of nodes
    char node_ids[10]; //!< Dictionary of nodes
    int node_count = 0;
    String received_chars = "";

};


#endif
