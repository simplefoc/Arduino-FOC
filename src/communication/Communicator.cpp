#include "Communicator.h"



Communicator::Communicator(HardwareSerial& serial){
  com_port = &serial;
}

/**
	CurrentSense linking method
*/
void Communicator::addNode(CommunicationNode* _node, char id) {
  node_list[node_count] = _node;
  node_ids[node_count++] = id;
}

void Communicator::run(){
  // a string to hold incoming data
  while (com_port->available()) {
    // get the new byte:
    char inChar = (char)com_port->read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      // execute the user command
      char id = received_chars.charAt(0);
      for(int i=0; i<node_count; i++){
        if(id == node_ids[i]){
          com_port->println(node_list[i]->communicate(received_chars.substring(1)));
          break;
        }
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
}

