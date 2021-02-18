#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/pid.h"
#include "../common/lowpass_filter.h"
#include "commands.h"


typedef void (* CommandCallback)(char*);

class Commander
{
  public:
    /**
     * Default constructor - setting all variabels to default values
     */
    Commander(HardwareSerial &serial);

    void run();
    void run(char* user_input);

    void add(char id , CommandCallback onCommand);

    bool verbose = 1;
    uint8_t decimal_places = 3;

    // monitoring functions
    HardwareSerial* com_port; //!< Serial terminal variable if provided
    
    void motor(FOCMotor* motor, char* user_cmd);
    void lpf(LowPassFilter* lpf, char* user_cmd);
    void pid(PIDController* pid, char* user_cmd);
    void variable(float* value, char* user_cmd);

  private:
    CommandCallback call_list[20];
    char call_ids[20]; //!< Dictionary of nodes
    int call_count = 0;
    char received_chars[20] = {0};
    int rec_cnt = 0;
    char cmd_scan_msg[2] = {CMD_SCAN,0};
        
    void verbosePrint(const char* message);
    void verbosePrint(const __FlashStringHelper *message);
    void printNumber(const float number, const bool newline = 0);

};


#endif
