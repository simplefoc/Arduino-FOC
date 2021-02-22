#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/pid.h"
#include "../common/lowpass_filter.h"
#include "commands.h"

// callback function pointer definiton
typedef void (* CommandCallback)(char*); //!< command callback function pointer

class Commander
{
  public:
    /**
     * Default constructor receiving a serial interface that it uses to output the values to
     * Also if the function run() is used it uses this serial instance to read the serial for user commands
     * 
     * @param serial - Serial com port instance
     */
    Commander(HardwareSerial &serial);
    Commander();

    /**
     * Function reading the serial port and firing callbacks that have been added to the commander 
     * once the user has requested them - when he sends the command  
     */ 
    void run();
    void run(HardwareSerial &reader);
    /**x
     * Function reading the string of user input and firing callbacks that have been added to the commander 
     * once the user has requested them - when he sends the command  
     * 
     * @param user_input - string of user inputs
     */ 
    void run(char* user_input);

    /**
     *  Function adding a callback to the coomander withe the command id
     * @param id         - char command letter
     * @param onCommand  - function pointer void function(char*) 
     */
    void add(char id , CommandCallback onCommand);

    // printing variables
    bool verbose = 1; //!< flag signaling that the commands should output user understanable text
    uint8_t decimal_places = 3; //!< number of decimal places to be used when displaying numbers

    // monitoring functions
    HardwareSerial* com_port = nullptr; //!< Serial terminal variable if provided
    
    /**
     */
    void motor(FOCMotor* motor, char* user_cmd);
    void lpf(LowPassFilter* lpf, char* user_cmd);
    void pid(PIDController* pid, char* user_cmd);
    void variable(float* value, char* user_cmd);

  private:
    // Subscribed command callback variables
    CommandCallback call_list[20];//!< array of command callback pointers - 20 is an arbitrary number
    char call_ids[20]; //!< added callback commands
    int call_count = 0;//!< number callbacks that are subscribed

    // helping variable for serial communication reading
    char received_chars[20] = {0}; //!< so far received user message - waiting for newline
    int rec_cnt = 0; //!< number of characters receives
    char cmd_scan_msg[2] = {CMD_SCAN,0}; //!< scan message to be sent to the nodes - should be solved differently maybe
        
    // serial printing functions
    /**
     *  print the string message only if verbose mode on
     *  @param message - message to be printed
     */
    void verbosePrint(const char* message); 
    /**
     *  Print the string message only if verbose mode on 
     *  - Function handling the case for strings defined by F macro
     *  @param message - message to be printed
     */
    void verbosePrint(const __FlashStringHelper *message);  
    /**
     *  print the numbers to the serial with desired decimal point number
     *  @param message - number to be printed
     *  @param newline - if needs lewline (1) otherwise (0)
     */
    void dump(const float number, const bool newline = 1);
    void dump(const int number, const bool newline = 1);
    void dump(const char* message, const bool newline = 1);
    void dump(const __FlashStringHelper *message, const bool newline = 1);
    void dump(const char message, const bool newline = 1);

    void printError();
};


#endif
