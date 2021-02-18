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
    /**
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
     * Function setting the configuration parameters  of the motor, target value of the control loop
     * and outputing them to the monitoring port( if available ) :
     * - configure PID controller constants
     * - change motion control loops
     * - monitor motor variabels
     * - set target values
     * - check all the configuration values 
     * 
     * To check the config value just enter the command letter.
     * For example: 
     * - to read velocity PI controller P gain run: P
     * - to set velocity PI controller P gain  to 1.2 run: P1.2
     * 
     * To change the target value just enter a number in the terminal:
     * For example: 
     * - to change the target value to -0.1453 enter: -0.1453
     * - to get the current target value enter: V3 
     * 
     * List of commands:
     *  - P: velocity PI controller P gain
     *  - I: velocity PI controller I gain
     *  - L: velocity PI controller voltage limit
     *  - R: velocity PI controller voltage ramp
     *  - F: velocity Low pass filter time constant
     *  - K: angle P controller P gain
     *  - N: angle P controller velocity limit
     *  - C: control loop 
     *    - 0: voltage 
     *    - 1: velocity 
     *    - 2: anglex
     *  - V: get motor variables
     *    - 0: currently set voltage
     *    - 1: current velocity
     *    - 2: current angle
     *    - 3: current target value
     *
     * - Look into the documentation (docs.simplefoc.com) for more information.
     * 
     * @param command String containing the user command
     * 
     * returns 0 for error or 1 for executed command
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
    void print(const float number, const bool newline = 0);

    void printError();

    void print(const int number, const bool newline);
    void print(const char* message, const bool newline);
    void print(const __FlashStringHelper *message, const bool newline);
    void print(const char message, const bool newline);
};


#endif
