#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/pid.h"
#include "../common/lowpass_filter.h"
#include "commands.h"

// Commander verbose display to the user type
enum VerboseMode{
  nothing = 0,   // display nothing - good for monitoring
  on_request,    // display only on user request
  user_friendly  // display textual messages to the user
};


// callback function pointer definiton
typedef void (* CommandCallback)(char*); //!< command callback function pointer

/**
 * Commander class implementing string communication protocol based on IDvalue (ex AB5.321 - command id `A`, sub-command id `B`,value `5.321`)
 * 
 *  - This class can be used in combination with HardwareSerial instance which it would read and write 
 *    or it can be used to parse strings that have been received from the user outside this library
 *  - Commander class implements command protocol for few standard components of the SimpleFOC library
 *     - FOCMotor
 *     - PIDController
 *     - LowPassFilter
 *  - Commander also provides a very simple command > callback interface that enables user to 
 *    attach a callback function to certain command id - see function add()
 */
class Commander
{
  public:
    /**
     * Default constructor receiving a serial interface that it uses to output the values to
     * Also if the function run() is used it uses this serial instance to read the serial for user commands
     * 
     * @param serial - Serial com port instance
     */
    Commander(Stream &serial);
    Commander();

    /**
     * Function reading the serial port and firing callbacks that have been added to the commander 
     * once the user has requested them - when he sends the command  
     * 
     *  - It has default commands (the letters can be changed in the commands.h file)
     *    '@' - Verbose mode        
     *    '#' - Number of decimal places
     *    '?' - Scan command - displays all the labels of attached nodes 
     */ 
    void run();
    /**
     * Function reading the string of user input and firing callbacks that have been added to the commander 
     * once the user has requested them - when he sends the command  
     * 
     *  - It has default commands (the letters can be changed in the commands.h file)
     *    '@' - Verbose mode        
     *    '#' - Number of decimal places
     *    '?' - Scan command - displays all the labels of attached nodes
     * 
     * @param reader - Stream to read user input
     */ 
    void run(Stream &reader);
    /**
     * Function reading the string of user input and firing callbacks that have been added to the commander 
     * once the user has requested them - when he sends the command  
     * 
     *  - It has default commands (the letters can be changed in the commands.h file)
     *    '@' - Verbose mode        
     *    '#' - Number of decimal places
     *    '?' - Scan command - displays all the labels of attached nodes
     * 
     * @param user_input - string of user inputs
     */ 
    void run(char* user_input);

    /**
     *  Function adding a callback to the coomander withe the command id
     * @param id         - char command letter
     * @param onCommand  - function pointer void function(char*) 
     * @param label      - string label to be displayed when scan command sent
     */
    void add(char id , CommandCallback onCommand, char* label = nullptr);

    // printing variables
    VerboseMode verbose = VerboseMode::user_friendly; //!< flag signaling that the commands should output user understanable text
    uint8_t decimal_places = 3; //!< number of decimal places to be used when displaying numbers

    // monitoring functions
    Stream* com_port = nullptr; //!< Serial terminal variable if provided
    
    /**
     * 
     * FOC motor (StepperMotor and BLDCMotor) command interface
     *  - It has several paramters (the letters can be changed in the commands.h file)
     *    'Q' - Q current PID controller & LPF (see function pid and lpf for commands)
     *    'D' - D current PID controller & LPF (see function pid and lpf for commands)  
     *    'V' - Velocity PID controller & LPF  (see function pid and lpf for commands)  
     *    'A' - Angle PID controller & LPF     (see function pid and lpf for commands) 
     *    'L' - Limits                         
     *           sub-commands:
     *           'C' - Current  
     *           'U' - Voltage   
     *           'V' - Velocity  
     *    'C' - Motion control type config   
     *          sub-commands:
     *          'D' - downsample motiron loop 
     *          '0' - torque    
     *          '1' - velocity 
     *          '2' - angle    
     *    'T' - Torque control type        
     *          sub-commands:
     *          '0' - voltage      
     *          '1' - current     
     *          '2' - foc_current 
     *    'E' - Motor status (enable/disable)  
     *          sub-commands:
     *          '0' - enable    
     *          '1' - disable  
     *    'R' - Motor resistance               
     *    'S' - Sensor offsets                 
     *          sub-commands:
     *          'M' - sensor offset          
     *          'E' - sensor electrical zero 
     *    'M' - Monitoring control             
     *          sub-commands:
     *          'D' - downsample monitoring     
     *          'C' - clear monitor             
     *          'S' - set monitoring variables  
     *          'G' - get variable value        
     *    '' - Target get/set                  
     *  
     *  - Each of them can be get by sening the command letter -(ex. 'R' - to get the phase resistance)
     *  - Each of them can be set by sending 'IdSubidValue' - (ex. SM1.5 for setting sensor zero offset to 1.5)
     *
     */
    void motor(FOCMotor* motor, char* user_cmd);

    /**
     * Low pass fileter command interface
     *  - It only has one property - filtering time constant Tf
     *  - It can be get by sending 'F'
     *  - It can be set by sending 'Fvalue' - (ex. F0.01 for settin Tf=0.01)
     */
    void lpf(LowPassFilter* lpf, char* user_cmd);
    /**
     * PID controller command interface
     *  - It has several paramters (the letters can be changed in the commands.h file)
     *     - P gain       - 'P'
     *     - I gain       - 'I'
     *     - D gain       - 'D'
     *     - output ramp  - 'R'
     *     - output limit - 'L'
     *  - Each of them can be get by sening the command letter -(ex. 'D' - to get the D gain)
     *  - Each of them can be set by sending 'IDvalue' - (ex. I1.5 for setting I=1.5)
     */
    void pid(PIDController* pid, char* user_cmd);
    /**
     * Float variable scalar command interface
     *  - It only has one property - one float value
     *  - It can be get by sending an empty string '\n'
     *  - It can be set by sending 'value' - (ex. 0.01 for settin *value=0.01)
     */
    void scalar(float* value, char* user_cmd);

  private:
    // Subscribed command callback variables
    CommandCallback call_list[20];//!< array of command callback pointers - 20 is an arbitrary number
    char call_ids[20]; //!< added callback commands
    char* call_label[20]; //!< added callback labels
    int call_count = 0;//!< number callbacks that are subscribed

    // helping variable for serial communication reading
    char received_chars[20] = {0}; //!< so far received user message - waiting for newline
    int rec_cnt = 0; //!< number of characters receives
        
    // serial printing functions
    /**
     *  print the string message only if verbose mode on
     *  @param message - message to be printed
     */
    void printVerbose(const char* message); 
    /**
     *  Print the string message only if verbose mode on 
     *  - Function handling the case for strings defined by F macro
     *  @param message - message to be printed
     */
    void printVerbose(const __FlashStringHelper *message);  
    /**
     *  print the numbers to the serial with desired decimal point number
     *  @param message - number to be printed
     *  @param newline - if needs lewline (1) otherwise (0)
     */
    void print(const float number);
    void print(const int number);
    void print(const char* message);
    void print(const __FlashStringHelper *message);
    void print(const char message);
    void println(const float number);
    void println(const int number);
    void println(const char* message);
    void println(const __FlashStringHelper *message);
    void println(const char message);

    void printError();
};


#endif
