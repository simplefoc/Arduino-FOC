#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/pid.h"
#include "../common/lowpass_filter.h"
#include "commands.h"


#define MAX_COMMAND_LENGTH 20


// Commander verbose display to the user type
enum VerboseMode : uint8_t {
  nothing       = 0x00, // display nothing - good for monitoring
  on_request    = 0x01, // display only on user request
  user_friendly = 0x02,  // display textual messages to the user
  machine_readable = 0x03 // display machine readable commands, matching commands to set each settings
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
     * @param eol - the end of line sentinel character
     * @param echo - echo last typed character (for command line feedback)
     */
    Commander(Stream &serial, char eol = '\n', bool echo = false);
    Commander(char eol = '\n', bool echo = false);

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
     * @param reader - temporary stream to read user input
     * @param eol - temporary end of line sentinel
     */
    void run(Stream &reader, char eol = '\n');
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
    void add(char id , CommandCallback onCommand, const char* label = nullptr);

    // printing variables
    VerboseMode verbose = VerboseMode::user_friendly; //!< flag signaling that the commands should output user understanable text
    uint8_t decimal_places = 3; //!< number of decimal places to be used when displaying numbers

    // monitoring functions
    Stream* com_port = nullptr; //!< Serial terminal variable if provided
    char eol = '\n'; //!< end of line sentinel character
    bool echo = false; //!< echo last typed character (for command line feedback)

    /**
     *
     * FOC motor (StepperMotor and BLDCMotor) command interface
     * @param motor    - FOCMotor (BLDCMotor or StepperMotor) instance 
     * @param user_cmd - the string command
     * 
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
     *    '' - Target setting interface 
     *         Depends of the motion control mode:
     *          - torque                          : torque (ex. M2.5) 
     *          - velocity (open and closed loop) : velocity torque (ex.M10 2.5 or M10 to only chanage the target witout limits)
     *          - angle    (open and closed loop) : angle velocity torque (ex.M3.5 10 2.5 or M3.5 to only chanage the target witout limits)
     *
     *  - Each of them can be get by sening the command letter -(ex. 'R' - to get the phase resistance)
     *  - Each of them can be set by sending 'IdSubidValue' - (ex. SM1.5 for setting sensor zero offset to 1.5f)
     *
     */
    void motor(FOCMotor* motor, char* user_cmd);

    /**
     * Low pass fileter command interface
     * @param lpf      - LowPassFilter instance 
     * @param user_cmd - the string command
     * 
     *  - It only has one property - filtering time constant Tf
     *  - It can be get by sending 'F'
     *  - It can be set by sending 'Fvalue' - (ex. F0.01 for settin Tf=0.01)
     */
    void lpf(LowPassFilter* lpf, char* user_cmd);
    /**
     * PID controller command interface
     * @param pid      - PIDController instance 
     * @param user_cmd - the string command
     * 
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
     * @param value    - float variable pointer 
     * @param user_cmd - the string command
     * 
     *  - It only has one property - one float value
     *  - It can be get by sending an empty string '\n'
     *  - It can be set by sending 'value' - (ex. 0.01f for settin *value=0.01)
     */
    void scalar(float* value, char* user_cmd);
    /**
     *  Target setting interface, enables setting the target and limiting variables at once. 
     *  The values are sent separated by a separator specified as the third argument. The default separator is the space.
     * 
     * @param motor     - FOCMotor (BLDCMotor or StepperMotor) instance 
     * @param user_cmd  - the string command
     * @param separator - the string separator in between target and limit values, default is space - " "
     *  
     *  Example: P2.34 70 2
     *  `P` is the user defined command, `2.34` is the target angle `70` is the target 
     *  velocity and `2` is the desired max current.
     *  
     *  It depends of the motion control mode:
     *    - torque   : torque (ex. P2.5) 
     *    - velocity : velocity torque (ex.P10 2.5 or P10 to only chanage the target witout limits)
     *    - angle    : angle velocity torque (ex.P3.5 10 2.5 or P3.5 to only chanage the target witout limits)
     */
    void target(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");

    /**
     * FOC motor (StepperMotor and BLDCMotor) motion control interfaces
     * @param motor     - FOCMotor (BLDCMotor or StepperMotor) instance 
     * @param user_cmd  - the string command
     * @param separator - the string separator in between target and limit values, default is space - " "
     * 
     * Commands:
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
     *    '' - Target setting interface 
     *         Depends of the motion control mode:
     *          - torque                          : torque (ex. M2.5) 
     *          - velocity (open and closed loop) : velocity torque (ex.M10 2.5 or M10 to only chanage the target witout limits)
     *          - angle    (open and closed loop) : angle velocity torque (ex.M3.5 10 2.5 or M3.5 to only chanage the target witout limits)
     */
    void motion(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");

    bool isSentinel(char ch);
  private:
    // Subscribed command callback variables
    CommandCallback call_list[20];//!< array of command callback pointers - 20 is an arbitrary number
    char call_ids[20]; //!< added callback commands
    char* call_label[20]; //!< added callback labels
    int call_count = 0;//!< number callbacks that are subscribed

    // helping variable for serial communication reading
    char received_chars[MAX_COMMAND_LENGTH] = {0}; //!< so far received user message - waiting for newline
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

    void printMachineReadable(const float number);
    void printMachineReadable(const int number);
    void printMachineReadable(const char* message);
    void printMachineReadable(const __FlashStringHelper *message);
    void printMachineReadable(const char message);

    void printlnMachineReadable(const float number);
    void printlnMachineReadable(const int number);
    void printlnMachineReadable(const char* message);
    void printlnMachineReadable(const __FlashStringHelper *message);
    void printlnMachineReadable(const char message);


    void printError();
};


#endif
