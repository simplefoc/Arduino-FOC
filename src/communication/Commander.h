#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/pid.h"
#include "../common/lowpass_filter.h"
#include "commands.h"


typedef void (* CommandCallback)(String);

class Commander
{
  public:
    /**
     * Default constructor - setting all variabels to default values
     */
    Commander(HardwareSerial &serial);

    void run();

    void add(char id , CommandCallback onCommand);

    // monitoring functions
    HardwareSerial* com_port; //!< Serial terminal variable if provided
    
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
     *    - 2: angle
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
    void motor(FOCMotor* motor, String user_cmd);
    void lpf(LowPassFilter* lpf, String user_cmd);
    void pid(PIDController* pid, String user_cmd);
    void scalar(float* value, String user_cmd);

  private:
    CommandCallback call_list[20];
    char call_ids[20]; //!< Dictionary of nodes
    int call_count = 0;
    String received_chars = "";

};


#endif
