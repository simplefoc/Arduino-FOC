/**
 * Comprehensive BLDC motor control example using encoder and the minimal version off the SimpleFOC library
 * - THis code is completely stand alone and you dont need to have SimpleFOClibary installed to run it 
 * 
 * Check the docs.simplefoc.com for more information
 * 
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values 
 * 
 * To check the config value just enter the command letter.
 * For example: - to read velocity PI controller P gain run: P
 *              - to set velocity PI controller P gain  to 1.2 run: P1.2
 * 
 * To change the target value just enter a number in the terminal:
 * For example: - to change the target value to -0.1453 enter: -0.1453
 *              - to get the current target value enter: V3 
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
 */

#include "BLDCMotor.h"
#include "Encoder.h"

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11,8);

//Encoder(int encA, int encB , int cpr, int index)
//    - encA, encB    - encoder A and B pins
//    - ppr           - impulses per rotation  (cpr=ppr*4)
//    - index pin     - (optional input)
Encoder encoder = Encoder(2, 3, 8196);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() { 

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA,doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // choose FOC algorithm to be used:
  // FOCModulationType::SinePWM  (default)
  // FOCModulationType::SpaceVectorPWM
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // power supply voltage [V]
  // default 12V
  motor.voltage_power_supply = 12;
  // motor and sensor aligning voltage [V]
  // default 6V
  motor.voltage_sensor_align = 3;
  
  // set control loop type to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::angle
  motor.controller = ControlType::voltage;

  // controller configuration based on the control type 
  // velocity PI controller parameters
  motor.PI_velocity.P = 0.2;
  motor.PI_velocity.I = 20;
  // default voltage_power_supply/2
  motor.PI_velocity.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PI_velocity.voltage_ramp = 1000;

  // velocity low pass filtering
  // default 10ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.0;

  // angle loop controller
  motor.P_angle.P = 3;
  motor.P_angle.velocity_limit = 10;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // enable monitoring
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 2;

  Serial.println("Full control example: ");
  Serial.println("Voltage control target 2V.");
  
  _delay(1000);

}

// target velocity variable
float target = 0;
// loop stats variables
unsigned long  t = 0;
long timestamp = _micros();

void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();

  // user communication
  motor.command(serialReceiveUserCommand());
}

// utility function enabling serial communication the user
String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}
