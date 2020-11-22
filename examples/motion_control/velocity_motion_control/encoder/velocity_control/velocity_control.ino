/**
 * 
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and encoder 
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 * 
 * 
 * 
 * NOTE :
 * > Arduino UNO example code for running velocity motion control using an encoder with index significantly
 * > Since Arduino UNO doesn't have enough interrupt pins we have to use software interrupt library PciManager.
 *  
 * > If running this code with Nucleo or Bluepill or any other board which has more than 2 interrupt pins 
 * > you can supply doIndex directly to the encoder.enableInterrupts(doA,doB,doIndex) and avoid using PciManger
 * 
 * > If you don't want to use index pin initialize the encoder class without index pin number:
 * > For example:
 * > - Encoder encoder = Encoder(2, 3, 8192);
 * > and initialize interrupts like this:
 * > - encoder.enableInterrupts(doA,doB)
 * 
 * Check the docs.simplefoc.com for more info about the possible encoder configuration.
 * 
 */
#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// encoder instance
Encoder encoder = Encoder(2, 3, 8192, A0);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}
// If no available hadware interrupt pins use the software interrupt
PciListenerImp listenerIndex(encoder.index_pin, doIndex);


void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  // software interrupts
  PciManager.registerListener(&listenerIndex);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = ControlType::velocity;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set the target velocity using serial terminal:");
  _delay(1000);
}

// velocity set point variable
float target_velocity = 0;

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  
  // user communication
  serialReceiveUserCommand();
}

// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      
      // change the motor target
      target_velocity = received_chars.toFloat();
      Serial.print("Target velocity: ");
      Serial.println(target_velocity);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}

