/**
 * 
 * STM32 Bluepill position motion control example with magnetic sensor
 * 
 * The same example can be ran with any STM32 board - just make sure that put right pin numbers.
 * 
 */
#include <SimpleFOC.h>

// Magnetic sensor instance 
// MISO PA7
// MOSI PA6
// SCK PA5
MagneticSensor AS5x4x = MagneticSensor(PA4, 16384, 0x3FFF);

// Motor instance
BLDCMotor motor = BLDCMotor(PB6, PB7, PB8, 11, PA5);

void setup() {

  // initialise magnetic sensor hardware
  AS5x4x.init();
  // link the motor to the sensor
  motor.linkSensor(&AS5x4x);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = ControlType::angle;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PI_velocity.P = 0.2;
  motor.PI_velocity.I = 20;
  // maximal voltage to be set to the motor
  motor.PI_velocity.voltage_limit = 6;
  
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.P_angle.velocity_limit = 4;

  // use debugging with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useDebugging(Serial);

  
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();


  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}

// angle set point variable
float target_angle = 0;

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
  motor.move(target_angle);


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
      target_angle = received_chars.toFloat();
      Serial.print("Target angle: ");
      Serial.println(target_angle);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}


