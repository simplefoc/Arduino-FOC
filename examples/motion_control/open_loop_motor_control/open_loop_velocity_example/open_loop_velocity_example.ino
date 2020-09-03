// Open loop motor control example 
 #include <SimpleFOC.h>

// motor instance
BLDCMotor motor = BLDCMotor(3, 10, 6, 11, 7);

void setup() {
  
  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // limiting motor movements
  motor.voltage_limit = 3;   // rad/s
  motor.velocity_limit = 20; // rad/s

  // open loop control config
  motor.controller = ControlType::velocity_openloop;

  // init motor hardware
  motor.init();
  

  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);
}

float target_velocity = 0; // rad/s

void loop() {
  // open loop velocity movement 
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_velocity);

  // receive the used commands from serial
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
      Serial.print("Target velocity ");
      Serial.println(target_velocity);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}