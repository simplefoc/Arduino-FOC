#include <SimpleFOC.h>

// Motor instance
// its not important how many pole pairs do you set, the progam will find it alone
BLDCMotor motor = BLDCMotor(9, 5, 6, 0, 8);

// magnetic sensor instance
MagneticSensor AS5x4x = MagneticSensor(10, 16384, 0x3FFF);

void setup() {

  // initialise magnetic sensor hardware
  AS5x4x.init();
  // link the motor to the sensor
  motor.linkSensor(&AS5x4x);

  // power supply voltage
  motor.voltage_power_supply = 12;
  // set motion control loop to be used
  motor.controller = ControlType::voltage;

  // initialize motor hardware
  motor.init();

  // debugging port
  Serial.begin(115200);

  // pole pairs calculation routine
  Serial.println("Motor pole pair number estimation example");
  Serial.println("---------------------------------------------\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*M_PI; // search electrical angle to turn
  
  // move motor to the electrical angle 0
  motor.setPhaseVoltage(pp_search_voltage, 0);
  _delay(1000);
  // read the sensor angle 
  float angle_begin = AS5x4x.getAngle();
  _delay(50);
  
  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01;
    motor.setPhaseVoltage(pp_search_voltage, motor_angle);
  }
  _delay(1000);
  // read the sensor value for 180
  float angle_end = AS5x4x.getAngle();
  _delay(50);
  // turn off the motor
  motor.setPhaseVoltage(0,0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print("Estimated pole pairs number is: ");
  Serial.println(pp);
  Serial.println("Electrical angle / Encoder angle = Pole pairs ");
  Serial.print(pp_search_angle*180/M_PI);
  Serial.print("/");
  Serial.print((angle_end-angle_begin)*180/M_PI);
  Serial.print(" = ");
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();
   

  // a bit of debugging the result
  if(pp <= 0 ){
    Serial.println("Pole pair number cannot be negative");
    Serial.println(" - Try changing the search_voltage value or motor/sensor configuration.");
    return;
  }else if(pp > 30){
    Serial.println("Pole pair number very high, possible error.");
  }else{
    Serial.println("If pp is estimated well your motor should turn now!");
    Serial.println(" - If it is not moving try to relaunch the program!");
    Serial.println(" - You can also try to adjust the target voltage using serial terminal!");
  }

  
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align sensor and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println("\n Motor ready.");
  Serial.println("Set the target voltage using serial terminal:");
}

// uq voltage
float target_voltage = 2;

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
  motor.move(target_voltage);
  
  // communicate with the user
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
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}