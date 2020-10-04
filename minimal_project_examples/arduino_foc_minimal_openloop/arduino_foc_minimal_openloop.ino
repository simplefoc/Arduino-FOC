#include "BLDCMotor.h"


// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 5
#define INH_B 4
#define INH_C 3
#define INL_A 8
#define INL_B 9
#define INL_C 10
#define EN_GATE 13
#define M_PWM A0
#define M_OC A1
#define OC_ADJ A3


// motor instance
BLDCMotor motor = BLDCMotor(INH_A, INH_B, INH_C, INL_A, INL_B, INL_C, 11, EN_GATE);

void setup() {

  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);


  
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

float target_position = 0; // rad/s


void loop() {
  // open  loop angle movements 
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(2);

  // receive the used commands from serial
  serialReceiveUserCommand();
  Serial.println(micros());
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
      target_position = received_chars.toFloat();
      Serial.print("Target position: ");
      Serial.println(target_position);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}
