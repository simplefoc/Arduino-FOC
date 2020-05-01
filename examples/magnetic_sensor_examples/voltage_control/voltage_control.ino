#include <SimpleFOC.h>

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 5, 6, 11, 8);

// MagneticSensor(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  _cpr            - counts per revolution 
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensor AS5x4x = MagneticSensor(10, 16384, 0x3FFF);

void setup() {
  // debugging port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  AS5x4x.init();
  
  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set FOC loop to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::angle
  motor.controller = ControlType::voltage;

  // use debugging with serial for motor init
  // comment out if not needed
  motor.useDebugging(Serial);

  // link the motor to the sensor
  motor.linkSensor(&AS5x4x);
  // intialise motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();


  Serial.println("Motor ready.");
  Serial.println("Set the target voltage using serial terminal:");
  _delay(1000);
}

// uq voltage
float target_voltage = 2;

void loop() {

  // iterative state calculation calculating angle
  // and setting FOC pahse voltage
  // the faster you run this funciton the better
  // in arduino loop it should have ~1kHz
  // the best would be to be in ~10kHz range
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this funciton can be run at much lower frequency than loopFOC funciton
  // it can go as low as ~50Hz
  motor.move(target_voltage);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();
}


// Serial communication callback function
// gets the target value from the user
void serialEvent() {
  // a string to hold incoming data
  static String inputString; 
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      target_voltage = inputString.toFloat();
      Serial.print("Tagret voltage: ");
      Serial.println(target_voltage);
      inputString = "";
    }
  }
}