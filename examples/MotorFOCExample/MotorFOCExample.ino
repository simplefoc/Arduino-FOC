#include "BLDCMotor.h"

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - cpr           - counts per rotation  (cpm=ppm*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 32768, 4);

void setup() {
  // debugging port
  Serial.begin(115200);

  // check if you need internal pullups
  // Pullup::EXTERN - external pullup added
  // Pullup::INTERN - needs internal arduino pullup
  encoder.init(Pullup::EXTERN);
  // interupt intitialisation
  // A callback and B callback
  attachInterrupt(digitalPinToInterrupt(encoder.pinA), []() {
    encoder.handleA();
  }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder.pinB), []() {
    encoder.handleB();
  }, CHANGE);

  // link the motor to the sensor
  motor.linkEncoder(&encoder);

  // intialise motor
  motor.init(DriverType::bipolar);
  motor.enable();
  // align encoder and start FOC
  motor.initFOC();

  // set FOC loop to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::velocity_ultra_slow
  // ControlType::angle
  motor.controller = ControlType::velocity;

  // velocity PI controller parameters
  motor.PI_velocity.K = 1;
  motor.PI_velocity.Ti = 0.003;
  
  // power supply voltage
  motor.U_max = 12;
  // maximal velocity allowed for position control
  motor.velocity_max = 6;

  Serial.println("Motor ready.");
  delay(1000);
}

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
  motor.move(4);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor_monitor();
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void motor_monitor() {
  switch (motor.controller) {
    case ControlType::velocity_ultra_slow:
    case ControlType::velocity:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
      break;
    case ControlType::angle:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_angle_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
      break;
    case ControlType::voltage:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
      break;
  }
}

