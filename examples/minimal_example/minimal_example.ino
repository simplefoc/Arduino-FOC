#include <ArduinoFOC.h>

// This example gives you a minimal code needed to run the FOC algorithm
// All the configuration is set to defualt values
// motor.power_supply_voltage= 12V
// motor.driver = DriverType::bipolar
// encoder.pullup = Pullup::EXTERN
// motor.PI_velocity.K = 1
// motor.PI_velocity.Ti = 0.003

//  BLDCMotor( phA, phB, phC, pole_pairs,  enable)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(encA, encB , cpr, index)
Encoder encoder = Encoder(2, 3, 32768, 4);

// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  // debugging port
  Serial.begin(115200);

  // initialise encoder hardware
  encoder.init(doA, doB);
  // link the motor to the sensor
  motor.linkEncoder(&encoder);
  // intialise motor
  motor.init();
  // velocity control
  motor.controller = ControlType::velocity;
  // align encoder and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  delay(1000);
}

// target velocity variable
float target_velocity = 2;

void loop() {
  // foc loop
  motor.loopFOC();
  // control loop
  motor.move(target_velocity);
  motor_monitor();
}
