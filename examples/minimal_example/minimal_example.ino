#include <SimpleFOC.h>

// This example gives you a minimal code needed to run the FOC algorithm
// All the configuration is set to defualt values
// motor.power_supply_voltage= 12V
// encoder.quadrature = Quadrature::ENABLE
// encoder.pullup = Pullup::EXTERN
// motor.PI_velocity.K = 1
// motor.PI_velocity.Ti = 0.003

//  BLDCMotor( phA, phB, phC, pole_pairs,  enable)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(encA, encB , ppr, index)
Encoder encoder = Encoder(2, 3, 8192, 4);

// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
// please set the right PCINT(0,1,2)_vect parameter
ISR (PCINT2_vect) { encoder.handleIndex(); }

void setup() {
  // debugging port
  Serial.begin(115200);

  // initialise encoder hardware
  encoder.init(doA, doB);

  // velocity control
  motor.controller = ControlType::velocity;

  // contoller configuration based on the controll type 
  motor.PI_velocity.K = 0.3;
  motor.PI_velocity.Ti = 0.003;
  motor.PI_velocity.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  motor.PI_velocity.voltage_ramp = 300;


  // link the motor to the sensor
  motor.linkEncoder(&encoder);
  // intialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  _delay(1000);
}

// target velocity variable
float target_velocity = 2;

void loop() {
  // foc loop
  motor.loopFOC();
  // control loop
  motor.move(target_velocity);
}
