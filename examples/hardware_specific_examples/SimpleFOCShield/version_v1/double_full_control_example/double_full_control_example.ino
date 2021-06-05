
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 5, 6, 8);

// BLDC motor & driver instance
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(3, 10, 11, 7);

// encoder instance
Encoder encoder1 = Encoder(2, A3, 500);
// channel A and B callbacks
void doA1(){encoder1.handleA();}
void doB1(){encoder1.handleB();}

// encoder instance
Encoder encoder2 = Encoder(A1, A2, 500);
// channel A and B callbacks
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}

// commander communication instance
Commander command = Commander(Serial);
void doMotor1(char* cmd){ command.motor(&motor1, cmd); }
void doMotor2(char* cmd){ command.motor(&motor2, cmd); }

void setup() {

  // initialize encoder sensor hardware
  encoder1.init();
  encoder1.enableInterrupts(doA1, doB1);
  // initialize encoder sensor hardware
  encoder2.init();
  encoder2.enableInterrupts(doA2, doB2);
  // link the motor to the sensor
  motor1.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);


  // driver config
  // power supply voltage [V]
  driver1.voltage_power_supply = 12;
  driver1.init();
  // link driver
  motor1.linkDriver(&driver1);
  // power supply voltage [V]
  driver2.voltage_power_supply = 12;
  driver2.init();
  // link driver
  motor2.linkDriver(&driver2);

  // set control loop type to be used
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // contoller configuration based on the controll type
  motor1.PID_velocity.P = 0.05f;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;
  // default voltage_power_supply
  motor1.voltage_limit = 12;
  // contoller configuration based on the controll type
  motor2.PID_velocity.P = 0.05f;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;
  // default voltage_power_supply
  motor2.voltage_limit = 12;

  // angle loop velocity limit
  motor1.velocity_limit = 20;
  motor2.velocity_limit = 20;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // initialise motor
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC();

  // initialise motor
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC();

  // set the inital target value
  motor1.target = 2;
  motor2.target = 2;

  // subscribe motor to the commander
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Double motor sketch ready."));

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  // user communication
  command.run();
}