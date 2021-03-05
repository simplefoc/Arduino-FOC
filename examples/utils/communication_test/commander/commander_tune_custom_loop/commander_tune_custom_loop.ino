/**
 * A simple example to show how to use the commander with the control loops outside of the scope of the SimpleFOC library
*/
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 6, 8);

// encoder instance
Encoder encoder = Encoder(2, 3, 500);
// channel A and B callbacks
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// target voltage to be set to the motor
float target_velocity = 0;

// PID controllers and low pass filters
PIDController PIDv{0.05, 1, 0, 100000000, 12};
LowPassFilter LPFv{0.01};

//add communication
Commander command = Commander(Serial);
void doController(char* cmd) { command.pid(&PIDv, cmd); }
void doFilter(char* cmd) { command.lpf(&LPFv, cmd); }
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }


void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set motion control loop to be used ( doing nothing )
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // use monitoring with serial
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // subscribe the new commands
  command.add('C', doController, "tune velocity pid");
  command.add('F', doFilter, "tune velocity LPF");
  command.add('T', doTarget, "motor target");

  _delay(1000);
  Serial.println(F("Commander listening"));
  Serial.println(F(" - Send ? to see the node list..."));
}



void loop() {
  // looping foc
  motor.loopFOC();

  // calculate voltage
  float target_voltage = PIDv(target_velocity - LPFv(motor.shaft_velocity));
  // set the voltage
  motor.move(target_voltage);

  // user communication
  command.run();
}