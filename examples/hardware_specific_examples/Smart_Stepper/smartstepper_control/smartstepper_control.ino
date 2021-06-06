/**
 * Smart Stepper support with SimpleFOClibrary
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, A2);

// Stepper motor & driver instance
StepperMotor motor = StepperMotor(50);
int in1[2] = {5, 6};
int in2[2] = {A4, 7};
StepperDriver2PWM driver = StepperDriver2PWM(4, in1, 9, in2);

// instantiate the commander
Commander command = Commander(SerialUSB);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with SerialUSB 
  SerialUSB.begin(115200);
  // comment out if not needed
  motor.useMonitoring(SerialUSB);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command M
  command.add('M', doMotor, "my motor");

  SerialUSB.println(F("Motor ready."));
  SerialUSB.println(F("Set the target voltage using Serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();
  
  // user communication
  command.run();
}