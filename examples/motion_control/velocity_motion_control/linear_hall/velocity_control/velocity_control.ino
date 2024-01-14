/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 */
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// hall sensor instance
LinearHall sensor = LinearHall(A0, A1, 11);

// velocity set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // initialize sensor hardware. This moves the motor to find the min/max sensor readings and 
  // averages them to get the center values. The motor can't move until motor.init is called, and 
  // motor.initFOC can't do its calibration until the sensor is intialized, so this must be done inbetween.
  // You can then take the values printed to the serial monitor and pass them to sensor.init to 
  // avoid having to move the motor every time. In that case it doesn't matter whether sensor.init 
  // is called before or after motor.init.
  sensor.init(&motor);
  Serial.print("LinearHall centerA: ");
  Serial.print(sensor.centerA);
  Serial.print(", centerB: ");
  Serial.println(sensor.centerB);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}


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
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}