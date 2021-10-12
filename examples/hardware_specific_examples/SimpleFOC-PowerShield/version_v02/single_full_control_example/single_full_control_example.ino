
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8, 4, 7);

// encoder instance
Encoder encoder = Encoder(10, 11, 500);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// inline current sensor instance
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, A0, A1);

// commander communication instance
Commander command = Commander(Serial);
// void doMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd){ command.scalar(&motor.target, cmd); }

void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 20;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  // set control loop type to be used
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // contoller configuration based on the controll type
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 20;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0;

  // subscribe motor to the commander
  // command.add('M', doMotor, "motor");
  command.add('T', doTarget, "target");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor ready.");

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // motor monitoring
  motor.monitor();

  // user communication
  command.run();
}