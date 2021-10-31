/** 
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(PHASE_UH, PHASE_UL, PHASE_VH, PHASE_VL, PHASE_WH, PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, OP1_OUT, OP2_OUT, OP3_OUT);


// encoder instance
Encoder encoder = Encoder(HALL2, HALL3, 2048, HALL1);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
  
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 

  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
 
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;


  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

// angle set point variable
float target_angle = 0;

void loop() {
  // main FOC algorithm function

  // Motion control function
  motor.move(target_angle);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  
  // user communication
  command.run();
}