/**
 * Comprehensive BLDC motor control example using encoder and the DRV8302 board
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * check the https://docs.simplefoc.com for full list of motor commands
 *
 */
#include <SimpleFOC.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 21
#define INH_B 19
#define INH_C 18

#define EN_GATE 5
#define M_PWM 25
#define M_OC 26
#define OC_ADJ 12
#define OC_GAIN 14

#define IOUTA 34
#define IOUTB 35
#define IOUTC 32

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// DRV8302 board has 0.005Ohm shunt resistors and the gain of 12.22 V/V
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// encoder instance
Encoder encoder = Encoder(22, 23, 500);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
  pinMode(OC_GAIN,OUTPUT);
  digitalWrite(OC_GAIN,LOW);


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 15000;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 0.5;
  
  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;
  
  // velocity loop PID
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5.0;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 20.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motor.PID_current_q.P = 3.0;
  motor.PID_current_q.I = 100.0;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.02;
  // current d loop PID
  motor.PID_current_d.P = 3.0;
  motor.PID_current_d.I = 100.0;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.02;

  // Limits 
  motor.velocity_limit = 100.0; // 100 rad/s velocity limit
  motor.voltage_limit = 12.0;   // 12 Volt limit 
  motor.current_limit = 2.0;    // 2 Amp current limit


  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // monitor the two currents d and q
  motor.monitor_downsample = 1000;

  // initialise motor
  motor.init();

  cs.init();
  cs.driverSync(&driver);
  // driver 8302 has inverted gains on all channels
  cs.gain_a *=-1;
  cs.gain_b *=-1;
  cs.gain_c *=-1;
  motor.linkCurrentSense(&cs);
  
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0;

  // define the motor id
  command.add('M', onMotor, "motor");

  Serial.println(F("Full control example: "));
  Serial.println(F("Run user commands to configure and the motor (find the full command list in docs.simplefoc.com) \n "));
  Serial.println(F("Initial motion control loop is voltage loop."));
  Serial.println(F("Initial target voltage 2V."));

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // monitoring the state variables
  motor.monitor();

  // user communication
  command.run();
}