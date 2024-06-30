/**
 *
 * SimpleFOCMini motor control example
 * 
 * For Arduino UNO or the other boards with the UNO headers
 * the most convenient way to use the board is to stack it to the pins:
 * - 12 - ENABLE
 * - 11 - IN1
 * - 10 - IN2
 * -  9 - IN3
 *
 */
#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8); // mini v1.0
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 12); // mini v1.1

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // if SimpleFOCMini is stacked in arduino headers
  // on pins 12,11,10,9,8 
  // pin 12 is used as ground
  pinMode(12,OUTPUT);
  pinMode(12,LOW);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;

  // default voltage_power_supply
  motor.voltage_limit = 2; // Volts

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command M
  command.add('M', doMotor, "motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
  motor.target = 1; //initial target velocity 1 rad/s
  Serial.println("Target velocity: 1 rad/s");
  Serial.println("Voltage limit 2V");
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
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}