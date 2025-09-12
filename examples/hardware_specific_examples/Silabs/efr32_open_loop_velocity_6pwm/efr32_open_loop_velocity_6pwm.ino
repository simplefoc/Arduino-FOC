/** 
 * Silabs MG24 6PWM open loop velocity control example
 *
 * HARDWARE CONFIGURATION:
 *  CPU Board: Arduino Nano Matter
 *  Motor Driver Board: BOOSTXL-DRV8305EVM
 *  BLDC Motor: Newark DF45M024053-A2
 */

#include <SimpleFOC.h>

static bool allow_run = false;

// BLDC motor instance
BLDCMotor *motor;

// BLDC driver instance
BLDCDriver6PWM *driver;

// Commander instance
Commander *command;

void doMotor(char* cmd) {
  if (!command) return;
  command->motor(motor, cmd);
}

void setup() {
  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // Commander
  command = new Commander(Serial);
  if (!command) return;

  // Driver
  driver = new BLDCDriver6PWM(6, 7, 8, 9, 10, 11, 12);
  if (!driver) return;

  // Driver config
  // power supply voltage [V]
  driver->voltage_power_supply = 24;
  // pwm frequency to be used [Hz]
  driver->pwm_frequency = 20000; // 20 kHz
  // Max DC voltage allowed - default voltage_power_supply
  driver->voltage_limit = 12;
  // dead zone percentage of the duty cycle - default 0.02 - 2%
  // Can set value to 0 because the DRV8305 will provide the 
  // required dead-time.
  driver->dead_zone = 0;

  // init driver
  if (!driver->init()) {
    Serial.println("Driver init failed!");
    return;
  }
  driver->enable();

  // Motor
  motor = new BLDCMotor(8);
  if (!motor) return;

  // link the motor and the driver
  motor->linkDriver(driver);

  // default voltage_power_supply
  motor->voltage_limit = 0.8f;

  // set motion control loop to be used
  motor->controller = MotionControlType::velocity_openloop;

  // choose FOC modulation (optional) - SinePWM or SpaceVectorPWM
  motor->foc_modulation = FOCModulationType::SpaceVectorPWM;

  // initialize motor
  if (!motor->init()) {
    Serial.println("Motor init failed!");
    return;
  }

  // add target command M
  command->add('M', doMotor, "motor");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  allow_run = true;
  _delay(1000);
}


void loop() {
  if (!allow_run) return;

  // open loop velocity movement
  // using motor.voltage_limit
  motor->move();

  // user communication
  command->run();
}
