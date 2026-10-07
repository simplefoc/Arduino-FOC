// SimpleFOC open-loop velocity — Arduino UNO Q (Zephyr core)
// Phases A/B/C on pins 8, 3, 6 -> all on TIM3 (TIM3_CH1 / CH3 / CH4)
// so the three PWM channels share one timer/time-base.
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);                       // pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(8, 3, 6, 7);    // enable on D7

// target variable
float target_velocity = 0;

// commander interface
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd)  { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);   // verbose output for debugging

  // driver config
  driver.voltage_power_supply = 12;  // [V]
  driver.voltage_limit = 5;          // [V] hard cap the driver can output
  driver.pwm_frequency = 25000;      // [Hz] applied at runtime via pwm_set_pulse_dt()
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);
  
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
 
  // motor limits
  motor.voltage_limit = 3;           // [V] start low for low-resistance motors

  // open-loop velocity control
  motor.controller = MotionControlType::velocity_openloop;

  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  // commands: T = target velocity [rad/s], L = voltage limit [V]
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit,  "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s] with e.g. T5");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  command.run();
}