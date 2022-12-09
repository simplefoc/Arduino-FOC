// 6pwm openloop velocity example
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// using FTM0 timer
// BLDCDriver6PWM(pwmA_H, pwmA_L, pwmB_H,pwmB_L, pwmC_H, pwmC_L)
BLDCDriver6PWM driver = BLDCDriver6PWM(22,23, 9,10, 6,20, 8);
// using FTM3 timer - available on Teensy3.5 and Teensy3.6
// BLDCDriver6PWM driver = BLDCDriver6PWM(2,14, 7,8, 35,36, 8);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // currnet = resistance*voltage, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  //initial motor target
  motor.target=0;

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit 
  motor.move();

  // user communication
  command.run();
}
