// 6pwm openloop velocity example
// 
// Teensy4.x 6pwm driver generates a 6pwm signal using FlexTimers and it doesn't support the QuadTimers
//  - Each high-low pair of the 6pwm has to be A and B channels of the same FlexTimer and to the same submodule 
// 
// List of available teensy 4.1 pins with their respective submodules and channels 
// FlexPWM(timer number)_(submodule)_(channel) 
// FlexPWM4_2_A   pin 2
// FlexPWM4_2_B   pin 3  
// FlexPWM1_3_B   pin 7  
// FlexPWM1_3_A   pin 8  
// FlexPWM2_2_A   pin 6  
// FlexPWM2_2_B   pin 9  
// FlexPWM3_1_B   pin 28  
// FlexPWM3_1_A   pin 29  
// FlexPWM2_3_A   pin 36  
// FlexPWM2_3_B   pin 37  
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// make sure to provide channel A for high side and channel B for low side
// BLDCDriver6PWM(pwmA_H, pwmA_L, pwmB_H,pwmB_L, pwmC_H, pwmC_L)
// Example configuration 
BLDCDriver6PWM driver = BLDCDriver6PWM(2,3, 6,9, 8,7);

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
