// 6pwm standalone example code for Teensy 4.x boards
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


// BLDC driver instance
// make sure to provide channel A for high side and channel B for low side
// BLDCDriver6PWM(pwmA_H, pwmA_L, pwmB_H,pwmB_L, pwmC_H, pwmC_L)
// Example configuration 
BLDCDriver6PWM driver = BLDCDriver6PWM(2,3, 6,9, 8,7);

void setup() {
  Serial.begin(115200);
  // Enable debugging
  // Driver init will show debugging output
  SimpleFOCDebug::enable(&Serial);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 30000;
  // dead zone percentage of the duty cycle - default 0.02 - 2%
  driver.dead_zone=0.02;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  
  // driver init
  driver.init();
  
  // enable driver
  driver.enable();
  
  _delay(1000);
}

void loop() {
  // setting pwm
  // phase A: 3V
  // phase B: 6V
  // phase C: 5V
  driver.setPwm(3,6,5);
}