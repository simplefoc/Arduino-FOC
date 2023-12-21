// 6pwm standalone example code for Teensy 3.x boards
#include <SimpleFOC.h>


// BLDC driver instance
// using FTM0 timer
BLDCDriver6PWM driver = BLDCDriver6PWM(22,23, 9,10, 6,20, 8);
// using FTM3 timer - available on Teensy3.5 and Teensy3.6
// BLDCDriver6PWM driver = BLDCDriver6PWM(2,14, 7,8, 35,36, 8);

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