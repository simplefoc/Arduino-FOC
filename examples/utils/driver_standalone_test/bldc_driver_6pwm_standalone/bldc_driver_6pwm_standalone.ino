// BLDC driver standalone example
#include <SimpleFOC.h>

// BLDC driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(5, 6, 9,10, 3, 11, 8);

void setup() {

  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 50000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // daad_zone [0,1] - default 0.02f - 2%
  driver.dead_zone = 0.05f;

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