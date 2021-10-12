// Stepper driver standalone example
#include <SimpleFOC.h>


// Stepper driver instance
// StepperDriver2PWM(pwm1, in1, pwm2, in2, (en1, en2 optional))
int in1[] = {4,5};
int in2[] = {9,8};
StepperDriver2PWM driver = StepperDriver2PWM(3, in1, 10 , in2, 11, 12);

// StepperDriver2PWM(pwm1, dir1, pwm2, dir2,(en1, en2 optional))
// StepperDriver2PWM driver = StepperDriver2PWM(3, 4, 5, 6, 11, 12);

void setup() {
  
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 30000;
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
    driver.setPwm(3,6);
}