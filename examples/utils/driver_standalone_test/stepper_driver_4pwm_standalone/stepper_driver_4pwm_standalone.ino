// Stepper driver standalone example
#include <SimpleFOC.h>


// Stepper driver instance
// StepperDriver4PWM(ph1A, ph1B, ph2A, ph2B, (en1, en2 optional))
StepperDriver4PWM driver = StepperDriver4PWM(5, 6, 9,10, 7, 8);

void setup() {
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 30000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  
  // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // enable driver
  driver.enable();
  Serial.println("Driver ready!");
  _delay(1000);
}

void loop() {
    // setting pwm
    // phase A: 3V
    // phase B: 6V
    driver.setPwm(3,6);
}