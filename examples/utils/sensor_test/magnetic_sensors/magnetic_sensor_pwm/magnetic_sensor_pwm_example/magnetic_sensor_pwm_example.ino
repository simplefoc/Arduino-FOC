#include <SimpleFOC.h>


/**
 * Magnetic sensor reading pwm signal on pin 2.  The pwm duty cycle is proportional to the sensor angle.
 * 
 * MagneticSensorPWM(uint8_t MagneticSensorPWM, int _min, int _max)
 * - pinPWM         - the pin that is reading the pwm from magnetic sensor
 * - min_raw_count  - the smallest expected reading.  Whilst you might expect it to be 0 it is often ~5.  Getting this wrong results in a small click once per revolution
 * - max_raw_count  - the largest value read.  whilst you might expect it to be 1kHz = 1000 it is often ~910. depending on the exact frequency and saturation
 */
MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
void doPWM(){sensor.handlePWM();}

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
