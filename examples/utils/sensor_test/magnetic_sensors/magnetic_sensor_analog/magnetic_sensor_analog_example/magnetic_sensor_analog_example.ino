#include <SimpleFOC.h>



/**
 * Magnetic sensor reading analog voltage on pin A1.  This voltage is proportional to rotation position.
 * Tested on AS5600 magnetic sensor running in 'analog mode'.  Note AS5600 works better in 'i2C mode' (less noise) but only supports one sensor per i2c bus. 
 * 
 * MagneticSensorAnalog(uint8_t _pinAnalog, int _min, int _max)
 * - pinAnalog      - the pin that is reading the pwm from magnetic sensor
 * - min_raw_count  - the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * - max_raw_count  - the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note ESP32 will be closer to 4096 with its 12bit ADC
 */
MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();

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