#include <SimpleFOC.h>

/**
 * An example to find out the raw max and min count to be provided to the constructor
 * Spin your motor/sensor/magnet to see what is the maximum output of the sensor and what is the minimum value 
 * And replace values 14 and 1020 with new values. Once when you replace them make sure there is no jump in the angle reading sensor.getAngle(). 
 * If there is a jump that means you can still find better values. 
 */

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

int max_count = 0;
int min_count = 100000; 

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();

  // keep track of min and max
  if(sensor.raw_count > max_count) max_count = sensor.raw_count;
  else if(sensor.raw_count < min_count) min_count = sensor.raw_count;

  // display the raw count, and max and min raw count
  Serial.print("angle:");
  Serial.print(sensor.getAngle());
  Serial.print("\t, raw:");
  Serial.print(sensor.raw_count);
  Serial.print("\t, min:");
  Serial.print(min_count);
  Serial.print("\t, max:");
  Serial.println(max_count);
  delay(100);
}