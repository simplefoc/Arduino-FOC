#include <SimpleFOC.h>


/**
 * An example to find out the raw max and min count to be provided to the constructor
 * SPin your motor/sensor/magnet to see what is the maximum output of the sensor and what is the minimum value 
 * And replace values 4 and 904 with new values. Once when you replace them make sure there is no jump in the angle reading sensor.getAngle(). 
 * If there is a jump that means you can still find better values. 
 */
MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
void doPWM(){sensor.handlePWM();}

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.pulse_length_us);
  Serial.print("\t");
  Serial.println(sensor.getAngle());
}
