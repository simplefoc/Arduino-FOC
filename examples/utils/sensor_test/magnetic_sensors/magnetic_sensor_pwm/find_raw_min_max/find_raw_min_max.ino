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
   // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  _delay(1000);
}

int max_pulse= 0;
int min_pulse = 10000;

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();

  // keep track of min and max
  if(sensor.pulse_length_us > max_pulse) max_pulse = sensor.pulse_length_us;
  else if(sensor.pulse_length_us < min_pulse) min_pulse = sensor.pulse_length_us;

  // display the raw count, and max and min raw count
  Serial.print("angle:");
  Serial.print(sensor.getAngle());
  Serial.print("\t, raw:");
  Serial.print(sensor.pulse_length_us);
  Serial.print("\t, min:");
  Serial.print(min_pulse);
  Serial.print("\t, max:");
  Serial.println(max_pulse);
}
