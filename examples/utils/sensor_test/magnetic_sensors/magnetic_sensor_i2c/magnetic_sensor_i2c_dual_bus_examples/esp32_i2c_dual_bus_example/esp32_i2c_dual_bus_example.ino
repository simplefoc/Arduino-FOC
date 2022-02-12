#include <SimpleFOC.h>

/** Annoyingly some i2c sensors (e.g. AS5600) have a fixed chip address.  This means only one of these devices can be addressed on a single bus
 * This example shows how a second i2c bus can be used to communicate with a second sensor.  
 */ 

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);


void setup() {

  Serial.begin(115200);
  _delay(750);

  Wire.setClock(400000);
  Wire1.setClock(400000);

  // Normally SimpleFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
  // It seems safe to call begin multiple times
  Wire1.begin(19, 23, (uint32_t)400000);

  sensor0.init();
  sensor1.init(&Wire1);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor0.update();
  sensor1.update();
  
  _delay(200);
  Serial.print(sensor0.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
}
