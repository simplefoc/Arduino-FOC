#include <SimpleFOC.h>

/** Annoyingly some i2c sensors (e.g. AS5600) have a fixed chip address.  This means only one of these devices can be addressed on a single bus
 * This example shows how a second i2c bus can be used to communicate with a second sensor.  
 */ 

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

#if defined(_STM32_DEF_) // if stm chips
  // example of stm32 defining 2nd bus
  TwoWire Wire1(PB11, PB10);

#elif defined(ESP_H) // if esp32
  // esp32 defines a Wire1 but doesn't define pins! 
  // nothing to do here for esp32! (See below)
#else
  // Wire constructors vary - you'll have to check what works for your chip
  TwoWire Wire1(SDA1, SCL1);
#endif

void setup() {

  Serial.begin(115200);
  _delay(750);

  Wire.setClock(400000);
  Wire1.setClock(400000);

  #if defined(ESP_H) // if esp32
    // Normally SimpeFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
    // It seems safe to call begin multiple times
    Wire1.begin(19,23,400000);
  #endif

  sensor0.init();
  sensor1.init(&Wire1);
}

void loop() {
  _delay(200);
  Serial.print(sensor0.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
}
