/**
 *  Generic sensor example code 
 * 
 * This is a code intended to demonstrate how to implement the generic sensor class 
 * 
 */

#include <SimpleFOC.h>

// sensor reading function example
//  for the magnetic sensor with analog communication
// returning an angle in radians in between 0 and 2PI
float readSensor(){
  return analogRead(A0)*_2PI/1024.0;
}

// sensor intialising function
void initSensor(){
  pinMode(A0,INPUT);
}

// generic sensor class contructor
// - read sensor callback 
// - init sensor callback (optional)
GenericSensor sensor = GenericSensor(readSensor, initSensor);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // if callbacks are not provided in the constructor 
  // they can be assigned directly: 
  //sensor.readCallback = readSensor;
  //sensor.initCallback = initSensor;

  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  sensor.update();

  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}