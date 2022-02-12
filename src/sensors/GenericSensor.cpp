#include "GenericSensor.h"


/*
  GenericSensor( float (*readCallback)() )
  - readCallback - pointer to the function which reads the sensor angle.
*/

GenericSensor::GenericSensor(float (*readCallback)(), void (*initCallback)()){
  // if function provided add it to the 
  if(readCallback != nullptr) this->readCallback = readCallback;
  if(initCallback != nullptr) this->initCallback = initCallback;
}

void GenericSensor::init(){
  // if init callback specified run it
  if(initCallback != nullptr) this->initCallback();
  this->Sensor::init(); // call base class init
}

/*
	Shaft angle calculation
*/
float GenericSensor::getSensorAngle(){
  return this->readCallback();
}