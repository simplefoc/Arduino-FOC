#include "MagneticSensorAnalog.h"

/** MagneticSensorAnalog(uint8_t _pinAnalog, int _min, int _max)
 * @param _pinAnalog  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * @param _max_raw_count  the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note: For ESP32 (with 12bit ADC the value will be nearer 4096)
 */
MagneticSensorAnalog::MagneticSensorAnalog(uint8_t _pinAnalog, int _min_raw_count, int _max_raw_count){

  pinAnalog = _pinAnalog;

  cpr = _max_raw_count - _min_raw_count;
  min_raw_count = _min_raw_count;
  max_raw_count = _max_raw_count;

  if(pullup == Pullup::USE_INTERN){
    pinMode(pinAnalog, INPUT_PULLUP);
  }else{
    pinMode(pinAnalog, INPUT);
  }

}


void MagneticSensorAnalog::init(){
	raw_count = getRawCount();

  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorAnalog::getSensorAngle(){
  // raw data from the sensor
  raw_count = getRawCount();   
  return ( (float) (raw_count) / (float)cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorAnalog::getRawCount(){
	return analogRead(pinAnalog);
}
