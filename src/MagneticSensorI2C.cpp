#include "MagneticSensorI2C.h"

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  @param _chip_address  I2C chip address
//  @param _bit_resolution  bit resolution of the sensor  
//  @param _angle_register_msb  angle read register
//  @param _bits_used_msb number of used bits in msb
MagneticSensorI2C::MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _bits_used_msb){
  // chip I2C address
  chip_address = _chip_address; 
  // angle read register of the magnetic sensor
  angle_register_msb = _angle_register_msb;
  // register maximum value (counts per revolution)
  cpr = pow(2,_bit_resolution);
  
  // depending on the sensor architecture there are different combinations of
  // LSB and MSB register used bits
  // AS5600 uses 0..7 LSB and 8..11 MSB
  // AS5048 uses 0..5 LSB and 6..13 MSB 
  // used bits in LSB
  lsb_used = _bit_resolution - _bits_used_msb;
  // extraction masks
  lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
  msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
}


void MagneticSensorI2C::init(){
  
	//I2C communication begin
	Wire.begin();

	// velocity calculation init
	angle_prev = 0;
	velocity_calc_timestamp = _micros(); 

	// full rotations tracking number
	full_rotation_offset = 0;
	angle_data_prev = getRawCount();  
	zero_offset = 0;
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorI2C::getAngle(){
  // raw data from the sensor
  float angle_data = getRawCount(); 

  // tracking the number of rotations 
  // in order to expand angle range form [0,2PI] 
  // to basically infinity
  float d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(abs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;

  // zero offset adding
  angle_data -= (int)zero_offset;
  // return the full angle 
  // (number of full rotations)*2PI + current sensor angle
  return full_rotation_offset + ( angle_data / (float)cpr) * _2PI;
}

// Shaft velocity calculation
float MagneticSensorI2C::getVelocity(){
  // calculate sample time
  float Ts = (_micros() - velocity_calc_timestamp)*1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // current angle
  float angle_c = getAngle();
  // velocity calculation
  float vel = (angle_c - angle_prev)/Ts;
  
  // save variables for future pass
  angle_prev = angle_c;
  velocity_calc_timestamp = _micros();
  return vel;
}

// set current angle as zero angle 
// return the angle [rad] difference
float MagneticSensorI2C::initRelativeZero(){
  float angle_offset = -getAngle();
  zero_offset = getRawCount();

  // angle tracking variables
  full_rotation_offset = 0;
  return angle_offset;
}
// set absolute zero angle as zero angle
// return the angle [rad] difference
float MagneticSensorI2C::initAbsoluteZero(){
  float rotation = -(int)zero_offset;
  // init absolute zero
  zero_offset = 0;

  // angle tracking variables
  full_rotation_offset = 0;
  // return offset in radians
  return rotation / (float)cpr * _2PI;
}
// returns 0 if it has no absolute 0 measurement
// 0 - incremental encoder without index
// 1 - encoder with index & magnetic sensors
int MagneticSensorI2C::hasAbsoluteZero(){
  return 1;
}
// returns 0 if it does need search for absolute zero
// 0 - magnetic sensor 
// 1 - ecoder with index
int MagneticSensorI2C::needsAbsoluteZeroSearch(){
  return 0;
}


// function reading the raw counter of the magnetic sensor
int MagneticSensorI2C::getRawCount(){
	return (int)MagneticSensorI2C::read(angle_register_msb);
}

// I2C functions 
/*
* Read a register from the sensor
* Takes the address of the register as a uint8_t
* Returns the value of the register
*/
int MagneticSensorI2C::read(uint8_t angle_reg_msb) {
  // read the angle register first MSB then LSB
	byte readArray[2];
	uint16_t readValue = 0;
  // notify the device that is aboout to be read
	Wire.beginTransmission(chip_address);
	Wire.write(angle_reg_msb);
  Wire.endTransmission(false);
  
  // read the data msb and lsb
	Wire.requestFrom(chip_address, (uint8_t)2);
	for (byte i=0; i < 2; i++) {
		readArray[i] = Wire.read();
	}

  // depending on the sensor architecture there are different combinations of 
  // LSB and MSB register used bits
  // AS5600 uses 0..7 LSB and 8..11 MSB
  // AS5048 uses 0..5 LSB and 6..13 MSB
  readValue = ( readArray[1] &  lsb_mask );
	readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
	return readValue;
}
