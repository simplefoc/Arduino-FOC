#include "MagneticSensorSPI.h"

// MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register)
//  cs              - SPI chip select pin 
//  _bit_resolution   sensor resolution bit number
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI::MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register){
  // chip select pin
  chip_select_pin = cs; 
  // angle read register of the magnetic sensor
  angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTAR;
  // register maximum value (counts per revolution)
  cpr = pow(2,_bit_resolution);

}


void MagneticSensorSPI::init(){
	// 1MHz clock (AMS should be able to accept up to 10MHz)
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(chip_select_pin, OUTPUT);

  
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();
	SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
	SPI.setDataMode(SPI_MODE1) ;
	SPI.setClockDivider(SPI_CLOCK_DIV8);

	digitalWrite(chip_select_pin, HIGH);
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
float MagneticSensorSPI::getAngle(){
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
float MagneticSensorSPI::getVelocity(){
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
float MagneticSensorSPI::initRelativeZero(){
  float angle_offset = -getAngle();
  zero_offset = getRawCount();

  // angle tracking variables
  full_rotation_offset = 0;
  return angle_offset;
}
// set absolute zero angle as zero angle
// return the angle [rad] difference
float MagneticSensorSPI::initAbsoluteZero(){
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
int MagneticSensorSPI::hasAbsoluteZero(){
  return 1;
}
// returns 0 if it does need search for absolute zero
// 0 - magnetic sensor 
// 1 - ecoder with index
int MagneticSensorSPI::needsAbsoluteZeroSearch(){
  return 0;
}


// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::getRawCount(){
	return (int)MagneticSensorSPI::read(angle_register);
}

// SPI functions 
/**
 * Utility function used to calculate even parity of word
 */
byte MagneticSensorSPI::spiCalcEvenParity(word value){
	byte cnt = 0;
	byte i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1) cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
word MagneticSensorSPI::read(word angle_register){
	word command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | angle_register;

	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	byte right_byte = command & 0xFF;
	byte left_byte = ( command >> 8 ) & 0xFF;


#if !defined(_STM32_DEF_) // if not stm chips
  //SPI - begin transaction
  SPI.beginTransaction(settings);
#endif
  //SPI - begin transaction
  //SPI.beginTransaction(settings);

  //Send the command
  digitalWrite(chip_select_pin, LOW);
  digitalWrite(chip_select_pin, LOW);
  SPI.transfer(left_byte);
  SPI.transfer(right_byte);
  digitalWrite(chip_select_pin,HIGH);
  digitalWrite(chip_select_pin,HIGH);
  
  delayMicroseconds(10);
  
  //Now read the response
  digitalWrite(chip_select_pin, LOW);
  digitalWrite(chip_select_pin, LOW);
  left_byte = SPI.transfer(0x00);
  right_byte = SPI.transfer(0x00);
  digitalWrite(chip_select_pin, HIGH);
  digitalWrite(chip_select_pin,HIGH);

#if !defined(_STM32_DEF_) // if not stm chips
  //SPI - end transaction
  SPI.endTransaction();
#endif

	// Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close(){
	SPI.end();
}
