#include "MagneticSensor.h"


// MagneticSensor(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  _cpr            - counts per revolution 
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensor::MagneticSensor(int cs, float _cpr, int _angle_register){
  // chip select pin
  chip_select_pin = cs; 
  // angle read register of the magnetic sensor
  angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTAR;
  // register maximum value (counts per revolution)
  cpr = _cpr;
}


void MagneticSensor::init(){
  // 1MHz clock (AMS should be able to accept up to 10MHz)
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
	
	//setup pins
	pinMode(chip_select_pin, OUTPUT);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();

  // velocity calculation init
  angle_prev = 0;
  velocity_calc_timestamp = _micros();    
}

//	Shaft angle calculation
// angle is in range [0,2*PI]
float MagneticSensor::getAngle(){
	float rotation;
  	rotation = getRawCount() - (int)zero_offset;
	return rotation / (float)cpr * _2PI;
}

// Shaft velocity calculation
float MagneticSensor::getVelocity(){
  // calculate sample time
  float Ts = (_micros() - velocity_calc_timestamp)*1e-6;
  if(Ts > 0.5) Ts = 0.01; // debounce
  // current angle
  float angle_c = getAngle();

  // overflow compensation
  float d_angle = angle_c - angle_prev;
  // if angle changed more than 3PI/2 = 270 degrees
  // consider it as overflow
  if( abs(d_angle) > _3PI_2 ) d_angle += d_angle < 0 ? _2PI :  -_2PI;

  // velocity calculation
  float vel = d_angle/Ts;
  
  // save variables for future pass
  angle_prev = angle_c;
  velocity_calc_timestamp = _micros();
  return vel;
}

// set current agle as zero angle 
// return the angle [rad] difference
float MagneticSensor::initRelativeZero(){
  float angle_offset = -getAngle();
  zero_offset = getRawCount();
  return angle_offset;
}
// set absoule zero angle as zero angle
// return the angle [rad] difference
float MagneticSensor::initAbsoluteZero(){
  float rotation;
  rotation = -(int)zero_offset;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); // more than -180
  // init absolute zero
  zero_offset = 0;
  // return offset in radians
  return rotation / (float)cpr * _2PI;
}
// returns 0 if it has no absolute 0 measurement
// 0 - incremental encoder without index
// 1 - encoder with index & magnetic sensors
int MagneticSensor::hasAbsoluteZero(){
  return 1;
}
// returns 0 if it does need search for absolute zero
// 0 - magnetic sensor 
// 1 - ecoder with index
int MagneticSensor::needsAbsoluteZeroSearch(){
  return 0;
}


// function reading the raw counter of the magnetic sensor
int MagneticSensor::getRawCount(){
	return (int)MagneticSensor::read(angle_register);
}

// SPI finctions 
/**
 * Utility function used to calculate even parity of word
 */
byte MagneticSensor::spiCalcEvenParity(word value){
	byte cnt = 0;
	byte i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
word MagneticSensor::read(word angle_register){
	word command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | angle_register;

	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	byte right_byte = command & 0xFF;
	byte left_byte = ( command >> 8 ) & 0xFF;

	//SPI - begin transaction
	SPI.beginTransaction(settings);

	//Send the command
	digitalWrite(chip_select_pin, LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(chip_select_pin,HIGH);

	//Now read the response
	digitalWrite(chip_select_pin, LOW);
	left_byte = SPI.transfer(0x00);
	right_byte = SPI.transfer(0x00);
	digitalWrite(chip_select_pin, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	//Check if the error bit is set
	if (left_byte & 0x40) {
		errorFlag = true;
	}
	else {
		errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensor::close(){
	SPI.end();
}
