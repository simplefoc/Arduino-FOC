#include "MagneticSensorI2C.h"

/** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5600_I2C = {
  .chip_address = 0x36,
  .bit_resolution = 12,
  .angle_register = 0x0C,
  .msb_mask = 0x0F,
  .msb_shift = 8,
  .lsb_mask = 0xFF,
  .lsb_shift = 0
};

/** Typical configuration for the 12bit AMS AS5048 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5048_I2C = {
  .chip_address = 0x40,  // highly configurable.  if A1 and A2 are held low, this is probable value
  .bit_resolution = 14,
  .angle_register = 0xFE,
  .msb_mask = 0xFF,
  .msb_shift = 6,
  .lsb_mask = 0x3F,
  .lsb_shift = 0
};

/** Typical configuration for the 12bit MT6701 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s MT6701_I2C = {
  .chip_address = 0x06, 
  .bit_resolution = 14,
  .angle_register = 0x03,
  .msb_mask = 0xFF,
  .msb_shift = 6,
  .lsb_mask = 0xFC,
  .lsb_shift = 2
};


// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  @param _chip_address  I2C chip address
//  @param _bit_resolution  bit resolution of the sensor
//  @param _angle_register_msb  angle read register
//  @param _bits_used_msb number of used bits in msb
MagneticSensorI2C::MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _bits_used_msb, bool lsb_right_aligned){
  _conf.chip_address =  _chip_address;
  _conf.bit_resolution = _bit_resolution;
  _conf.angle_register = _angle_register_msb;
  _conf.msb_mask = (uint8_t)( (1 << _bits_used_msb) - 1 );
  
  uint8_t lsb_used = _bit_resolution - _bits_used_msb; // used bits in LSB
  _conf.lsb_mask = (uint8_t)( (1 << (lsb_used)) - 1 );
  if (!lsb_right_aligned)
    _conf.lsb_shift = 8-lsb_used;
  else
    _conf.lsb_shift = 0;
  _conf.msb_shift = lsb_used;

  cpr = _powtwo(_bit_resolution);

  wire = &Wire;
}



MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config){
  _conf = config;
  cpr = _powtwo(config.bit_resolution);
  wire = &Wire;
}



MagneticSensorI2C MagneticSensorI2C::AS5600() {
  return {AS5600_I2C};
}



void MagneticSensorI2C::init(TwoWire* _wire){
  wire = _wire;
  wire->begin();  // I2C communication begin
  this->Sensor::init(); // call base class init
}



//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorI2C::getSensorAngle(){
  // (number of full rotations)*2PI + current sensor angle 
  return  ( getRawCount() / (float)cpr) * _2PI ;
}



// I2C functions
/*
* Read an angle from the angle register of the sensor
*/
int MagneticSensorI2C::getRawCount() {
  // read the angle register first MSB then LSB
	byte readArray[2];
	uint16_t readValue = 0;
  // notify the device that is aboout to be read
	wire->beginTransmission(_conf.chip_address);
	wire->write(_conf.angle_register);
  currWireError = wire->endTransmission(false);
  // read the data msb and lsb
	wire->requestFrom(_conf.chip_address, 2);
	for (byte i=0; i < 2; i++) {
		readArray[i] = wire->read();
	}
  readValue = (readArray[0] & _conf.msb_mask) << _conf.msb_shift;
  readValue |= (readArray[1] & _conf.lsb_mask) >> _conf.lsb_shift;
	return readValue;
}

/*
* Checks whether other devices have locked the bus. Can clear SDA locks.
* This should be called before sensor.init() on devices that suffer i2c slaves locking sda
* e.g some stm32 boards with AS5600 chips
* Takes the sda_pin and scl_pin
* Returns 0 for OK, 1 for other master and 2 for unfixable sda locked LOW
*/
int MagneticSensorI2C::checkBus(byte sda_pin, byte scl_pin) {

  pinMode(scl_pin, INPUT_PULLUP);
  pinMode(sda_pin, INPUT_PULLUP);
  delay(250);

  if (digitalRead(scl_pin) == LOW) {
    // Someone else has claimed master!");
    return 1;
  }

  if(digitalRead(sda_pin) == LOW) {
    // slave is communicating and awaiting clocks, we are blocked
    pinMode(scl_pin, OUTPUT);
    for (byte i = 0; i < 16; i++) {
      // toggle clock for 2 bytes of data
      digitalWrite(scl_pin, LOW);
      delayMicroseconds(20);
      digitalWrite(scl_pin, HIGH);
      delayMicroseconds(20);
    }
    pinMode(sda_pin, INPUT);
    delayMicroseconds(20);
    if (digitalRead(sda_pin) == LOW) {
      // SDA still blocked
      return 2;
    }
    _delay(1000);
  }
  // SDA is clear (HIGH)
  pinMode(sda_pin, INPUT);
  pinMode(scl_pin, INPUT);

  return 0;
}
